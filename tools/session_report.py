"""Night report: one interactive page for a whole SteamVR session (fork, docs/DESIGN.md section 13).

Reads what ``tools.analyze_session`` wrote (``--analysis``), the per-tick head tracker error of a
replay of the whole session (``--replay``: one ``spacecal-replay run <segment> --csv`` file per
segment) and the recording itself for the incident close-ups (``--root``). Writes one HTML page
with six graphs, each with a hover explanation:

1. night overview: per device, lost tracking, guard states, flagged glitches, markers
2. head tracker error over the night, split into forward / right / up, and the three views
3. calibration through the night: applied calibration and every solver proposal
4. tracking quality per minute and device (tracking share or flagged glitches)
5. incident close-ups, picked automatically
6. guard tuning: how often the unexplained step exceeds a value, against the jump limit

Plotly is loaded from cdnjs; the data is embedded, so the page works as a published artifact.

Usage::

    python -m tools.session_report 1a2b3c4d --analysis logs/analysis_1a2b3c4d_60hz \\
        --replay logs/replay_csv/1a2b3c4d --out logs/report_1a2b3c4d.html
"""

from __future__ import annotations

import argparse
import html
import json
import math
import os
import struct
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from tools.blackbox import (  # noqa: E402
    CHUNK_EXTENSION,
    HEADER_SIZE,
    HEADER_STRUCT,
    POSE_FLAG_CONNECTED,
    POSE_FLAG_VALID,
    TYPE_POSE,
    TYPE_WFD,
    pose_results,
    read_chunk,
)

PLOTLY = "https://cdnjs.cloudflare.com/ajax/libs/plotly.js/3.1.2/plotly.min.js"
STATE_CODE = {"TRUSTED": 0, "SUSPECT": 1, "UNTRUSTED": 2, "RECOVERING": 3}
STATE_RANK = np.array([0, 2, 3, 1])  # by state code: UNTRUSTED > SUSPECT > RECOVERING > TRUSTED
RANK_STATE = np.array([0, 3, 1, 2])  # back from rank to state code
ROLE_ORDER = ["headset", "head tracker", "hip", "chest", "foot", "knee", "elbow", "right controller", "left controller"]
DEVICE_COLORS = ["#7a8aa0", "#14a394", "#d9822b", "#b5527a", "#9a7a3e", "#4a86c5", "#6aa84f", "#a08cc8", "#b0a090", "#909aa8"]
TICK_HZ = 60.0


# ---- helpers ----------------------------------------------------------------------------------

def r1(x, nd=1):
    """JSON-friendly rounding; NaN/inf become null."""
    if x is None:
        return None
    try:
        xf = float(x)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(xf):
        return None
    return round(xf, nd)


def rl(a, nd=1):
    return [r1(x, nd) for x in a]


def clock(unix: float) -> str:
    return datetime.fromtimestamp(unix).strftime("%H:%M:%S")


def clock_min(unix: float) -> str:
    return datetime.fromtimestamp(unix).strftime("%H:%M")


def fmt_len(m: float) -> str:
    """0.034 -> '3.4 cm', 7.19 -> '7.2 m'"""
    if m >= 1.0:
        return f"{m:.1f} m"
    if m >= 0.1:
        return f"{m * 100:.0f} cm"
    return f"{m * 100:.1f} cm"


def fmt_dur(s: float) -> str:
    if s < 90:
        return f"{s:.0f} s" if s >= 10 else f"{s:.1f} s"
    if s < 5400:
        return f"{s / 60:.0f} min"
    return f"{s / 3600:.1f} h"


def yaw_deg(qw, qx, qy, qz):
    """heading of a rotation around world up (OpenVR: y up)"""
    return np.degrees(np.arctan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qx * qx + qy * qy)))


def qrot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    w = q[:, :1]
    xyz = q[:, 1:]
    t = 2.0 * np.cross(xyz, v)
    return v + w * t + np.cross(xyz, t)


def load_csv(path: Path) -> pd.DataFrame:
    if path.exists() and path.stat().st_size > 0:
        return pd.read_csv(path)
    return pd.DataFrame()


def device_label(role: str, info: dict) -> str:
    model = info.get("model", "")
    serial = info.get("serial", "")
    tail = serial.split("-")[-1][-4:] if serial else ""
    if role == "headset":
        return "Headset"
    if role.startswith("head tracker"):
        return "Head tracker"
    if role == "hip":
        return "Hip"
    if role in ("foot", "knee", "elbow", "chest"):
        return f"{role.capitalize()} {tail}".strip()
    if role.endswith("controller"):
        side = role.split()[0]
        kind = "Index" if "Knuckles" in model else ("Quest" if "Quest" in model else "Controller")
        return f"{kind} {side}"
    return f"{model} {tail}".strip() or "Device"


def role_rank(role: str) -> int:
    for i, r in enumerate(ROLE_ORDER):
        if role.startswith(r):
            return i
    return len(ROLE_ORDER)


def decimate_minmax(x: np.ndarray, y: np.ndarray, n: int):
    """keep each bucket's lowest and highest point so spikes survive; NaN gaps stay gaps"""
    if len(x) <= n:
        return x, y
    edges = np.linspace(0, len(x), n // 2 + 1).astype(int)
    ox, oy = [], []
    for a, b in zip(edges[:-1], edges[1:]):
        if b <= a:
            continue
        seg = y[a:b]
        if np.all(np.isnan(seg)):
            ox.append(x[a])
            oy.append(np.nan)
            continue
        i0 = a + int(np.nanargmin(seg))
        i1 = a + int(np.nanargmax(seg))
        for i in sorted({i0, i1}):
            ox.append(x[i])
            oy.append(y[i])
        if np.isnan(seg).any():
            ox.append(x[a + int(np.flatnonzero(np.isnan(seg))[0])])
            oy.append(np.nan)
    return np.asarray(ox), np.asarray(oy)


# ---- the report data --------------------------------------------------------------------------

class Report:
    def __init__(self, sid: str, analysis: Path, replay: Path | None, root: Path | None):
        self.sid = sid
        self.dir = analysis
        self.summary = json.loads((analysis / "summary.json").read_text(encoding="utf-8"))
        self.t0 = float(self.summary["start"])
        self.t1 = float(self.summary["end"])
        self.params = self.summary.get("params", {})
        self.guard = self._guard_params()
        self.trust = load_csv(analysis / "trust.csv")
        self.cal = load_csv(analysis / "calibrations.csv")
        self.applied = load_csv(analysis / "applied.csv")
        self.minutes = load_csv(analysis / "minutes.csv")
        self.markers = load_csv(analysis / "markers.csv")
        self.vflags = load_csv(analysis / "vflags.csv")
        self.hist = load_csv(analysis / "hist.csv")
        self.losses = load_csv(analysis / "losses.csv")
        if not self.vflags.empty:
            self.vflags = self.vflags[self.vflags["mode"] == "tick"].reset_index(drop=True)
        self.root = root
        self.mono_off = self._mono_offset()
        # SteamVR's first minutes: before the night's first applied calibration the stored one can be far off
        # (a new universe after the SteamVR start); the head tracker charts start there
        self.first_cal = self.t0
        if not self.cal.empty:
            ok = self.cal[self.cal["outcome"].isin(["applied", "forced", "corrected"])]
            if len(ok):
                self.first_cal = float(ok["unix"].min())
        self.head = self._load_replay(replay) if replay else pd.DataFrame()
        self._devices()

    # ---- basics ----
    def _guard_params(self) -> dict:
        p = {"r_ok": 0.03, "r_high": 0.1, "v_unexplained": 1.0, "j_tol": 0.03, "rot_unexplained_dps": 200.0, "rot_tol_deg": 10.0, "dt_max": 0.5}
        gj = Path(os.environ.get("APPDATA", "")) / "space-calibrator" / "guard.json"
        try:
            trust = json.loads(gj.read_text(encoding="utf-8")).get("trust", {})
            for k in p:
                if k in trust:
                    p[k] = float(trust[k])
        except (OSError, ValueError):
            pass
        for k in ("v_unexplained", "j_tol", "rot_unexplained_dps", "rot_tol_deg", "dt_max"):
            if k in self.params:
                p[k] = float(self.params[k])
        return p

    def rel(self, unix):
        return np.asarray(unix, dtype=np.float64) - self.t0

    def _mono_offset(self) -> float:
        for df in (self.trust, self.cal, self.applied):
            if not df.empty and "t" in df and "unix" in df:
                return float(np.median(df["unix"].to_numpy() - df["t"].to_numpy()))
        return 0.0

    def _devices(self):
        roles = {int(k): v for k, v in self.summary.get("roles", {}).items()}
        infos = {int(k): v.get("info", {}) for k, v in self.summary.get("devices", {}).items()}
        ids = [d for d in roles if d in infos]
        ids.sort(key=lambda d: (role_rank(roles[d]), d))
        self.target = next((d for d in ids if roles[d].startswith("head tracker")), None)
        self.devs = []
        for i, d in enumerate(ids):
            self.devs.append({"id": d, "label": device_label(roles[d], infos[d]), "role": roles[d],
                              "model": infos[d].get("model", ""), "serial": infos[d].get("serial", ""),
                              "sys": infos[d].get("sys", ""), "color": DEVICE_COLORS[i % len(DEVICE_COLORS)]})
        self.row = {d["id"]: i for i, d in enumerate(self.devs)}
        self.label = {d["id"]: d["label"] for d in self.devs}
        # tracked hours per device at the guard tick, for per-hour rates
        self.hours = {}
        for d in ids:
            ev = self.summary["devices"][str(d)].get("flags", {}).get("tick", {}).get("evaluated", 0)
            self.hours[d] = max(ev / TICK_HZ / 3600.0, 1e-6)

    def _load_replay(self, folder: Path) -> pd.DataFrame:
        parts = []
        for f in sorted(folder.glob("seg_*.csv")):
            df = pd.read_csv(f)
            if df.empty:
                continue
            df["seg"] = f.stem
            parts.append(df)
        if not parts:
            return pd.DataFrame()
        df = pd.concat(parts, ignore_index=True).sort_values("t", kind="stable")
        df = df.drop_duplicates(subset="t", keep="first")
        df["rel"] = df["t"] + self.mono_off - self.t0
        df["state"] = df["target_state"].map(STATE_CODE).fillna(0).astype(int)
        return df.reset_index(drop=True)

    # ---- 1. overview ----
    def trust_intervals(self):
        out = []
        if self.trust.empty:
            return out
        for d, g in self.trust.sort_values("t").groupby("device"):
            rows = g.to_dict("records")
            for i, r in enumerate(rows):
                if r["new_state"] == "TRUSTED":
                    continue
                end = rows[i + 1]["unix"] if i + 1 < len(rows) else self.t1
                out.append({"dev": int(d), "a": float(r["unix"]), "b": float(end), "state": r["new_state"],
                            "reason": r["reason_name"], "residual": float(r.get("residual_m", 0.0))})
        return out

    def overview(self) -> dict:
        ti = self.trust_intervals()
        trust = [[self.row[t["dev"]], r1(t["a"] - self.t0, 2), r1(t["b"] - self.t0, 2), STATE_CODE.get(t["state"], 0), t["reason"]]
                 for t in ti if t["dev"] in self.row]
        losses = []
        if not self.losses.empty:
            for r in self.losses.itertuples():
                if r.device in self.row:
                    losses.append([self.row[r.device], r1(r.unix - self.t0, 2), r1(r.unix - self.t0 + r.seconds, 2), r1(r.seconds, 2)])
        glitches = []
        if not self.vflags.empty:
            for r in self.vflags.itertuples():
                if r.device in self.row:
                    glitches.append([self.row[r.device], r1(r.unix - self.t0, 2), r1(r.unexplained_m * 100, 1), r1(r.unexplained_rot_deg, 1), r.flags])
        markers = []
        if not self.markers.empty:
            for r in self.markers.itertuples():
                markers.append([r1(r.unix - self.t0, 2), str(r.source)])
        steps = []
        if not self.cal.empty:
            app = self.cal[self.cal["outcome"].isin(["applied", "forced", "corrected"])]
            for r in app.itertuples():
                if r.delta_trans_m >= 0.05:
                    steps.append([r1(r.unix - self.t0, 2), r1(r.delta_trans_m * 100, 1), r1(r.delta_rot_deg, 2), r.outcome, r.trigger])
        return {"trust": trust, "losses": losses, "glitches": glitches, "markers": markers, "steps": steps}

    # ---- 2. head tracker error ----
    def head_data(self) -> dict:
        H = self.head
        if H.empty:
            return {}
        # from the first applied calibration to shortly before the end (SteamVR shutting down leaves stale poses)
        H = H[(H["rel"] >= self.first_cal - self.t0 + 2.0) & (H["rel"] <= self.t1 - self.t0 - 15.0)]
        v = H[H["residual_valid"] == 1]
        sec = np.floor(H["rel"].to_numpy()).astype(np.int64)
        H = H.assign(sec=sec)
        vs = H[H["residual_valid"] == 1]
        g = vs.groupby("sec")["residual_m"]
        med = g.median() * 100
        mx = g.max() * 100
        worst = H.assign(rank=STATE_RANK[H["state"].to_numpy()]).groupby("sec")["rank"].max()
        secs = np.arange(int(max(0, sec.min())), int(sec.max()) + 1)
        med = med.reindex(secs)
        mx = mx.reindex(secs)
        worst = pd.Series(RANK_STATE[worst.reindex(secs).fillna(0).astype(int).to_numpy()], index=secs)
        # state bands (live-equivalent states, merged)
        bands = []
        cur = None
        for s, st in zip(secs, worst.to_numpy()):
            if st != 0:
                if cur and cur[2] == st and s - cur[1] <= 1:
                    cur[1] = s + 1
                else:
                    if cur:
                        bands.append(cur)
                    cur = [int(s), int(s) + 1, int(st)]
            elif cur:
                bands.append(cur)
                cur = None
        if cur:
            bands.append(cur)
        # forward / right / up, 10 s medians
        b10 = (np.floor(vs["rel"].to_numpy() / 10.0) * 10).astype(np.int64)
        d10 = vs.assign(b=b10).groupby("b")[["err_forward_m", "err_right_m", "err_up_m"]].median() * 100
        allb = np.arange(int(d10.index.min()), int(d10.index.max()) + 10, 10) if len(d10) else np.array([])
        d10 = d10.reindex(allb)
        # where the head tracker sat: three views
        comp = v[["err_forward_m", "err_right_m", "err_up_m"]].dropna().to_numpy() * 100
        lim = 12.0
        if len(comp):
            lim = float(np.clip(np.nanpercentile(np.abs(comp), 99.9) * 1.1, 12.0, 40.0))
        step = 0.5 if lim <= 20 else 1.0
        edges = np.arange(-lim, lim + step / 2, step)
        views = {}
        if len(comp):
            fwd, right, up = comp[:, 0], comp[:, 1], comp[:, 2]
            for name, (xx, yy) in {"top": (right, fwd), "side": (fwd, up), "back": (right, up)}.items():
                h2, _, _ = np.histogram2d(yy, xx, bins=[edges, edges])  # rows = y
                views[name] = [[int(c) for c in row] for row in h2]
        res = v["residual_m"].to_numpy()
        share = {}
        if len(res):
            share = {"ok": r1(100 * np.mean(res < self.guard["r_ok"]), 1),
                     "mid": r1(100 * np.mean((res >= self.guard["r_ok"]) & (res < self.guard["r_high"])), 1),
                     "high": r1(100 * np.mean(res >= self.guard["r_high"]), 1),
                     "high_min": r1(np.sum(res >= self.guard["r_high"]) / TICK_HZ / 60.0, 1),
                     "p50": r1(np.median(res) * 100, 2), "p95": r1(np.percentile(res, 95) * 100, 1), "max": r1(res.max() * 100, 1)}
        seg_starts = [r1(x, 1) for x in self.head.groupby("seg")["rel"].min().to_numpy()]
        ymax = float(np.clip(np.nanpercentile(mx.to_numpy(), 99.9) * 1.15 if mx.notna().any() else 0.0, self.guard["r_high"] * 150, 60.0))
        med30 = med.rolling(31, center=True, min_periods=5).median()  # the trend: a 30-second running median
        dirs = d10.to_numpy()
        dlim = float(np.clip(np.nanpercentile(np.abs(dirs), 99.5) * 1.3, 3.0, 30.0)) if np.isfinite(dirs).any() else 10.0
        return {"t0": int(secs[0]), "med": rl(med30.to_numpy(), 2), "max": rl(mx.to_numpy(), 1), "bands": bands, "ymax": r1(ymax, 1), "dlim": r1(dlim, 1),
                "from": r1(self.first_cal - self.t0, 1),
                "dir_t0": int(allb[0]) if len(allb) else 0, "fwd": rl(d10["err_forward_m"].to_numpy(), 2),
                "right": rl(d10["err_right_m"].to_numpy(), 2), "up": rl(d10["err_up_m"].to_numpy(), 2),
                "views": views, "edges": rl(edges, 2), "share": share, "seg_starts": seg_starts}

    # ---- 3. calibration ----
    def cal_data(self) -> dict:
        if self.applied.empty or self.target is None:
            return {}
        a = self.applied[self.applied["device"] == self.target].sort_values("unix")
        if "carried" in a:
            a = a[~a["carried"].astype(str).str.lower().eq("true")]
        first_ok = None
        if not self.cal.empty:
            ok = self.cal[self.cal["outcome"].isin(["applied", "forced", "corrected"])].sort_values("unix")
            if len(ok):
                first_ok = float(ok["unix"].iloc[0])
        if first_ok is not None:
            a = a[a["unix"] >= first_ok - 0.5]
        if a.empty:
            return {}
        yaw = yaw_deg(a["qw"].to_numpy(), a["qx"].to_numpy(), a["qy"].to_numpy(), a["qz"].to_numpy())
        ref = {"x": float(a["tx"].median()), "y": float(a["ty"].median()), "z": float(a["tz"].median()), "yaw": float(np.median(yaw))}
        dyaw = (yaw - ref["yaw"] + 180.0) % 360.0 - 180.0
        out = {"t": rl(self.rel(a["unix"]), 2), "x": rl((a["tx"] - ref["x"]) * 100, 2), "y": rl((a["ty"] - ref["y"]) * 100, 2),
               "z": rl((a["tz"] - ref["z"]) * 100, 2), "yaw": rl(dyaw, 3), "ref": {k: r1(v, 4) for k, v in ref.items()},
               "end": r1(self.t1 - self.t0, 1)}
        c = self.cal.sort_values("unix")
        reason = np.where(c["outcome"].isin(["applied", "forced", "corrected"]), c["outcome"], c["error_name"].fillna("none"))
        later = c[c["unix"] > self.first_cal + 1.0]["delta_trans_m"] * 100
        out["p_ymax"] = r1(max(30.0, float(np.percentile(later, 99.5)) * 1.3) if len(later) else 50.0, 1)
        out["p"] = {"t": rl(self.rel(c["unix"]), 2), "cm": rl(np.maximum(c["delta_trans_m"] * 100, 0.01), 2),
                    "deg": rl(c["delta_rot_deg"], 2), "rms": rl(c["rms_m"] * 1000, 1), "outcome": c["outcome"].tolist(),
                    "reason": [str(x) for x in reason], "trigger": c["trigger"].tolist()}
        if first_ok is not None:
            out["skipped_before"] = r1(first_ok - self.t0, 1)
        return out

    # ---- 4. heatmap ----
    def heat_data(self) -> dict:
        M = self.minutes
        if M.empty:
            return {}
        start_min = int(self.t0 // 60)
        start_of_day = int(datetime.fromtimestamp(start_min * 60).strftime("%H")) * 60 + int(datetime.fromtimestamp(start_min * 60).strftime("%M"))
        hm = M["minute"].str.split(":", expand=True).astype(int)
        mod = (hm[0] * 60 + hm[1] - start_of_day) % 1440
        M = M.assign(m=mod.to_numpy())
        n_min = int(M["m"].max()) + 1
        rows = [d["id"] for d in self.devs]
        track = [[None] * n_min for _ in rows]
        glitch = [[0] * n_min for _ in rows]
        height = [[None] * n_min for _ in rows]
        speed = [[None] * n_min for _ in rows]
        for r in M.itertuples():
            if r.device not in self.row:
                continue
            i = self.row[r.device]
            track[i][r.m] = r1(r.tracking_frac * 100, 0)
            height[i][r.m] = r1(r.mean_y, 2)
            speed[i][r.m] = r1(r.mean_speed, 2)
        if not self.vflags.empty:
            vm = ((self.vflags["unix"] // 60).astype(int) - start_min).to_numpy()
            for dev, m in zip(self.vflags["device"].to_numpy(), vm):
                if dev in self.row and 0 <= m < n_min:
                    glitch[self.row[dev]][m] += 1
        return {"m0": r1(start_min * 60 - self.t0, 1), "n": n_min, "track": track, "glitch": glitch, "height": height, "speed": speed}

    # ---- 6. tuning ----
    def tuning_data(self) -> dict:
        Hh = self.hist
        # the allowance each tick would have needed not to be flagged: (unexplained - tolerance) / dt.
        # A curve read at x gives the flags per hour the guard would raise with its allowance set to x.
        out = {"pos": [], "rot": [],
               "limit_cm": r1((self.guard["v_unexplained"] / TICK_HZ + self.guard["j_tol"]) * 100, 2),
               "v_limit": r1(self.guard["v_unexplained"], 3), "rot_limit": r1(self.guard["rot_unexplained_dps"], 1),
               "j_tol_cm": r1(self.guard["j_tol"] * 100, 1), "rot_tol": r1(self.guard["rot_tol_deg"], 1)}
        if Hh.empty:
            return out
        Hh = Hh[Hh["mode"] == "tick"]
        for d in self.devs:
            if self.hours[d["id"]] < 0.25:
                continue  # barely tracked (a controller lying in a corner): per-hour rates mean nothing
            for metric, key, scale, floor in (("need_v_unexplained", "pos", 1.0, 0.02), ("need_rot_unexplained", "rot", 1.0, 2.0)):
                g = Hh[(Hh["device"] == d["id"]) & (Hh["metric"] == metric)].sort_values("lo")
                if g.empty:
                    continue
                lo = g["lo"].to_numpy() * scale
                cnt = g["count"].to_numpy().astype(np.float64)
                above = np.cumsum(cnt[::-1])[::-1]  # values >= lo of each bin
                keep = lo >= floor
                rate = above / self.hours[d["id"]]
                out[key].append({"id": d["id"], "x": rl(lo[keep], 4), "y": rl(rate[keep], 3)})
        return out

    # ---- 5. incidents ----
    def incident_candidates(self):
        cands = []
        settle = self.t0 + 180.0  # SteamVR's first minutes: universes settle, the first solve moves everything
        if not self.vflags.empty:
            for dev, g in self.vflags.sort_values("unix").groupby("device"):
                u = g["unix"].to_numpy()
                ue = g["unexplained_m"].to_numpy()
                start = 0
                for i in range(1, len(u) + 1):
                    if i == len(u) or u[i] - u[i - 1] > 1.0:
                        m = float(ue[start:i].max())
                        if m >= 0.15 and u[start] > settle:
                            cands.append({"t": float(u[start]), "t_end": float(u[i - 1]), "score": m / 0.25, "kind": "glitch", "dev": int(dev), "size": m})
                        start = i
        if not self.cal.empty:
            app = self.cal[self.cal["outcome"].isin(["applied", "forced", "corrected"])]
            for r in app.itertuples():
                if r.delta_trans_m >= 0.10 and r.unix > settle:
                    cands.append({"t": float(r.unix), "t_end": float(r.unix), "score": r.delta_trans_m / 0.1, "kind": "calibration", "size": float(r.delta_trans_m), "deg": float(r.delta_rot_deg), "trigger": r.trigger})
        if not self.markers.empty:
            for r in self.markers.itertuples():
                if str(r.source) != "auto":
                    # a marker you set yourself is always worth a look
                    cands.append({"t": float(r.unix), "t_end": float(r.unix), "score": 10.0, "kind": "marker", "source": str(r.source)})
        if not self.losses.empty:
            L = self.losses[(self.losses["seconds"] >= 0.5) & self.losses["device"].isin(self.row.keys())].sort_values("unix")
            u = L["unix"].to_numpy()
            dv = L["device"].to_numpy()
            i = 0
            while i < len(u):
                j = i
                while j + 1 < len(u) and u[j + 1] - u[i] <= 1.5:
                    j += 1
                devs = set(dv[i:j + 1].tolist())
                if len(devs) >= 3 and u[i] > settle:
                    cands.append({"t": float(u[i]), "t_end": float(u[j]), "score": 1.5 + 0.5 * len(devs), "kind": "dropout", "devs": sorted(devs)})
                i = j + 1
        if not self.trust.empty and self.target is not None:
            T = self.trust[(self.trust["device"] == self.target) & (self.trust["new_state"] == "UNTRUSTED")]
            for r in T.itertuples():
                if r.unix > settle:
                    cands.append({"t": float(r.unix), "t_end": float(r.unix), "score": 2.0, "kind": "head_untrusted", "reason": r.reason_name})
        return sorted(cands, key=lambda c: c["t"])

    def incidents(self, max_n: int = 8):
        cands = self.incident_candidates()
        clusters = []
        for c in cands:
            if clusters and c["t"] - clusters[-1]["t_end"] <= 45.0:
                cl = clusters[-1]
                cl["items"].append(c)
                cl["t_end"] = max(cl["t_end"], c["t_end"])
            else:
                clusters.append({"t": c["t"], "t_end": c["t_end"], "items": [c]})
        for cl in clusters:
            s = sorted((i["score"] for i in cl["items"]), reverse=True)
            cl["score"] = s[0] + 0.3 * sum(s[1:4])
        top = sorted(clusters, key=lambda c: -c["score"])[:max_n]
        top.sort(key=lambda c: c["t"])
        chunks = self._chunk_index() if self.root else {}
        out = []
        for cl in top:
            w0 = cl["t"] - 12.0
            w1 = min(cl["t_end"] + 18.0, w0 + 75.0)
            out.append(self._incident(cl, w0, w1, chunks))
        return out

    def _headline(self, cl) -> str:
        best = max(cl["items"], key=lambda i: i["score"])
        k = best["kind"]
        if k == "glitch":
            return f"{self.label.get(best['dev'], 'Device')} jumped {fmt_len(best['size'])}"
        if k == "calibration":
            return f"Calibration moved {fmt_len(best['size'])}"
        if k == "marker":
            return f"Marker set ({best['source']})"
        if k == "dropout":
            return f"{len(best['devs'])} devices lost tracking"
        return "Head tracker untrusted"

    def _incident(self, cl, w0: float, w1: float, chunks: dict) -> dict:
        center = cl["t"]
        inc = {"t": r1(center - self.t0, 2), "clock": clock(center), "title": self._headline(cl), "w0": r1(w0 - center, 2), "w1": r1(w1 - center, 2)}
        facts = []
        # head tracker error in the window
        H = self.head
        if not H.empty:
            a, b = w0 - self.t0, w1 - self.t0
            h = H[(H["rel"] >= a) & (H["rel"] <= b)]
            if len(h):
                err = np.where(h["residual_valid"].to_numpy() == 1, h["residual_m"].to_numpy() * 100, np.nan)
                x = h["rel"].to_numpy() - (center - self.t0)
                inc["head"] = {"x": rl(x[::2], 2), "err": rl(err[::2], 2), "state": h["state"].to_numpy()[::2].tolist()}
                if np.isfinite(err).any():
                    i = int(np.nanargmax(err))
                    facts.append(f"Head tracker error peaked at {err[i]:.1f} cm ({x[i]:+.1f} s).")
        # glitches
        if not self.vflags.empty:
            g = self.vflags[(self.vflags["unix"] >= w0) & (self.vflags["unix"] <= w1)]
            for dev, gg in g.groupby("device"):
                m = gg["unexplained_m"].max()
                facts.append(f"{self.label.get(int(dev), dev)}: {len(gg)} flagged tick(s), biggest unexplained jump {fmt_len(m)} (limit {self.guard['v_unexplained'] / TICK_HZ * 100 + self.guard['j_tol'] * 100:.1f} cm).")
        # guard states
        if not self.trust.empty:
            tr = self.trust[(self.trust["unix"] >= w0) & (self.trust["unix"] <= w1)].sort_values("unix")
            for dev, gg in tr.groupby("device"):
                chain = [gg["old_state"].iloc[0]] + gg["new_state"].tolist()
                back = gg[gg["new_state"] == "TRUSTED"]
                tail = ""
                if len(back) and chain[0] == "TRUSTED":
                    tail = f", trusted again after {back['unix'].iloc[-1] - gg['unix'].iloc[0]:.1f} s"
                facts.append(f"Guard: {self.label.get(int(dev), dev)} {' > '.join(s.lower() for s in chain)}{tail}.")
            inc["trust"] = [[r1(r.unix - center, 2), self.label.get(int(r.device), str(r.device)), r.old_state, r.new_state, r.reason_name] for r in tr.itertuples()]
        # calibrations
        if not self.cal.empty:
            c = self.cal[(self.cal["unix"] >= w0) & (self.cal["unix"] <= w1)]
            app = c[c["outcome"].isin(["applied", "forced", "corrected"])]
            for r in app.itertuples():
                facts.append(f"Calibration {r.outcome} ({r.trigger}) at {r.unix - center:+.1f} s moved the lighthouse world {fmt_len(r.delta_trans_m)} and {r.delta_rot_deg:.1f} deg.")
            rej = c[c["outcome"] == "rejected"]
            if len(rej):
                reasons = rej["error_name"].value_counts()
                facts.append(f"{len(rej)} calibration(s) rejected: " + ", ".join(f"{k.replace('_', ' ')} {v}" for k, v in reasons.items()) + ".")
            inc["cal"] = [[r1(r.unix - center, 2), r.outcome, r1(r.delta_trans_m * 100, 1), r1(r.delta_rot_deg, 2), r.trigger, str(r.error_name)] for r in c.itertuples()]
        # lost tracking
        if not self.losses.empty:
            L = self.losses[(self.losses["unix"] >= w0 - 5) & (self.losses["unix"] <= w1) & self.losses["device"].isin(self.row.keys())]
            for dev, gg in L.groupby("device"):
                facts.append(f"{self.label.get(int(dev), dev)} lost tracking {len(gg)}x, {gg['seconds'].sum():.1f} s in total.")
        if not self.markers.empty:
            mk = self.markers[(self.markers["unix"] >= w0) & (self.markers["unix"] <= w1)]
            for r in mk.itertuples():
                facts.append(f"Marker ({r.source}) at {r.unix - center:+.1f} s.")
            inc["markers"] = [[r1(r.unix - center, 2), str(r.source)] for r in mk.itertuples()]
        inc["facts"] = facts
        if chunks:
            try:
                inc["devices"] = self._device_traces(chunks, w0, w1, center)
            except Exception as exc:  # a damaged chunk only costs the device traces
                inc["devices"] = []
                inc["facts"].append(f"(device traces unavailable: {exc})")
        return inc

    # ---- chunks for the close-ups ----
    def _chunk_index(self) -> dict:
        best = {}
        for f in self.root.rglob(f"{self.sid}_*{CHUNK_EXTENSION}"):
            try:
                idx = int(f.stem.split("_")[1])
                size = f.stat().st_size
            except (ValueError, IndexError, OSError):
                continue
            if idx not in best or size > best[idx][1]:
                best[idx] = (f, size)
        out = {}
        for idx, (f, _) in best.items():
            try:
                with open(f, "rb") as fh:
                    hdr = HEADER_STRUCT.unpack(fh.read(HEADER_SIZE))
                out[idx] = (f, float(hdr[7]))  # unix_start
            except (OSError, struct.error):
                continue
        return out

    def _device_traces(self, chunks: dict, w0: float, w1: float, center: float):
        sel = [f for idx, (f, us) in sorted(chunks.items()) if us <= w1 and us + 75.0 >= w0 - 2.0]
        recs, unix = [], []
        for f in sel:
            c = read_chunk(f)
            r = c.records
            recs.append(r)
            unix.append(c.header.mono_to_unix(r["t"]))
        if not recs:
            return []
        r = np.concatenate(recs)
        u = np.concatenate(unix)
        o = np.argsort(u, kind="stable")
        r, u = r[o], u[o]
        # applied calibration per device, from applied.csv (carried rows included)
        A = self.applied
        out = []
        for d in self.devs:
            dev = d["id"]
            wf = r[(r["type"] == TYPE_WFD) & (r["device"] == dev)]
            wfu = u[(r["type"] == TYPE_WFD) & (r["device"] == dev)]
            m = (r["type"] == TYPE_POSE) & (r["device"] == dev) & (u >= w0 - 1.0) & (u <= w1)
            p = r[m]
            pu = u[m]
            if len(p) < 2:
                continue
            v = p["v"].astype(np.float64)
            pos, q, vel = v[:, 0:3], v[:, 3:7], v[:, 7:10]
            trk = ((p["b"] & POSE_FLAG_VALID) != 0) & ((p["b"] & POSE_FLAG_CONNECTED) != 0) & (pose_results(p) == 200)
            if len(wf):
                wv = wf["v"].astype(np.float64)
                k = np.clip(np.searchsorted(wfu, pu, side="right") - 1, 0, len(wf) - 1)
                wq = wv[k, 3:7] / np.maximum(np.linalg.norm(wv[k, 3:7], axis=1, keepdims=True), 1e-12)
                pos = wv[k, 0:3] + qrot(wq, pos)
                vel = qrot(wq, vel)
            raw = pos.copy()
            if not A.empty:
                a = A[(A["device"] == dev) & (A["unix"] <= w1)].sort_values("unix")
                if len(a):
                    au = a["unix"].to_numpy()
                    k = np.clip(np.searchsorted(au, pu, side="right") - 1, 0, len(a) - 1)
                    aq = a[["qw", "qx", "qy", "qz"]].to_numpy()[k]
                    aq = aq / np.maximum(np.linalg.norm(aq, axis=1, keepdims=True), 1e-12)
                    pos = a[["tx", "ty", "tz"]].to_numpy()[k] + qrot(aq, pos)
            x = pu - center
            hgt = np.where(trk, pos[:, 1], np.nan)
            hx, hy = decimate_minmax(x, hgt, 700)
            # unexplained step at the guard's tick, as the overlay sees it (latest sample per tick)
            ticks = np.arange(max(w0, pu[0]), min(w1, pu[-1]), 1.0 / TICK_HZ)
            ti = np.searchsorted(pu, ticks, side="right") - 1
            ok = (ti >= 0) & trk[np.clip(ti, 0, len(trk) - 1)]
            ti, tt = ti[ok], ticks[ok]
            keep = np.r_[True, np.diff(ti) != 0]
            ti, tt = ti[keep], tt[keep]
            ux, uy = [], []
            if len(ti) > 1:
                dt = np.diff(tt)
                dP = np.diff(raw[ti], axis=0)
                vb = 0.5 * (vel[ti][:-1] + vel[ti][1:])
                vn = np.linalg.norm(vb, axis=1)
                vb = vb * np.where(vn > 10.0, 10.0 / np.maximum(vn, 1e-12), 1.0)[:, None]
                has_v = (np.linalg.norm(vel[ti][:-1], axis=1) > 0) | (np.linalg.norm(vel[ti][1:], axis=1) > 0)
                une = np.where(has_v, np.linalg.norm(dP - vb * dt[:, None], axis=1), np.linalg.norm(dP, axis=1))
                une = np.where(dt <= self.guard["dt_max"], une, np.nan)
                ux = tt[1:] - center
                uy = np.maximum(une * 100, 0.01)
                # a gap (lost tracking) stays a gap instead of a line across it
                gap = np.r_[False, np.diff(tt[1:]) > 2.5 / TICK_HZ]
                if gap.any():
                    at = np.flatnonzero(gap)
                    ux = np.insert(ux, at, ux[at] - 0.5 / TICK_HZ)
                    uy = np.insert(uy, at, np.nan)
                if len(ux) > 900:
                    ux, uy = decimate_minmax(ux, uy, 900)
            out.append({"id": dev, "hx": rl(hx, 3), "h": rl(hy, 3), "ux": rl(ux, 3), "u": rl(uy, 2)})
        return out

    # ---- headline numbers and findings ----
    def stats_and_findings(self, head: dict, ov: dict, cal: dict):
        st = {}
        dur = self.t1 - self.t0
        st["duration"] = fmt_dur(dur)
        st["start"] = datetime.fromtimestamp(self.t0).strftime("%a %d %b %Y, %H:%M")
        st["end"] = clock_min(self.t1)
        st["glitch_ticks"] = len(self.vflags)
        findings = []
        # biggest glitch
        if not self.vflags.empty:
            v = self.vflags.loc[self.vflags["unexplained_m"].idxmax()]
            after = self.cal[(self.cal["unix"] >= v["unix"]) & (self.cal["unix"] <= v["unix"] + 15) &
                             self.cal["outcome"].isin(["applied", "forced", "corrected"]) & (self.cal["delta_trans_m"] >= 0.05)] if not self.cal.empty else []
            moved = "and no calibration step of more than 5 cm followed within 15 s" if len(after) == 0 else f"and a calibration step of {fmt_len(after['delta_trans_m'].max())} followed"
            findings.append(f"Biggest glitch: {self.label.get(int(v['device']), v['device'])} jumped {fmt_len(v['unexplained_m'])} at {clock(v['unix'])} that its own velocity did not explain, {moved}.")
        # head tracker
        sh = head.get("share", {}) if head else {}
        if sh:
            st["head_ok"] = sh["ok"]
            st["head_high_min"] = sh["high_min"]
            findings.append(f"The head tracker sat within {self.guard['r_ok'] * 100:.0f} cm of where the headset expected it {sh['ok']:.0f}% of the night (median {sh['p50']:.1f} cm, 95% under {sh['p95']:.1f} cm) and was off by more than {self.guard['r_high'] * 100:.0f} cm for {sh['high_min']:.1f} min in total.")
        # guard on the head tracker (live)
        ti = self.trust_intervals()
        if self.target is not None:
            out_s = sum(t["b"] - t["a"] for t in ti if t["dev"] == self.target)
            st["head_untrusted"] = fmt_dur(out_s)
            if not self.head.empty:
                hh = self.head[(self.head["rel"] >= self.first_cal - self.t0 + 2.0) & (self.head["rel"] <= self.t1 - self.t0 - 15.0)]
                rep_s = float((hh["state"] != 0).sum()) / TICK_HZ
                st["head_untrusted_replay"] = fmt_dur(rep_s)
                findings.append(f"The live guard did not trust the head tracker for {fmt_dur(out_s)} that night. Replayed with today's guard, "
                                f"which re-anchors after calibration steps, it would have been {fmt_dur(rep_s)}.")
        # calibrations
        if not self.cal.empty:
            c = self.cal
            app = c[c["outcome"].isin(["applied", "forced", "corrected"])]
            rej = c[c["outcome"] == "rejected"]
            st["cal_applied"] = int(len(app))
            st["cal_rejected"] = int(len(rej))
            big = app[(app["delta_trans_m"] >= 0.10) & (app["unix"] > self.t0 + 180)]
            reasons = rej["error_name"].value_counts()
            txt = f"The solver applied {len(app)} calibrations and rejected {len(rej)}"
            if len(reasons):
                txt += f" (mostly {reasons.index[0].replace('_', ' ')}, {reasons.iloc[0]}x)"
            if len(big):
                txt += f"; {len(big)} applied steps moved the lighthouse world by 10 cm or more, between {clock_min(big['unix'].min())} and {clock_min(big['unix'].max())}"
            findings.append(txt + ".")
        # tracking losses
        if not self.losses.empty:
            L = self.losses[self.losses["device"].isin(self.row.keys())]
            if len(L):
                tot = L.groupby("device")["seconds"].agg(["sum", "count", "max"]).sort_values("sum", ascending=False)
                d0 = int(tot.index[0])
                lmax = L[L["device"] == d0].sort_values("seconds").iloc[-1]
                findings.append(f"{self.label.get(d0, d0)} lost tracking most: {fmt_dur(tot.iloc[0]['sum'])} in {int(tot.iloc[0]['count'])} stretches, the longest {fmt_dur(lmax['seconds'])} at {clock(lmax['unix'])}.")
                st["losses"] = int(len(L))
        drops = [c for c in self.incident_candidates() if c["kind"] == "dropout"]
        if drops:
            findings.append(f"{len(drops)} times three or more devices lost tracking within 1.5 s of each other, first at {clock(drops[0]['t'])}. That points at something shared (a base station, the playspace, an occlusion) rather than one tracker.")
        st["dropouts"] = len(drops)
        st["markers_auto"] = int((self.markers["source"] == "auto").sum()) if not self.markers.empty else 0
        st["markers_user"] = int((self.markers["source"] != "auto").sum()) if not self.markers.empty else 0
        return st, findings

    def build(self) -> dict:
        ov = self.overview()
        head = self.head_data()
        cal = self.cal_data()
        stats, findings = self.stats_and_findings(head, ov, cal)
        off = datetime.fromtimestamp(self.t0).astimezone().utcoffset()
        wall0 = (self.t0 + (off.total_seconds() if off else 0.0)) * 1000.0
        return {
            "session": self.sid, "wall0": wall0, "end": r1(self.t1 - self.t0, 1),
            "title": self.title(), "generated": datetime.now().strftime("%Y-%m-%d %H:%M"),
            "guard": {k: r1(v, 4) for k, v in self.guard.items()}, "tick_hz": TICK_HZ,
            "devices": self.devs, "target": self.target, "stats": stats, "findings": findings,
            "overview": ov, "head": head, "cal": cal, "heat": self.heat_data(),
            "incidents": self.incidents(), "tuning": self.tuning_data(),
        }

    def title(self) -> str:
        d = datetime.fromtimestamp(self.t0)
        if d.hour >= 17:
            return f"Night of {d.day} {d.strftime('%b %Y')}"
        return f"Session of {d.day} {d.strftime('%b %Y')}"


# ---- page -------------------------------------------------------------------------------------

def _plain(o):
    """numpy scalars -> Python numbers for json"""
    if hasattr(o, "item"):
        return o.item()
    raise TypeError(f"not JSON serializable: {type(o).__name__}")


def render(data: dict, template: str) -> str:
    blob = json.dumps(data, separators=(",", ":"), allow_nan=False, default=_plain).replace("</", "<\\/")
    return (template.replace("{{TITLE}}", html.escape(data["title"]))
            .replace("{{PLOTLY}}", PLOTLY)
            .replace("{{DATA}}", blob))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="session id, e.g. 1a2b3c4d")
    ap.add_argument("--analysis", required=True, help="folder written by tools.analyze_session")
    ap.add_argument("--replay", default=None, help="folder with spacecal-replay --csv files (seg_*.csv)")
    ap.add_argument("--root", default=str(Path(os.environ.get("APPDATA", "")) / "space-calibrator" / "blackbox"),
                    help="black box folder, for the incident close-ups ('' to skip)")
    ap.add_argument("--out", required=True)
    ap.add_argument("--json", default=None, help="also write the page data here")
    args = ap.parse_args(argv)
    root = Path(args.root) if args.root and Path(args.root).exists() else None
    rep = Report(args.session, Path(args.analysis), Path(args.replay) if args.replay else None, root)
    data = rep.build()
    template = (Path(__file__).resolve().parent / "session_report.html").read_text(encoding="utf-8")
    Path(args.out).write_text(render(data, template), encoding="utf-8")
    if args.json:
        Path(args.json).write_text(json.dumps(data, indent=1, default=_plain), encoding="utf-8")
    print(f"wrote {args.out} ({Path(args.out).stat().st_size / 1e6:.1f} MB), {len(data['incidents'])} incidents")
    return 0


if __name__ == "__main__":
    sys.exit(main())
