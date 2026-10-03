"""Whole-session analysis of a black-box recording (fork, docs/DESIGN.md sections 4 and 13).

Streams every chunk of one SteamVR session (keep-all archive, saved events and live/, the most
complete copy of each chunk index wins) and writes, into ``--out``:

* ``report.txt``      human-readable summary
* ``minutes.csv``     one row per minute and device: tracking, height, speeds, plausibility flags
* ``flags.csv``       every sample the offline plausibility check flags (capped per device)
* ``vflags.csv``      every sample the velocity-aware rule (the one the overlay uses) flags
* ``hist.csv``        per device: histogram of the unexplained step and rotation (100 bins per decade)
* ``losses.csv``      every stretch without a tracking pose longer than t_stale, per device
* ``summary.json``    the numbers behind the report

The plausibility check mirrors ``src/overlay/trust/plausibility.cpp`` on poses decimated to the
overlay's tick (``--tick-hz``, default 90), with WorldFromDriver applied. It reads the thresholds
from guard.json, so the flag counts show what the live trust layer would see with them.

Usage::

    python -m tools.analyze_session 1a2b3c4d --out logs/analysis_1a2b3c4d
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from tools.blackbox import (  # noqa: E402
    CHUNK_EXTENSION,
    POSE_FLAG_CONNECTED,
    POSE_FLAG_VALID,
    RECORD_DTYPE,
    TYPE_APPLIED,
    TYPE_CALIBRATION,
    TYPE_DEVICE_STATE,
    TYPE_LIFECYCLE,
    TYPE_MARKER,
    TYPE_POSE,
    TYPE_SESSION,
    TYPE_TRUST,
    TYPE_WFD,
    _assemble_texts,
    _parse_device_info,
    pose_results,
    read_chunk,
    records_to_frame,
)

RUNNING_OK = 200
EVENT_TYPES = [TYPE_APPLIED, TYPE_TRUST, TYPE_MARKER, TYPE_SESSION, TYPE_CALIBRATION, TYPE_DEVICE_STATE, TYPE_LIFECYCLE]
DEFAULT_PARAMS = dict(v_max=4.0, j_tol=0.03, v_err_max=1.0, a_max=60.0, rot_max_dps=400.0, rot_tol_deg=10.0, t_stale=0.25, dt_max=0.5,
                      v_unexplained=1.0, rot_unexplained_dps=200.0, v_body_max=10.0, w_body_max_dps=3000.0)
HIST_EDGES = np.logspace(-5, 4, 901)  # 100 bins per decade
METRICS = ["speed_reported", "speed_observed", "v_err", "accel", "rot_rate_dps", "jump_margin_m",
           "unexplained_m", "unexplained_rot_deg", "need_v_unexplained", "need_rot_unexplained"]
# velocity-aware rule (src/overlay/trust/plausibility.cpp, the rule the overlay uses since 2026-09-30):
# the part of a step the device's own reported velocity does not explain. The flag tables above
# ("pos_jump", "rot_jump", "v_err", "accel") keep the older fixed bounds for comparison.
V_UNEXPLAINED_CANDIDATES = (0.5, 1.0, 1.5, 2.0)  # m/s, besides the configured one
ROT_UNEXPLAINED_CANDIDATES = (100.0, 200.0, 400.0)  # deg/s
FLAG_POS_JUMP, FLAG_ROT_JUMP, FLAG_VERR, FLAG_ACCEL = 1, 2, 4, 8
FLAG_NAMES = {FLAG_POS_JUMP: "pos_jump", FLAG_ROT_JUMP: "rot_jump", FLAG_VERR: "v_err", FLAG_ACCEL: "accel"}
MAX_FLAG_ROWS = 20000
MODES = ("tick", "pose")


def clock(unix_s: float) -> str:
    return datetime.fromtimestamp(float(unix_s)).strftime("%H:%M:%S")


def clock_min(minute: int) -> str:
    return datetime.fromtimestamp(minute * 60).strftime("%H:%M")


def qmul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        axis=1,
    )


def qrot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    w = q[:, :1]
    xyz = q[:, 1:]
    t = 2.0 * np.cross(xyz, v)
    return v + w * t + np.cross(xyz, t)


def load_params() -> dict:
    params = dict(DEFAULT_PARAMS)
    cfg = Path(os.environ.get("APPDATA", "")) / "space-calibrator" / "guard.json"
    try:
        trust = json.loads(cfg.read_text(encoding="utf-8")).get("trust", {})
        for k in params:
            if k in trust:
                params[k] = float(trust[k])
    except (OSError, json.JSONDecodeError):
        pass
    return params


def collect_chunk_files(root: Path, session: str) -> dict:
    """chunk index -> (path, size, folder label); the largest copy of an index wins."""
    best: dict = {}
    for f in root.rglob(f"{session}_*{CHUNK_EXTENSION}"):
        try:
            idx = int(f.stem.split("_")[1])
            size = f.stat().st_size
        except (ValueError, IndexError, OSError):
            continue
        label = f.parent.name if f.parent.parent.name in ("events", "sessions") else f.parent.name
        if idx not in best or size > best[idx][1]:
            best[idx] = (f, size, label)
    return dict(sorted(best.items()))


class Hist:
    def __init__(self):
        self.counts = np.zeros(len(HIST_EDGES) + 1, dtype=np.int64)
        self.n = 0
        self.max = -np.inf
        self.max_unix = 0.0

    def add(self, values: np.ndarray, unix: np.ndarray):
        finite = np.isfinite(values)
        values, unix = values[finite], unix[finite]
        if len(values) == 0:
            return
        idx = np.searchsorted(HIST_EDGES, values, side="right")
        self.counts += np.bincount(idx, minlength=len(self.counts))
        self.n += len(values)
        i = int(np.argmax(values))
        if values[i] > self.max:
            self.max = float(values[i])
            self.max_unix = float(unix[i])

    def pct(self, p: float) -> float:
        if self.n == 0:
            return float("nan")
        target = p / 100.0 * self.n
        cum = np.cumsum(self.counts)
        i = int(np.searchsorted(cum, target))
        if i >= len(HIST_EDGES):
            return self.max
        return float(HIST_EDGES[i])

    def frac_above(self, x: float) -> float:
        if self.n == 0:
            return float("nan")
        i = int(np.searchsorted(HIST_EDGES, x, side="right"))
        return float(self.counts[i + 1 :].sum()) / self.n


class DeviceState:
    def __init__(self, index: int):
        self.index = index
        self.n = 0
        self.n_tracking = 0
        self.results: Counter = Counter()
        self.first_unix = None
        self.last_unix = None
        self.last_t = None
        self.last_track_t = None
        self.gaps = Counter()  # pose-stream gaps by threshold
        self.long_gaps = []  # (unix, seconds) of pose gaps > 1 s
        self.track_losses = []  # (unix, seconds) without a tracking pose for > 0.25 s
        self.max_gap = (0.0, 0.0)
        self.hist = {mode: {m: Hist() for m in METRICS} for mode in MODES}
        self.flag_counts = {mode: Counter() for mode in MODES}
        self.flags = {mode: [] for mode in MODES}  # rows for flags.csv
        self.vflags = {mode: [] for mode in MODES}  # velocity-aware rule at the default candidates
        self.stale_episodes = []  # (unix, seconds)
        self.carry = {}  # mode -> last sample of the decimated plausibility history
        self.heights = []  # per-minute median heights (tracking)


class Analysis:
    def __init__(self, session: str, params: dict, tick_hz: float):
        self.session = session
        self.params = params
        self.tick = 1.0 / tick_hz
        self.devices_info: dict = {}
        self.device_info_changes = []
        self.notes = []
        self.dev: dict = {}
        self.wfd: dict = defaultdict(lambda: {"t": [], "q": [], "T": []})
        self.events_recs = []
        self.events_unix = []
        self.chunks = []
        self.minutes: dict = {}  # (minute, dev) -> accumulators
        self.mono_to_unix = None

    # ---- world transform ------------------------------------------------------------------
    def add_wfd(self, recs: np.ndarray):
        for d in np.unique(recs["device"]):
            r = recs[recs["device"] == d]
            v = r["v"].astype(np.float64)
            h = self.wfd[int(d)]
            for i in range(len(r)):
                T = v[i, 0:3]
                q = v[i, 3:7]
                nq = np.linalg.norm(q)
                if nq > 0:
                    q = q / nq
                if h["t"]:
                    lastT = h["T"][-1]
                    lastq = h["q"][-1]
                    if np.linalg.norm(T - lastT) < 1e-5 and abs(float(np.dot(q, lastq))) > 0.99999999:
                        continue
                h["t"].append(float(r["t"][i]))
                h["q"].append(q)
                h["T"].append(T)

    def to_world(self, dev: int, t: np.ndarray, p: np.ndarray, q: np.ndarray, v: np.ndarray):
        h = self.wfd.get(dev)
        if not h or not h["t"]:
            return p, q, v
        ht = np.asarray(h["t"])
        idx = np.clip(np.searchsorted(ht, t, side="right") - 1, 0, len(ht) - 1)
        R = np.asarray(h["q"])[idx]
        T = np.asarray(h["T"])[idx]
        return T + qrot(R, p), qmul(R, q), qrot(R, v)

    # ---- per chunk ------------------------------------------------------------------------
    def process_chunk(self, idx: int, path: Path, label: str):
        c = read_chunk(path)
        r = c.records
        info = {"index": idx, "folder": label, "records": int(len(r)), "unix_start": c.header.unix_start, "version": c.header.version}
        if len(r) == 0:
            self.chunks.append(info)
            return
        r = r[np.argsort(r["t"], kind="stable")]
        u = c.header.mono_to_unix(r["t"])
        if self.mono_to_unix is None:
            self.mono_to_unix = c.header.unix_start - c.header.mono_start
        info.update(first_unix=float(u[0]), last_unix=float(u[-1]), types=Counter(r["type"].tolist()))
        self.chunks.append(info)

        for tx in _assemble_texts(r, u):
            if tx["type"] == "DEVICE":
                parsed = _parse_device_info(tx["text"])
                old = self.devices_info.get(tx["device"])
                if old is not None and (old.get("serial") != parsed.get("serial") or old.get("recorded") != parsed.get("recorded")):
                    self.device_info_changes.append((tx["unix"], tx["device"], old, parsed))
                self.devices_info[tx["device"]] = parsed
            else:
                self.notes.append((tx["unix"], tx["kind"], tx["text"]))

        wfd = r[r["type"] == TYPE_WFD]
        if len(wfd):
            self.add_wfd(wfd)

        m = np.isin(r["type"], EVENT_TYPES)
        if m.any():
            self.events_recs.append(r[m].copy())
            self.events_unix.append(u[m].copy())

        pm = r["type"] == TYPE_POSE
        pr = r[pm]
        pu = u[pm]
        for d in np.unique(pr["device"]):
            sel = pr["device"] == d
            self.process_device(int(d), pr[sel], pu[sel])

    def minute_acc(self, minute: int, dev: int) -> dict:
        key = (minute, dev)
        acc = self.minutes.get(key)
        if acc is None:
            acc = dict(n=0, n_trk=0, sum_y=0.0, y_lo=np.inf, y_hi=-np.inf, sum_speed=0.0, sum_hspeed=0.0, sum_vy2=0.0,
                       n_still=0, path=0.0, flags={mode: Counter() for mode in MODES})
            self.minutes[key] = acc
        return acc

    def process_device(self, dev: int, recs: np.ndarray, unix: np.ndarray):
        S = self.dev.get(dev)
        if S is None:
            S = self.dev[dev] = DeviceState(dev)
        P = self.params
        t = recs["t"].astype(np.float64)
        n = len(t)
        S.n += n
        if S.first_unix is None:
            S.first_unix = float(unix[0])
        S.last_unix = float(unix[-1])
        res = pose_results(recs)
        valid = (recs["b"] & POSE_FLAG_VALID) != 0
        conn = (recs["b"] & POSE_FLAG_CONNECTED) != 0
        tracking = valid & conn & (res == RUNNING_OK)
        vals, cnts = np.unique(res, return_counts=True)
        for a, b in zip(vals.tolist(), cnts.tolist()):
            S.results[a] += b
        S.results["invalid"] += int((~valid).sum())
        S.results["disconnected"] += int((~conn).sum())
        S.n_tracking += int(tracking.sum())
        off = float(unix[0] - t[0])

        # pose-stream gaps
        tt = np.concatenate([[S.last_t], t]) if S.last_t is not None else t
        dts = np.diff(tt)
        for thr in (0.05, 0.25, 1.0):
            S.gaps[thr] += int((dts > thr).sum())
        if len(dts):
            i = int(np.argmax(dts))
            if dts[i] > S.max_gap[1]:
                S.max_gap = (float(tt[i] + off), float(dts[i]))
            for i in np.nonzero(dts > 1.0)[0]:
                S.long_gaps.append((float(tt[i] + off), float(dts[i])))
        S.last_t = float(t[-1])

        # tracking losses (no tracking pose for > 0.25 s, including the pose stream stopping)
        ttrk = t[tracking]
        if len(ttrk):
            tt2 = np.concatenate([[S.last_track_t], ttrk]) if S.last_track_t is not None else ttrk
            d2 = np.diff(tt2)
            for i in np.nonzero(d2 > 0.25)[0]:
                S.track_losses.append((float(tt2[i] + off), float(d2[i])))
            S.last_track_t = float(ttrk[-1])

        v13 = recs["v"].astype(np.float64)
        p, q, v = self.to_world(dev, t, v13[:, 0:3], v13[:, 3:7], v13[:, 7:10])
        speed = np.linalg.norm(v, axis=1)

        # per minute
        minute = np.floor(unix / 60.0).astype(np.int64)
        for mnt in np.unique(minute):
            sel = minute == mnt
            acc = self.minute_acc(int(mnt), dev)
            acc["n"] += int(sel.sum())
            st = sel & tracking
            k = int(st.sum())
            acc["n_trk"] += k
            if k:
                y = p[st, 1]
                acc["sum_y"] += float(y.sum())
                acc["y_lo"] = min(acc["y_lo"], float(np.percentile(y, 5)))
                acc["y_hi"] = max(acc["y_hi"], float(np.percentile(y, 95)))
                acc["sum_speed"] += float(speed[st].sum())
                acc["sum_hspeed"] += float(np.hypot(v[st, 0], v[st, 2]).sum())
                acc["sum_vy2"] += float((v[st, 1] ** 2).sum())
                acc["n_still"] += int((speed[st] < 0.02).sum())

        # ---- decimate to the overlay tick and run the plausibility check -------------------
        # "tick": like the overlay, the sample time is the tick time (dt = tick spacing);
        # "pose": the same samples with their own pose times (what the devices really did).
        ki = np.floor(t / self.tick).astype(np.int64)
        last_of_tick = np.r_[ki[1:] != ki[:-1], True]
        keep = last_of_tick & tracking
        if not keep.any():
            return
        wmag = np.degrees(np.linalg.norm(v13[:, 10:13], axis=1))  # reported angular speed, deg/s (frame-free)
        self._plausibility(S, dev, "tick", ki[keep] * self.tick, p[keep], q[keep], v[keep], unix[keep], wmag[keep])
        self._plausibility(S, dev, "pose", t[keep], p[keep], q[keep], v[keep], unix[keep], wmag[keep])

    def _plausibility(self, S, dev, mode, T, Pp, Qq, Vv, U, Wm):
        P = self.params
        carry = S.carry.get(mode)
        if carry is not None:
            T = np.concatenate([[carry["T"]], T])
            Pp = np.vstack([carry["P"], Pp])
            Qq = np.vstack([carry["Q"], Qq])
            Vv = np.vstack([carry["V"], Vv])
            U = np.concatenate([[carry["U"]], U])
            Wm = np.concatenate([[carry["W"]], Wm])
        # identical poses: the history keeps the last distinct sample (stale logic)
        same = np.r_[False, (np.linalg.norm(np.diff(Pp, axis=0), axis=1) == 0.0) & (np.abs(np.sum(Qq[1:] * Qq[:-1], axis=1)) > 0.9999999)]
        if same.any():
            if mode == "tick":
                # stale episodes: a run of identical samples lasting longer than t_stale
                runs = np.flatnonzero(np.diff(np.r_[0, same.astype(np.int8), 0]))
                for a, b in zip(runs[::2], runs[1::2]):
                    dur = T[b - 1] - T[a - 1]
                    if dur > P["t_stale"]:
                        S.stale_episodes.append((float(U[a - 1]), float(dur)))
            T, Pp, Qq, Vv, U, Wm = T[~same], Pp[~same], Qq[~same], Vv[~same], U[~same], Wm[~same]
        if len(T) < 2:
            S.carry[mode] = dict(T=T[-1], P=Pp[-1], Q=Qq[-1], V=Vv[-1], U=U[-1], W=Wm[-1], obsV=None if carry is None else carry.get("obsV"))
            return
        dt = np.diff(T)
        dP = np.diff(Pp, axis=0)
        d = np.linalg.norm(dP, axis=1)
        reset = dt > P["dt_max"]
        ok = (dt > 0) & ~reset
        safe_dt = np.where(dt > 0, dt, 1.0)
        obsV = np.where(ok[:, None], dP / safe_dt[:, None], np.nan)
        v_err = np.linalg.norm(obsV - Vv[1:], axis=1)
        first_prev = carry["obsV"] if (carry is not None and carry.get("obsV") is not None) else np.full(3, np.nan)
        prev_obsV = np.vstack([first_prev[None, :], obsV[:-1]])
        accel = np.linalg.norm(obsV - prev_obsV, axis=1) / safe_dt
        dot = np.minimum(1.0, np.abs(np.sum(Qq[1:] * Qq[:-1], axis=1)))
        rot = 2.0 * np.degrees(np.arccos(dot))
        pos_jump = ok & (d > P["v_max"] * dt + P["j_tol"])
        rot_jump = ok & (rot > P["rot_max_dps"] * dt + P["rot_tol_deg"])
        verr_flag = ok & (v_err > P["v_err_max"])
        accel_flag = ok & np.isfinite(accel) & (accel > P["a_max"])
        Uu = U[1:]

        H = S.hist[mode]
        H["speed_reported"].add(np.linalg.norm(Vv[1:][ok], axis=1), Uu[ok])
        H["speed_observed"].add(d[ok] / dt[ok], Uu[ok])
        H["v_err"].add(v_err[ok], Uu[ok])
        am = ok & np.isfinite(accel)
        H["accel"].add(accel[am], Uu[am])
        H["rot_rate_dps"].add(rot[ok] / dt[ok], Uu[ok])
        H["jump_margin_m"].add((d - P["v_max"] * dt)[ok], Uu[ok])

        # velocity-aware rule: step minus what the reported velocity explains (trapezoid, capped)
        vbar = 0.5 * (Vv[:-1] + Vv[1:])
        vn = np.linalg.norm(vbar, axis=1)
        vbar = vbar * np.where(vn > P["v_body_max"], P["v_body_max"] / np.maximum(vn, 1e-12), 1.0)[:, None]
        has_v = (np.linalg.norm(Vv[:-1], axis=1) > 0) | (np.linalg.norm(Vv[1:], axis=1) > 0)
        unexpl = np.where(has_v, np.linalg.norm(dP - vbar * dt[:, None], axis=1), d)
        wbar = np.minimum(0.5 * (Wm[:-1] + Wm[1:]), P["w_body_max_dps"])
        has_w = (Wm[:-1] > 0) | (Wm[1:] > 0)
        unexpl_rot = np.where(has_w, np.maximum(0.0, rot - wbar * dt), rot)
        H["unexplained_m"].add(unexpl[ok], Uu[ok])
        H["unexplained_rot_deg"].add(unexpl_rot[ok], Uu[ok])
        H["need_v_unexplained"].add(np.maximum((unexpl - P["j_tol"]) / safe_dt, 1e-6)[ok], Uu[ok])
        H["need_rot_unexplained"].add(np.maximum((unexpl_rot - P["rot_tol_deg"]) / safe_dt, 1e-6)[ok], Uu[ok])

        flags = pos_jump * FLAG_POS_JUMP + rot_jump * FLAG_ROT_JUMP + verr_flag * FLAG_VERR + accel_flag * FLAG_ACCEL
        C = S.flag_counts[mode]
        for bit in FLAG_NAMES:
            C[bit] += int(((flags & bit) != 0).sum())
        C["any"] += int((flags != 0).sum())
        C["evaluated"] += int(ok.sum())
        for vu in V_UNEXPLAINED_CANDIDATES:
            C[f"u{vu}"] += int((ok & (unexpl > vu * dt + P["j_tol"])).sum())
        for ru in ROT_UNEXPLAINED_CANDIDATES:
            C[f"r{ru:.0f}"] += int((ok & (unexpl_rot > ru * dt + P["rot_tol_deg"])).sum())
        vpos = ok & (unexpl > P["v_unexplained"] * dt + P["j_tol"])
        vrot = ok & (unexpl_rot > P["rot_unexplained_dps"] * dt + P["rot_tol_deg"])
        vfl = vpos * FLAG_POS_JUMP + vrot * FLAG_ROT_JUMP
        C["v_any"] += int((vfl != 0).sum())
        rows = S.vflags[mode]
        for i in np.nonzero(vfl)[0][: max(0, MAX_FLAG_ROWS - len(rows))]:
            rows.append((float(Uu[i]), dev, int(vfl[i]), float(unexpl[i]), float(dt[i]), float(v_err[i]),
                         -1.0, float(unexpl_rot[i]), float(np.linalg.norm(Vv[i + 1]))))
        fi = np.nonzero(flags)[0]
        if len(fi):
            fmin = np.floor(Uu[fi] / 60.0).astype(np.int64)
            pairs, counts = np.unique(np.stack([fmin, flags[fi].astype(np.int64)], axis=1), axis=0, return_counts=True)
            for (mnt, fl), cnt in zip(pairs.tolist(), counts.tolist()):
                self.minute_acc(int(mnt), dev)["flags"][mode][int(fl)] += int(cnt)
            rows = S.flags[mode]
            for i in fi[: max(0, MAX_FLAG_ROWS - len(rows))]:
                rows.append((float(Uu[i]), dev, int(flags[i]), float(d[i]), float(dt[i]), float(v_err[i]),
                             float(accel[i]) if np.isfinite(accel[i]) else -1.0, float(rot[i]), float(np.linalg.norm(Vv[i + 1]))))
        if mode == "pose":
            mins = np.floor(Uu / 60.0).astype(np.int64)
            for mnt in np.unique(mins):
                sel = (mins == mnt) & ok
                if sel.any():
                    self.minute_acc(int(mnt), dev)["path"] += float(d[sel].sum())
        S.carry[mode] = dict(T=T[-1], P=Pp[-1], Q=Qq[-1], V=Vv[-1], U=U[-1], W=Wm[-1], obsV=obsV[-1] if ok[-1] else None)

    # ---- finish ---------------------------------------------------------------------------
    def event_frames(self):
        if not self.events_recs:
            return {}
        recs = np.concatenate(self.events_recs)
        unix = np.concatenate(self.events_unix)
        order = np.argsort(recs["t"], kind="stable")
        recs, unix = recs[order], unix[order]
        return {name: records_to_frame(recs, unix, rt) for name, rt in [
            ("applied", TYPE_APPLIED), ("trust", TYPE_TRUST), ("markers", TYPE_MARKER), ("session", TYPE_SESSION),
            ("calibrations", TYPE_CALIBRATION), ("device_state", TYPE_DEVICE_STATE), ("lifecycle", TYPE_LIFECYCLE)]}


def device_label(info: dict, dev: int) -> str:
    if not info:
        return f"#{dev}"
    return f"#{dev} {info.get('model', '?')} {info.get('serial', '')} ({info.get('sys', '?')})"


def infer_roles(an: Analysis, frames) -> dict:
    """Head tracker = calibration target; the other Tundra trackers by median height."""
    roles = {}
    cal = frames.get("calibrations")
    target = None
    if cal is not None and len(cal):
        target = int(cal["target"].mode().iloc[0])
        roles[target] = "head tracker (calibration target)"
    heights = {}
    for (mnt, dev), acc in an.minutes.items():
        if acc["n_trk"]:
            heights.setdefault(dev, []).append(acc["sum_y"] / acc["n_trk"])
    trackers = [d for d, i in an.devices_info.items() if i.get("class") == 3 and i.get("sys") == "lighthouse" and d != target and d in heights]
    trackers.sort(key=lambda d: np.median(heights[d]))
    for k, d in enumerate(trackers):
        roles[d] = "foot" if k < len(trackers) - 1 else "hip"
    for d, i in an.devices_info.items():
        if d not in roles and d in heights:
            if i.get("class") == 1:
                roles[d] = "headset"
            elif i.get("class") == 2:
                roles[d] = {"1": "left controller", "2": "right controller"}.get(str(i.get("role")), "controller")
    return roles, {d: float(np.median(h)) for d, h in heights.items()}


def merge_episodes(rows, window=1.0):
    episodes = []
    for f in sorted(rows):
        if episodes and f[0] - episodes[-1]["end"] <= window:
            e = episodes[-1]
            e["end"] = f[0]
            e["n"] += 1
            e["devices"].add(f[1])
            e["kinds"] |= f[2]
            e["max_d"] = max(e["max_d"], f[3])
            e["max_verr"] = max(e["max_verr"], f[5])
            e["max_acc"] = max(e["max_acc"], f[6])
            e["max_rot"] = max(e["max_rot"], f[7])
        else:
            episodes.append(dict(start=f[0], end=f[0], n=1, devices={f[1]}, kinds=f[2], max_d=f[3], max_verr=f[5], max_acc=f[6], max_rot=f[7]))
    return episodes


def flag_text(bits: int) -> str:
    return "+".join(n for b, n in FLAG_NAMES.items() if bits & b)


def write_outputs(an: Analysis, out: Path, elapsed: float):
    out.mkdir(parents=True, exist_ok=True)
    frames = an.event_frames()
    roles, med_heights = infer_roles(an, frames)
    P = an.params
    lines = []
    w = lines.append
    # the recorder's device filter (recorded=1) names the guarded calibration's systems; older
    # recordings without it fall back to the usual pair
    guarded_sys = {i.get("sys") for i in an.devices_info.values() if str(i.get("recorded")) == "1"} or {"oculus", "lighthouse"}
    recorded = {d for d, i in an.devices_info.items() if i.get("sys") in guarded_sys}
    body = sorted(d for d in an.dev if d in recorded and an.devices_info[d].get("class") in (1, 2, 3))

    def name(dev):
        return f"#{dev} {roles.get(dev) or an.devices_info.get(dev, {}).get('model', '')}"

    # ---- overview ---------------------------------------------------------------------------
    idxs = [c["index"] for c in an.chunks]
    missing = sorted(set(range(min(idxs), max(idxs) + 1)) - set(idxs)) if idxs else []
    starts = [c for c in an.chunks if "first_unix" in c]
    t0 = min(c["first_unix"] for c in starts)
    t1 = max(c["last_unix"] for c in starts)
    total_records = sum(c["records"] for c in an.chunks)
    w(f"Session {an.session}: {datetime.fromtimestamp(t0):%Y-%m-%d %H:%M:%S} to {datetime.fromtimestamp(t1):%H:%M:%S} ({(t1 - t0) / 3600:.2f} h)")
    w(f"Chunks: {len(an.chunks)} read (index {min(idxs)}..{max(idxs)}), missing {len(missing)}: {missing}")
    w(f"  copies used from: {dict(Counter(c['folder'] for c in an.chunks))}")
    w(f"Records: {total_records:,} ({total_records / max(1.0, t1 - t0):.0f}/s); analysis took {elapsed:.0f} s")
    prev = None
    overlaps = []
    for c in sorted(starts, key=lambda c: c["index"]):
        if prev is not None and c["first_unix"] < prev["last_unix"] - 1.0:
            overlaps.append((prev["index"], c["index"], round(prev["last_unix"] - c["first_unix"], 1)))
        prev = c
    if overlaps:
        w(f"  chunks overlapping in time (two recorders?): {overlaps[:30]}")
    gaps = []
    prev = None
    for c in sorted(starts, key=lambda c: c["index"]):
        if prev is not None and c["first_unix"] - prev["last_unix"] > 5.0:
            gaps.append((prev["index"], c["index"], clock(prev["last_unix"]), round(c["first_unix"] - prev["last_unix"], 1)))
        prev = c
    if gaps:
        w(f"  time gaps between chunks > 5 s (from, to, at, seconds): {gaps[:30]}")
    w(f"Plausibility thresholds (guard.json): v_max {P['v_max']} m/s, j_tol {P['j_tol']} m, v_err_max {P['v_err_max']} m/s, a_max {P['a_max']} m/s^2, "
      f"rot {P['rot_max_dps']} deg/s + {P['rot_tol_deg']} deg; offline tick {1 / an.tick:.0f} Hz")
    w("  mode 'pose' = samples at their own pose times (what the devices did); 'tick' = sample time is the tick time, as the overlay does it")
    w("")

    lc = frames.get("lifecycle")
    if lc is not None and len(lc):
        w("Lifecycle records:")
        for _, row in lc.iterrows():
            w(f"  {clock(row['unix'])} {row['kind']:<20} code={int(row['code'])} f0={row['f0']:.3g} v=[{row['v0']:.3g}, {row['v1']:.3g}, {row['v2']:.3g}]")
        w("")
    ss = frames.get("session")
    if ss is not None and len(ss):
        w(f"SESSION records: {len(ss)}, dropped max {ss['dropped'].max():.0f}, ring high water max {ss['ring_high_water'].max():.0f}")
        w("")

    # ---- devices -------------------------------------------------------------------------------
    w("Devices (recorded systems):")
    w(f"  {'device':<52} {'role':<34} {'rate/s':>7} {'track%':>7} {'losses>0.25s':>13} {'lost s':>8} {'max gap s':>10} {'stale':>6} {'med y':>6}")
    for dev in sorted(d for d in an.dev if d in recorded):
        S = an.dev[dev]
        info = an.devices_info.get(dev, {})
        dur = max(1e-6, (S.last_unix or 0) - (S.first_unix or 0))
        lost = sum(s for _, s in S.track_losses)
        w(f"  {device_label(info, dev):<52.52} {roles.get(dev, ''):<34.34} {S.n / dur:7.1f} {100 * S.n_tracking / max(1, S.n):7.2f} {len(S.track_losses):13d} {lost:8.1f} {S.max_gap[1]:10.2f} {len(S.stale_episodes):6d} {med_heights.get(dev, float('nan')):6.2f}")
    others = sorted(d for d in an.devices_info if d not in recorded)
    w(f"  not recorded (other tracking systems, filter since 00:58): {len(others)} devices")
    w("")

    w("Tracking losses longer than 1 s (headset and lighthouse body devices):")
    rows = []
    for dev in body:
        info = an.devices_info.get(dev, {})
        if info.get("class") != 1 and info.get("sys") != "lighthouse":
            continue
        rows += [(u, dev, s) for u, s in an.dev[dev].track_losses if s > 1.0]
    rows.sort()
    for u, dev, s in rows[:150]:
        w(f"  {clock(u)} {name(dev):<36.36} {s:8.1f} s")
    if len(rows) > 150:
        w(f"  ... {len(rows) - 150} more")
    for dev in body:
        info = an.devices_info.get(dev, {})
        if info.get("class") == 2 and info.get("sys") != "lighthouse":
            ls = an.dev[dev].track_losses
            w(f"  {name(dev)} ({info.get('model')}): {len(ls)} losses > 0.25 s, {sum(s for _, s in ls):.0f} s total (not in the list)")
    w("")

    # ---- motion baselines --------------------------------------------------------------------
    w("Motion baselines (decimated to the tick, tracking samples only):")
    w(f"  {'device':<24} {'metric':<15} | pose: {'p50':>8} {'p99':>8} {'p99.9':>8} {'p99.99':>8} {'max':>8} {'at':>8} | tick: {'p99.9':>8} {'p99.99':>8} {'max':>8}")
    for dev in body:
        S = an.dev[dev]
        for m in METRICS:
            hp = S.hist["pose"][m]
            ht = S.hist["tick"][m]
            if hp.n == 0:
                continue
            w(f"  {name(dev):<24.24} {m:<15} |       {hp.pct(50):8.3f} {hp.pct(99):8.3f} {hp.pct(99.9):8.3f} {hp.pct(99.99):8.3f} {hp.max:8.3f} {clock(hp.max_unix):>8} |       {ht.pct(99.9):8.3f} {ht.pct(99.99):8.3f} {ht.max:8.3f}")
    w("  (jump_margin_m = step - v_max * dt; a position jump is flagged above j_tol)")
    w("")
    for mode in MODES:
        w(f"Offline plausibility flags, mode '{mode}':")
        w(f"  {'device':<24} {'evaluated':>10} {'any':>8} {'pos_jump':>9} {'rot_jump':>9} {'v_err':>8} {'accel':>8} {'any/hour':>9}")
        for dev in body:
            C = an.dev[dev].flag_counts[mode]
            hours = max(1e-6, C["evaluated"] * an.tick / 3600.0)
            w(f"  {name(dev):<24.24} {C['evaluated']:10d} {C['any']:8d} {C[FLAG_POS_JUMP]:9d} {C[FLAG_ROT_JUMP]:9d} {C[FLAG_VERR]:8d} {C[FLAG_ACCEL]:8d} {C['any'] / hours:9.1f}")
        w("")

    w(f"Velocity-aware rule: step not explained by the device's reported velocity (capped {P['v_body_max']} m/s, {P['w_body_max_dps']:.0f} deg/s)")
    w("  flagged samples for candidate allowances (position: unexplained > v * dt + j_tol; rotation: > r * dt + rot_tol)")
    for mode in MODES:
        w(f"  mode '{mode}':")
        head = " ".join(f"{'u' + str(v):>8}" for v in V_UNEXPLAINED_CANDIDATES) + " " + " ".join(f"{'r' + format(r, '.0f'):>8}" for r in ROT_UNEXPLAINED_CANDIDATES)
        w(f"    {'device':<24} {'evaluated':>10} {head}")
        for dev in body:
            C = an.dev[dev].flag_counts[mode]
            vals = " ".join(f"{C[f'u{v}']:8d}" for v in V_UNEXPLAINED_CANDIDATES) + " " + " ".join(f"{C[f'r{r:.0f}']:8d}" for r in ROT_UNEXPLAINED_CANDIDATES)
            w(f"    {name(dev):<24.24} {C['evaluated']:10d} {vals}")
    for mode in MODES:
        allrows = [f for dev in body for f in an.dev[dev].vflags[mode]]
        eps = merge_episodes(allrows)
        w(f"  episodes at the configured v={P['v_unexplained']} m/s, r={P['rot_unexplained_dps']:.0f} deg/s, mode '{mode}': {len(eps)}")
        for e in sorted(eps, key=lambda e: -e["max_d"])[:40]:
            devs = ", ".join(name(d) for d in sorted(e["devices"]))
            w(f"    {clock(e['start'])} {e['end'] - e['start']:5.1f}s n={e['n']:<5d} {flag_text(e['kinds']):<18} unexplained {e['max_d'] * 100:6.1f} cm, rot {e['max_rot']:5.1f} deg  [{devs}]")
    w("")
    for mode in MODES:
        allrows = [f for dev in body for f in an.dev[dev].flags[mode]]
        eps = merge_episodes(allrows)
        jumps = [e for e in eps if e["kinds"] & (FLAG_POS_JUMP | FLAG_ROT_JUMP)]
        w(f"Flag episodes, mode '{mode}' (flags within 1 s merged): {len(eps)}, with a position/rotation jump: {len(jumps)}")
        for e in sorted(jumps, key=lambda e: -e["max_d"])[:30]:
            devs = ", ".join(name(d) for d in sorted(e["devices"]))
            w(f"  {clock(e['start'])} {e['end'] - e['start']:5.1f}s n={e['n']:<5d} {flag_text(e['kinds']):<28} step {e['max_d'] * 100:6.1f} cm v_err {e['max_verr']:5.2f} acc {e['max_acc']:6.0f} rot {e['max_rot']:5.1f} deg  [{devs}]")
        w("")

    # ---- trust transitions -----------------------------------------------------------------
    for key, df in frames.items():
        if df is not None and len(df):
            df.assign(time=[clock(u) for u in df["unix"]]).to_csv(out / f"{key}.csv", index=False)
    tr = frames.get("trust")
    w(f"Live TRUST transitions: {0 if tr is None else len(tr)}")
    if tr is not None and len(tr):
        w("  by device and transition:")
        summ = tr.assign(tr_=tr["old_state"] + "->" + tr["new_state"]).groupby(["device", "tr_", "reason_name"]).size()
        for (dev, trn, reason), n in summ.items():
            w(f"    {name(int(dev)):<26.26} {trn:<22} {reason:<24} {n:5d}")
        bad = tr[tr["new_state"] == "UNTRUSTED"]
        w(f"  transitions to UNTRUSTED: {len(bad)}; grouped (within 2 s):")
        groups = []
        for _, row in bad.iterrows():
            if groups and row["unix"] - groups[-1]["end"] <= 2.0:
                groups[-1]["end"] = row["unix"]
                groups[-1]["devs"].add(int(row["device"]))
                groups[-1]["reasons"].add(row["reason_name"])
            else:
                groups.append({"start": row["unix"], "end": row["unix"], "devs": {int(row["device"])}, "reasons": {row["reason_name"]}})
        for g in groups:
            w(f"    {clock(g['start'])} {', '.join(name(d) for d in sorted(g['devs']))}  ({', '.join(sorted(g['reasons']))})")
        per_hour = Counter(int(u // 3600) for u in tr["unix"])
        w("  transitions per hour: " + ", ".join(f"{datetime.fromtimestamp(h * 3600):%H}h {n}" for h, n in sorted(per_hour.items())))
        w("  full list: trust.csv")
    w("")
    mk = frames.get("markers")
    w(f"Markers: {0 if mk is None else len(mk)}")
    if mk is not None and len(mk):
        # the user's notes (TEXT marker_note: "<marker t> <note>", latest per marker wins)
        marker_notes = {}
        for _, kind, text in an.notes:
            if kind == "marker_note":
                t_text, _, note = text.partition(" ")
                try:
                    marker_notes[round(float(t_text), 3)] = note
                except ValueError:
                    pass
        for _, row in mk.iterrows():
            note = marker_notes.get(round(float(row["t"]), 3), "")
            w(f"  {clock(row['unix'])} {row['source']} label={int(row['label'])}" + (f'  note: "{note}"' if note else ""))
    w("")

    # ---- calibrations --------------------------------------------------------------------
    cal = frames.get("calibrations")
    if cal is not None and len(cal):
        w(f"Calibration attempts: {len(cal)}")
        for (trig, outc), n in cal.groupby(["trigger", "outcome"]).size().items():
            w(f"  {trig:<22} {outc:<10} {n:6d}")
        errs = cal[cal["outcome"] == "rejected"].groupby("error_name").size().sort_values(ascending=False)
        if len(errs):
            w("  rejected because: " + ", ".join(f"{k} {v}" for k, v in errs.items()))
        ch = cal[cal["outcome"].isin(["applied", "forced", "corrected"])]
        if len(ch):
            w(f"  applied/forced/corrected: {len(ch)}; change vs active: median {ch['delta_trans_m'].median() * 1000:.1f} mm / {ch['delta_rot_deg'].median():.2f} deg, "
              f"p95 {ch['delta_trans_m'].quantile(0.95) * 1000:.1f} mm / {ch['delta_rot_deg'].quantile(0.95):.2f} deg, max {ch['delta_trans_m'].max() * 1000:.1f} mm / {ch['delta_rot_deg'].max():.2f} deg")
        rms = cal["rms_m"][cal["rms_m"] > 0]
        if len(rms):
            w(f"  solve RMS: median {rms.median() * 1000:.1f} mm, p95 {rms.quantile(0.95) * 1000:.1f} mm, max {rms.max() * 1000:.1f} mm")
        big = ch[(ch["delta_trans_m"] > 0.10) | (ch["delta_rot_deg"] > 2.0) | (ch["trigger"] != "continuous")]
        w(f"  changes > 10 cm or > 2 deg, and every non-continuous one ({len(big)}; all of them in calibrations.csv):")
        for _, row in big.iterrows():
            w(f"    {clock(row['unix'])} {row['trigger']:<22} {row['outcome']:<10} {row['delta_trans_m'] * 1000:7.1f} mm {row['delta_rot_deg']:6.2f} deg  rms {row['rms_m'] * 1000:6.1f} mm (prev {row['prev_rms_m'] * 1000:6.1f}) samples {int(row['samples'])} {row['error_name']}")
        cm = cal.assign(minute=(cal["unix"] // 60).astype(int))
        per_min = cm.groupby("minute").size()
        w(f"  attempts per minute: median {per_min.median():.0f}, max {per_min.max():.0f} at {clock_min(int(per_min.idxmax()))}")
        applied_min = cm[cm["outcome"].isin(["applied", "forced", "corrected"])].groupby("minute").size()
        if len(applied_min):
            w(f"  applied per minute: median {applied_min.median():.0f}, max {applied_min.max():.0f} at {clock_min(int(applied_min.idxmax()))}")
        w("")

    ap = frames.get("applied")
    if ap is not None and "carried" in ap:
        ap = ap[~ap["carried"].astype(bool)]  # repeats at chunk starts are not changes
    if ap is not None and len(ap):
        w(f"APPLIED records (transforms sent to the driver): {len(ap)}")
        steps = []
        for dev, g in ap.groupby("device"):
            if len(g) < 2:
                continue
            tvec = g[["tx", "ty", "tz"]].to_numpy(dtype=np.float64)
            qv = g[["qw", "qx", "qy", "qz"]].to_numpy(dtype=np.float64)
            dtr = np.linalg.norm(np.diff(tvec, axis=0), axis=1)
            drot = 2 * np.degrees(np.arccos(np.minimum(1.0, np.abs(np.sum(qv[1:] * qv[:-1], axis=1)))))
            for i in np.argsort(-dtr)[:5]:
                steps.append((float(g["unix"].iloc[i + 1]), int(dev), float(dtr[i]), float(drot[i])))
            w(f"  {name(int(dev)):<24.24} n={len(g):7d} step median {np.median(dtr) * 1000:.2f} mm, p99 {np.percentile(dtr, 99) * 1000:.1f} mm, max {dtr.max() * 1000:.1f} mm / {drot.max():.2f} deg")
        w("  largest steps:")
        for u, dev, dtr, drot in sorted(steps, key=lambda s: -s[2])[:12]:
            w(f"    {clock(u)} {name(dev):<24.24} {dtr * 1000:7.1f} mm {drot:6.2f} deg")
        w("")

    ds = frames.get("device_state")
    if ds is not None and len(ds):
        ds = ds[ds["device"].isin(list(recorded))]
        w(f"Device connect/disconnect records (recorded systems): {len(ds)}")
        for _, row in ds.iterrows():
            w(f"  {clock(row['unix'])} {name(int(row['device'])):<30.30} {'connected' if row['connected'] else 'DISCONNECTED'} ({row['result_name']})")
        w("")

    if an.notes:
        w("Notes:")
        seen = set()
        for u, kind, text in an.notes:
            if (kind, text) in seen:
                continue
            seen.add((kind, text))
            w(f"  {clock(u)} [{kind}] {text[:300]}")
        w("")

    # ---- per-minute activity ----------------------------------------------------------------
    hmd = next((d for d in body if an.devices_info[d].get("class") == 1), 0)
    hip = next((d for d, r in roles.items() if r == "hip"), None)
    minutes = sorted({m for m, _ in an.minutes})
    lh_body = [d for d in body if an.devices_info[d].get("sys") == "lighthouse"]
    cal_min = Counter((cal["unix"] // 60).astype(int)) if cal is not None and len(cal) else Counter()
    tr_min = Counter((tr["unix"] // 60).astype(int)) if tr is not None and len(tr) else Counter()
    mk_min = Counter((mk["unix"] // 60).astype(int)) if mk is not None and len(mk) else Counter()
    w("Activity per minute (hmd = headset, hip = hip tracker; lhTrk = lighthouse body devices tracking > 50%; flags on body devices):")
    w("  time   hmd%  still%  hmd_y  walk m/s  path m  hip_y  hip_vy_rms  lhTrk  flg_pose  flg_tick  calib  trust  marks")
    with open(out / "minutes.csv", "w", newline="", encoding="utf-8") as fcsv:
        cw = csv.writer(fcsv)
        cw.writerow(["minute", "device", "role", "n", "tracking_frac", "mean_y", "y_p5", "y_p95", "mean_speed", "mean_hspeed", "vy_rms", "still_frac", "path_m"]
                    + [f"{mode}_{n}" for mode in MODES for n in FLAG_NAMES.values()])
        for mnt in minutes:
            for dev in sorted(d for (m, d) in an.minutes if m == mnt and d in recorded):
                a = an.minutes[(mnt, dev)]
                k = max(1, a["n_trk"])
                has = a["n_trk"] > 0
                cw.writerow([clock_min(mnt), dev, roles.get(dev, ""), a["n"], round(a["n_trk"] / max(1, a["n"]), 4),
                             round(a["sum_y"] / k, 3) if has else "", round(a["y_lo"], 3) if has else "", round(a["y_hi"], 3) if has else "",
                             round(a["sum_speed"] / k, 3), round(a["sum_hspeed"] / k, 3), round((a["sum_vy2"] / k) ** 0.5, 3), round(a["n_still"] / k, 3), round(a["path"], 2)]
                            + [sum(v for f, v in a["flags"][mode].items() if f & bit) for mode in MODES for bit in FLAG_NAMES])
            h = an.minutes.get((mnt, hmd))
            hp = an.minutes.get((mnt, hip)) if hip is not None else None
            hmd_pct = 100.0 * h["n_trk"] / max(1, h["n"]) if h else 0.0
            still = 100.0 * h["n_still"] / h["n_trk"] if h and h["n_trk"] else float("nan")
            hy = h["sum_y"] / h["n_trk"] if h and h["n_trk"] else float("nan")
            walk = h["sum_hspeed"] / h["n_trk"] if h and h["n_trk"] else float("nan")
            path = h["path"] if h else 0.0
            hipy = hp["sum_y"] / hp["n_trk"] if hp and hp["n_trk"] else float("nan")
            hipvy = (hp["sum_vy2"] / hp["n_trk"]) ** 0.5 if hp and hp["n_trk"] else float("nan")
            lh_trk = sum(1 for d in lh_body if (mnt, d) in an.minutes and an.minutes[(mnt, d)]["n_trk"] > 0.5 * an.minutes[(mnt, d)]["n"])
            fp = sum(sum(an.minutes[(mnt, d)]["flags"]["pose"].values()) for d in body if (mnt, d) in an.minutes)
            ft = sum(sum(an.minutes[(mnt, d)]["flags"]["tick"].values()) for d in body if (mnt, d) in an.minutes)
            w(f"  {clock_min(mnt)} {hmd_pct:5.0f} {still:6.0f} {hy:6.2f} {walk:9.2f} {path:7.1f} {hipy:6.2f} {hipvy:11.2f} {lh_trk:6d} {fp:9d} {ft:9d} {cal_min.get(mnt, 0):6d} {tr_min.get(mnt, 0):6d} {mk_min.get(mnt, 0):6d}")
    w("")

    with open(out / "flags.csv", "w", newline="", encoding="utf-8") as fcsv:
        cw = csv.writer(fcsv)
        cw.writerow(["mode", "time", "unix", "device", "role", "flags", "step_m", "dt_s", "v_err", "accel", "rot_deg", "speed_reported"])
        for mode in MODES:
            for f in sorted(f for dev in body for f in an.dev[dev].flags[mode]):
                cw.writerow([mode, clock(f[0]), round(f[0], 3), f[1], roles.get(f[1], ""), flag_text(f[2])] + [round(x, 4) for x in f[3:]])

    with open(out / "vflags.csv", "w", newline="", encoding="utf-8") as fcsv:
        cw = csv.writer(fcsv)
        cw.writerow(["mode", "time", "unix", "device", "role", "flags", "unexplained_m", "dt_s", "v_err", "unexplained_rot_deg", "speed_reported"])
        for mode in MODES:
            for f in sorted(f for dev in body for f in an.dev[dev].vflags[mode]):
                cw.writerow([mode, clock(f[0]), round(f[0], 3), f[1], roles.get(f[1], ""), flag_text(f[2]), round(f[3], 4), round(f[4], 4),
                             round(f[5], 4), round(f[7], 3), round(f[8], 4)])

    with open(out / "losses.csv", "w", newline="", encoding="utf-8") as fcsv:
        cw = csv.writer(fcsv)
        cw.writerow(["time", "unix", "device", "role", "seconds"])
        for u, dev, sec in sorted((u, dev, sec) for dev in body for u, sec in an.dev[dev].track_losses):
            cw.writerow([clock(u), round(u, 3), dev, roles.get(dev, ""), round(sec, 3)])

    # histograms behind the percentiles: bin i holds values in [lo, hi); non-empty bins only
    with open(out / "hist.csv", "w", newline="", encoding="utf-8") as fcsv:
        cw = csv.writer(fcsv)
        cw.writerow(["mode", "device", "role", "metric", "lo", "hi", "count"])
        edges = np.r_[0.0, HIST_EDGES, np.inf]
        for mode in MODES:
            for dev in body:
                for m in ("unexplained_m", "unexplained_rot_deg", "need_v_unexplained", "need_rot_unexplained", "speed_reported"):
                    counts = an.dev[dev].hist[mode][m].counts
                    for i in np.nonzero(counts)[0]:
                        cw.writerow([mode, dev, roles.get(dev, ""), m, f"{edges[i]:.6g}", f"{edges[i + 1]:.6g}", int(counts[i])])

    summary = {
        "session": an.session, "start": t0, "end": t1,
        "chunks": len(an.chunks), "missing_chunks": missing, "params": P,
        "roles": {str(k): v for k, v in roles.items()},
        "devices": {str(d): {"info": an.devices_info.get(d, {}), "n": an.dev[d].n, "tracking": an.dev[d].n_tracking,
                             "results": {str(k): v for k, v in an.dev[d].results.items()}, "track_losses": len(an.dev[d].track_losses),
                             "flags": {mode: {FLAG_NAMES.get(k, str(k)): v for k, v in an.dev[d].flag_counts[mode].items()} for mode in MODES},
                             "pct": {mode: {m: {str(p): an.dev[d].hist[mode][m].pct(p) for p in (50, 99, 99.9, 99.99)} | {"max": an.dev[d].hist[mode][m].max} for m in METRICS} for mode in MODES}}
                    for d in body},
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=1, default=str), encoding="utf-8")
    (out / "report.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", help="session id, e.g. 1a2b3c4d")
    ap.add_argument("--root", default=str(Path(os.environ.get("APPDATA", "")) / "space-calibrator" / "blackbox"))
    ap.add_argument("--out", default=None)
    ap.add_argument("--tick-hz", type=float, default=90.0)
    ap.add_argument("--limit", type=int, default=0, help="only the first N chunks (testing)")
    args = ap.parse_args(argv)
    out = Path(args.out or (REPO / "logs" / f"analysis_{args.session}"))
    out.mkdir(parents=True, exist_ok=True)
    files = collect_chunk_files(Path(args.root), args.session)
    if args.limit:
        files = dict(list(files.items())[: args.limit])
    an = Analysis(args.session, load_params(), args.tick_hz)
    started = time.time()
    progress = out / "progress.txt"
    for k, (idx, (path, size, label)) in enumerate(files.items()):
        try:
            an.process_chunk(idx, path, label)
        except Exception as exc:  # keep going; a damaged chunk should not stop the analysis
            an.notes.append((time.time(), "analysis_error", f"chunk {idx} ({path}): {exc!r}"))
        if k % 5 == 0:
            progress.write_text(f"{k + 1}/{len(files)} chunks, {time.time() - started:.0f} s\n", encoding="utf-8")
    write_outputs(an, out, time.time() - started)
    progress.write_text(f"done: {len(files)} chunks, {time.time() - started:.0f} s\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
