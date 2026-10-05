"""Glitch collection: keep the minutes around every glitch of a finished session, drop the rest.

The recorder keeps every chunk of a session (keep-all archive, ``blackbox/sessions``) plus event
folders around markers. Most of a night is normal tracking. This tool finds the moments worth
keeping in a finished session and writes each one as a small, self-contained folder under
``blackbox/glitches/<session>/``, then deletes the session's archive and event folders.

Moments (from the recording's own records):

* major (kept from 120 s before to 60 s after): a marker you set (hotkey, overlay button, trigger
  hold), a SteamVR universe shift (the lighthouse devices' shared WorldFromDriver changed), a
  calibration correction or a calibration step of 10 cm or more (a head mount fix: of 10 cm or
  more), the head tracker leaving TRUSTED, three or more devices leaving TRUSTED or losing
  tracking within 1.5 s
* minor (15 s before and after): any other device leaving TRUSTED, a smaller head mount fix

A window folder holds the trimmed chunks (poses at full rate within 5 s of a moment, at most
120 per second and device elsewhere; every other record kept), each device's transforms and
device info carried to the window start so it reads on its own, the session's ``overlay.json``
(for the replay tool), your marker notes, ``summary.txt`` (what happened, in words) and
``log.txt`` (the overlay log of that window without the per-frame noise).

Old overlay logs are rewritten without the per-frame noise as well (``--logs``).

The session being recorded right now is never touched. Without ``--apply`` nothing is written
or deleted; the plan is printed.

Usage::

    python -m tools.glitch_collection               # plan for every finished session
    python -m tools.glitch_collection --apply       # write the collection, delete the rest
    python -m tools.glitch_collection --apply --logs
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import sys
import time
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from tools.blackbox import (  # noqa: E402
    CHUNK_EXTENSION,
    HEADER_SIZE,
    HEADER_STRUCT,
    MARKER_SOURCES,
    POSE_FLAG_CONNECTED,
    POSE_FLAG_VALID,
    RECORD_CARRIED,
    TRUST_REASONS,
    TYPE_APPLIED,
    TYPE_CALIBRATION,
    TYPE_DEVICE,
    TYPE_MARKER,
    TYPE_POSE,
    TYPE_TRUST,
    TYPE_WFD,
    _assemble_texts,
    _parse_device_info,
    pose_results,
    read_chunk,
    records_to_frame,
)

MAJOR = (120.0, 60.0)
MINOR = (15.0, 15.0)
FULL_RATE_AROUND = 5.0  # s around a moment kept at full pose rate
POSE_HZ = 120.0  # elsewhere at most this many poses per second and device
USER_SOURCES = {"hotkey", "overlay", "triggers"}
STATES = {0: "TRUSTED", 1: "SUSPECT", 2: "UNTRUSTED", 3: "RECOVERING"}
# overlay log lines that carry no information (per-frame or per-solve noise)
LOG_NOISE = re.compile(
    r"was not defined in a localisation file|applying device \[|Unknown evt with id|Configuration file saved to|"
    r"does not exist on disk|didn't update, skipping update|Loaded locale file"
)


def clock(unix: float) -> str:
    return datetime.fromtimestamp(unix).strftime("%H:%M:%S")


def stamp(unix: float) -> str:
    return datetime.fromtimestamp(unix).strftime("%Y-%m-%d_%H-%M-%S")


def dir_size(p: Path) -> int:
    return sum(f.stat().st_size for f in p.rglob("*") if f.is_file()) if p.exists() else 0


# ---- what is on disk ------------------------------------------------------------------------

class Session:
    def __init__(self, sid: str):
        self.sid = sid
        self.chunks: dict = {}  # index -> path (largest copy)
        self.archive: Path | None = None  # sessions/<start>_<sid>
        self.events: list = []  # event folders holding only this session's chunks
        self.mixed_events: list = []  # event folders that also hold other sessions' chunks
        self.live: list = []
        self.newest_mtime = 0.0

    @property
    def name(self) -> str:
        if self.archive is not None:
            return self.archive.name
        first = min(self.chunks) if self.chunks else None
        if first is None:
            return self.sid
        with open(self.chunks[first], "rb") as fh:
            h = HEADER_STRUCT.unpack(fh.read(HEADER_SIZE))
        return f"{stamp(h[7])}_{self.sid}"


def scan_root(root: Path) -> dict:
    sessions: dict = {}
    sizes: dict = {}
    for f in root.rglob(f"*{CHUNK_EXTENSION}"):
        if "glitches" in f.parts:
            continue
        try:
            sid, idx = f.stem.split("_")[0], int(f.stem.split("_")[1])
            st = f.stat()
        except (ValueError, IndexError, OSError):
            continue
        s = sessions.setdefault(sid, Session(sid))
        s.newest_mtime = max(s.newest_mtime, st.st_mtime)
        if idx not in s.chunks or st.st_size > sizes[(sid, idx)]:
            s.chunks[idx] = f
            sizes[(sid, idx)] = st.st_size
        top = f.relative_to(root).parts[0]
        if top == "sessions":
            s.archive = root / "sessions" / f.parent.name
        elif top == "live":
            s.live.append(f)
    events = root / "events"
    if events.is_dir():
        for d in sorted(events.iterdir()):
            if not d.is_dir():
                continue
            sids = {f.stem.split("_")[0] for f in d.glob(f"*{CHUNK_EXTENSION}")}
            for sid in sids:
                if sid in sessions:
                    (sessions[sid].events if len(sids) == 1 else sessions[sid].mixed_events).append(d)
    return sessions


def current_session(root: Path, sessions: dict) -> set:
    """The session being recorded: chunks in live/, or any chunk written in the last 15 minutes."""
    busy = {s.sid for s in sessions.values() if s.live or time.time() - s.newest_mtime < 900}
    return busy


# ---- finding the moments -----------------------------------------------------------------------

class Moment:
    def __init__(self, unix: float, kind: str, major: bool, text: str, device: int = -1):
        self.unix, self.kind, self.major, self.text, self.device = unix, kind, major, text, device


def same_transform(a: np.ndarray, b: np.ndarray) -> bool:
    return np.linalg.norm(a[0:3] - b[0:3]) < 0.001 and abs(float(np.dot(a[3:7], b[3:7]))) > np.cos(np.radians(0.02) / 2)


def qrot(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    w, x = q[0], q[1:]
    t = 2.0 * np.cross(x, v)
    return v + w * t + np.cross(x, t)


def find_moments(s: Session, overlay_target: int | None):
    """One pass over the session's chunks: trust, markers, calibrations, universe shifts, tracking losses."""
    moments = []
    devices: dict = {}
    trust_rows, losses = [], []
    last_wfd: dict = {}  # device -> transform (7)
    wfd_changes = []  # (unix, device, new transform, move at the device)
    tracking_state: dict = {}  # device -> (tracking, since unix)
    target = overlay_target
    for idx in sorted(s.chunks):
        try:
            c = read_chunk(s.chunks[idx])
        except (OSError, ValueError):
            continue
        r = c.records
        if not len(r):
            continue
        u = c.header.mono_to_unix(r["t"])
        for tx in _assemble_texts(r, u):
            if tx["type"] == "DEVICE":
                devices[tx["device"]] = _parse_device_info(tx["text"])
        body = {d for d, i in devices.items() if str(i.get("class")) in ("1", "2", "3")}
        hmd_sys = {i.get("sys") for i in devices.values() if str(i.get("class")) == "1"}
        lighthouse_like = {d for d in body if devices[d].get("sys") not in hmd_sys}

        cal = records_to_frame(r, u, TYPE_CALIBRATION)
        for row in cal.itertuples():
            if target is None:
                target = int(row.target)
            if row.outcome == "corrected" and row.trigger == "head_mount_fix":
                # the trust layer slid the calibration back to the head tracker's place on the headset: a
                # few a night, mostly a few cm; only large ones get a major window
                moments.append(Moment(row.unix, "head_mount_fix", row.delta_trans_m >= 0.10, f"calibration slid back to the head tracker's place on the headset: {row.delta_trans_m * 100:.1f} cm"))
            elif row.outcome == "corrected":
                moments.append(Moment(row.unix, "correction", True, f"calibration corrected ({row.trigger}): moved {row.delta_trans_m * 100:.1f} cm / {row.delta_rot_deg:.2f} deg"))
            elif row.outcome in ("applied", "forced") and row.delta_trans_m >= 0.10 and row.trigger != "startup":
                moments.append(Moment(row.unix, "calibration_step", True, f"calibration {row.outcome} ({row.trigger}): moved {row.delta_trans_m * 100:.1f} cm / {row.delta_rot_deg:.2f} deg, RMS {row.rms_m * 1000:.1f} mm"))
        mk = records_to_frame(r, u, TYPE_MARKER)
        for row in mk.itertuples():
            if row.source in USER_SOURCES:
                moments.append(Moment(row.unix, "user_marker", True, f"marker set by you ({row.source})"))
        tr = records_to_frame(r, u, TYPE_TRUST)
        for row in tr.itertuples():
            trust_rows.append((row.unix, int(row.device), row.old_state, row.new_state, row.reason_name, float(row.residual_m), float(row.x0)))

        # SteamVR universe shifts: a body device's WorldFromDriver changes
        wm = (r["type"] == TYPE_WFD) & (r["b"] != RECORD_CARRIED)
        for i in np.flatnonzero(wm):
            d = int(r["device"][i])
            if d not in lighthouse_like:
                continue
            v = r["v"][i].astype(np.float64)[0:7]
            v[3:7] /= max(np.linalg.norm(v[3:7]), 1e-12)
            if d in last_wfd and not same_transform(last_wfd[d], v):
                wfd_changes.append((float(u[i]), d, v.copy(), last_wfd[d].copy()))
            last_wfd[d] = v

        # tracking losses of body devices (0.5 s or more)
        pm = r["type"] == TYPE_POSE
        pr, pu = r[pm], u[pm]
        trk_all = ((pr["b"] & POSE_FLAG_VALID) != 0) & ((pr["b"] & POSE_FLAG_CONNECTED) != 0) & (pose_results(pr) == 200)
        for d in np.unique(pr["device"]):
            d = int(d)
            if d not in body:
                continue
            sel = pr["device"] == d
            tu, trk = pu[sel], trk_all[sel]
            state = tracking_state.get(d, (True, tu[0]))
            change = np.flatnonzero(np.diff(np.r_[state[0], trk].astype(np.int8)) != 0)
            for k in change:
                now = bool(trk[k])
                if now and not state[0] and tu[k] - state[1] >= 0.5:
                    losses.append((state[1], d, tu[k] - state[1]))
                state = (now, tu[k])
            tracking_state[d] = state

    labels = {d: device_label(d, i, d == target) for d, i in devices.items()}

    # universe shifts: two or more devices to the same new transform within 0.3 s, a re-solve's size
    used = set()
    for a in range(len(wfd_changes)):
        if a in used:
            continue
        ua, da, va, oa = wfd_changes[a]
        group = [a]
        for b in range(a + 1, len(wfd_changes)):
            ub, db, vb, ob = wfd_changes[b]
            if ub - ua > 0.3:
                break
            if b not in used and db != da and same_transform(va, vb):
                group.append(b)
        if len(group) < 2:
            continue
        # how far it moved the devices: old frame vs new frame at a point 2 m in front of the old origin
        p = oa[0:3] + qrot(oa[3:7], np.array([0.0, 0.0, -2.0]))
        dpos = va[0:3] + qrot(va[3:7], np.array([0.0, 0.0, -2.0])) - p
        move = float(np.linalg.norm(dpos))
        if 0.002 <= move <= 0.5:
            used.update(group)
            moments.append(Moment(ua, "universe_shift", True, f"SteamVR moved the lighthouse universe ~{move * 100:.1f} cm ({len(group)} devices switched transform)"))

    # trust: leaving TRUSTED
    leaving = [t for t in trust_rows if t[2] == "TRUSTED" and t[3] in ("SUSPECT", "UNTRUSTED")]
    for t in leaving:
        unix, d, old, new, reason, resid, x0 = t
        if d == target:
            moments.append(Moment(unix, "head_tracker", True, f"head tracker {old.lower()} > {new.lower()} ({reason}), residual {resid * 100:.1f} cm", d))
        else:
            moments.append(Moment(unix, "glitch", False, f"{labels.get(d, d)} {old.lower()} > {new.lower()} ({reason}), step {x0 * 100:.1f} cm", d))
    # three or more devices at once: shared cause
    events = sorted([(t[0], t[1], "trust") for t in leaving] + [(l[0], l[1], "lost") for l in losses])
    i = 0
    while i < len(events):
        j = i
        while j + 1 < len(events) and events[j + 1][0] - events[i][0] <= 1.5:
            j += 1
        devs = {e[1] for e in events[i:j + 1]}
        if len(devs) >= 3:
            what = ", ".join(sorted(labels.get(d, str(d)) for d in devs))
            moments.append(Moment(events[i][0], "several_devices", True, f"{len(devs)} devices lost tracking or trust together: {what}"))
        i = j + 1
    for start, d, dur in losses:
        if dur >= 3.0 and d == target:
            moments.append(Moment(start, "head_tracker", True, f"head tracker lost tracking for {dur:.1f} s", d))
    moments.sort(key=lambda m: m.unix)
    return moments, devices, labels, trust_rows, target


def device_label(d: int, info: dict, is_target: bool) -> str:
    model, serial = info.get("model", ""), info.get("serial", "")
    if is_target:
        return "head tracker"
    if str(info.get("class")) == "1":
        return "headset"
    tail = serial.split("-")[-1][-4:] if serial else str(d)
    if "Knuckles" in model:
        return "Index " + ("left" if "Left" in model else "right")
    if "Controller" in model:
        return model.replace("Meta Quest Pro", "Quest").strip()
    return f"tracker {tail}"


def windows_from(moments, start: float, end: float):
    spans = []
    for m in moments:
        pre, post = MAJOR if m.major else MINOR
        spans.append([max(start, m.unix - pre), min(end, m.unix + post), [m]])
    spans.sort(key=lambda w: w[0])
    merged = []
    for w in spans:
        if merged and w[0] <= merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], w[1])
            merged[-1][2].extend(w[2])
        else:
            merged.append(w)
    return merged


KIND_RANK = ["user_marker", "universe_shift", "correction", "head_tracker", "several_devices", "calibration_step", "glitch"]


def window_kind(ms) -> str:
    return min((m.kind for m in ms), key=lambda k: KIND_RANK.index(k) if k in KIND_RANK else 99)


# ---- writing the windows (one pass over the session) -------------------------------------------

def header_bytes(h) -> bytes:
    return HEADER_STRUCT.pack(h.magic, h.format_version, h.record_size, h.header_size, 0, h.chunk_index, h.mono_start, h.unix_start,
                              h.session_id, h.chunk_seconds, h.version.encode("utf-8")[:24])


def write_windows(s: Session, wins, folders: list) -> int:
    """Trimmed chunks for every window, in one pass over the session's chunks."""
    carried_wfd: dict = {}  # device -> last WORLD_FROM_DRIVER record
    carried_app: dict = {}  # device -> last APPLIED record
    device_parts = []  # every DEVICE record so far (device info texts)
    started = [False] * len(wins)
    written = 0
    for idx in sorted(s.chunks):
        try:
            c = read_chunk(s.chunks[idx])
        except (OSError, ValueError):
            continue
        r = c.records
        if not len(r):
            continue
        r = r[np.argsort(r["t"], kind="stable")]
        u = c.header.mono_to_unix(r["t"])
        for k, (w0, w1, ms) in enumerate(wins):
            if u[-1] < w0 or u[0] > w1:
                continue
            inside = (u >= w0) & (u <= w1)
            keep = inside.copy()
            pose = (r["type"] == TYPE_POSE) & inside
            if pose.any():
                focus = np.array([m.unix for m in ms])
                idxs = np.flatnonzero(pose)
                near = np.min(np.abs(u[idxs][:, None] - focus[None, :]), axis=1) <= FULL_RATE_AROUND
                far = idxs[~near]
                if len(far):
                    bucket = np.floor(u[far] * POSE_HZ).astype(np.int64) * 64 + r["device"][far].astype(np.int64)
                    _, first_of = np.unique(bucket, return_index=True)
                    drop = np.ones(len(far), dtype=bool)
                    drop[first_of] = False
                    keep[far[drop]] = False
            sel = r[keep]
            if not started[k]:
                # what is in effect at the window start, so the window reads on its own
                before = u < w0
                wfd, app = dict(carried_wfd), dict(carried_app)
                for typ, store in ((TYPE_WFD, wfd), (TYPE_APPLIED, app)):
                    for i in np.flatnonzero((r["type"] == typ) & before):
                        store[int(r["device"][i])] = r[i:i + 1]
                parts = device_parts + [r[(r["type"] == TYPE_DEVICE) & before]]
                t0 = sel["t"][0] if len(sel) else r["t"][0]
                carried = [rec.copy() for rec in list(wfd.values()) + list(app.values())]
                for rec in carried:
                    rec["b"] = RECORD_CARRIED
                    rec["t"] = t0 - 1e-6
                head = [p for p in parts if len(p)] + carried
                if head:
                    sel = np.concatenate(head + [sel])
                started[k] = True
            folders[k].mkdir(parents=True, exist_ok=True)
            (folders[k] / s.chunks[idx].name).write_bytes(header_bytes(c.header) + sel.tobytes())
            written += len(sel)
        for typ, store in ((TYPE_WFD, carried_wfd), (TYPE_APPLIED, carried_app)):
            for i in np.flatnonzero(r["type"] == typ):
                store[int(r["device"][i])] = r[i:i + 1].copy()
        dev = r[r["type"] == TYPE_DEVICE]
        if len(dev):
            device_parts.append(dev.copy())
    return written


def write_window_texts(s: Session, w0: float, w1: float, ms, out: Path, overlay_json: Path | None, notes: list, log_lines: list):
    out.mkdir(parents=True, exist_ok=True)
    if overlay_json is not None and overlay_json.exists():
        shutil.copy2(overlay_json, out / "overlay.json")
    lines = [f"Session {s.sid}, {clock(w0)} to {clock(w1)}", ""]
    for m in sorted(ms, key=lambda m: m.unix):
        lines.append(f"{clock(m.unix)}  {'*' if m.major else ' '} {m.text}")
    if notes:
        lines += ["", "Your notes:"] + [f"  {n}" for n in notes]
    lines += ["", "* = major moment (kept from 120 s before to 60 s after); others 15 s each side.",
              f"Poses at full rate within {FULL_RATE_AROUND:.0f} s of a moment, at most {POSE_HZ:.0f} per second and device elsewhere."]
    (out / "summary.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    if notes:
        (out / "notes.txt").write_text("\n".join(notes) + "\n", encoding="utf-8")
    if log_lines:
        (out / "log.txt").write_text("".join(log_lines), encoding="utf-8")


# ---- overlay logs -------------------------------------------------------------------------------

LOG_NAME = re.compile(r"log_overlay_(\d{4})_(\d{2})_(\d{2})_(\d{2})_(\d{2})_(\d{2})\.log$")


def log_files(logs: Path):
    out = []
    for f in logs.glob("log_overlay_*.log"):
        m = LOG_NAME.search(f.name)
        if m:
            start = datetime(*map(int, m.groups()))
            out.append((start.timestamp(), f.stat().st_mtime, f))
    return sorted(out)


def iter_log(f: Path, start_unix: float):
    """(unix, line) for every line; a line without a time inherits the previous one."""
    day = datetime.fromtimestamp(start_unix).replace(hour=0, minute=0, second=0, microsecond=0)
    last = start_unix
    with open(f, encoding="utf-8", errors="replace") as fh:
        for line in fh:
            if len(line) > 12 and line[2] == ":" and line[5] == ":":
                try:
                    h, mi, sec = int(line[0:2]), int(line[3:5]), float(line[6:18].split()[0])
                    t = (day + timedelta(hours=h, minutes=mi, seconds=sec)).timestamp()
                    while t < last - 3600:  # past midnight
                        day += timedelta(days=1)
                        t = (day + timedelta(hours=h, minutes=mi, seconds=sec)).timestamp()
                    last = t
                except ValueError:
                    pass
            yield last, line


def log_lines_by_window(logs, wins):
    """The overlay log lines of every window (without the noise), one pass per log file."""
    out = [[] for _ in wins]
    if not wins:
        return out
    lo, hi = wins[0][0], wins[-1][1]
    for start, mtime, f in logs:
        if start > hi or mtime < lo:
            continue
        k = 0
        for t, line in iter_log(f, start):
            if t > hi:
                break
            while k < len(wins) and wins[k][1] < t:
                k += 1
            if k == len(wins):
                break
            if t >= wins[k][0] and not LOG_NOISE.search(line):
                out[k].append(line)
    return out


def condense_log(f: Path, start: float) -> tuple:
    before = f.stat().st_size
    tmp = f.with_suffix(".condensed.tmp")
    with open(tmp, "w", encoding="utf-8") as fo:
        for _, line in iter_log(f, start):
            if not LOG_NOISE.search(line):
                fo.write(line)
    os.replace(tmp, f)
    return before, f.stat().st_size


# ---- main ---------------------------------------------------------------------------------------

def delete_sources(s: Session, sid: str):
    if s.archive and s.archive.exists():
        shutil.rmtree(s.archive)
    for d in s.events:
        if d.exists():
            shutil.rmtree(d)
    for d in s.mixed_events:
        # a marker folder that spans a SteamVR restart: only this session's chunks go
        for f in d.glob(f"{sid}_*{CHUNK_EXTENSION}"):
            f.unlink()
        if not any(d.glob(f"*{CHUNK_EXTENSION}")):
            shutil.rmtree(d)
    for f in s.live:
        f.unlink(missing_ok=True)


def notes_for(events: list, w0: float, w1: float):
    out = []
    for d in events:
        p = d / "notes.txt"
        m = re.match(r"(\d{4}-\d{2}-\d{2})_(\d{2})-(\d{2})-(\d{2})_(\w+)$", d.name)
        if not m or not p.exists():
            continue
        t = datetime.strptime(f"{m.group(1)} {m.group(2)}:{m.group(3)}:{m.group(4)}", "%Y-%m-%d %H:%M:%S").timestamp()
        if w0 - 1 <= t <= w1 + 1:
            out += [f"{clock(t)} ({m.group(5)}): {line.strip()}" for line in p.read_text(encoding="utf-8", errors="replace").splitlines() if line.strip()]
    return out


def overlay_json_for(events: list, t: float) -> Path | None:
    best, best_d = None, None
    for d in events:
        p = d / "overlay.json"
        if not p.exists():
            continue
        m = re.match(r"(\d{4}-\d{2}-\d{2})_(\d{2})-(\d{2})-(\d{2})", d.name)
        dt = abs(datetime.strptime(" ".join([m.group(1), ":".join(m.groups()[1:4])]), "%Y-%m-%d %H:%M:%S").timestamp() - t) if m else 1e12
        if best is None or dt < best_d:
            best, best_d = p, dt
    return best


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=str(Path(os.environ.get("APPDATA", "")) / "space-calibrator" / "blackbox"))
    ap.add_argument("--logs-dir", default=str(Path(os.environ.get("APPDATA", "")) / "space-calibrator" / "logs"))
    ap.add_argument("--session", action="append", help="only these session ids")
    ap.add_argument("--apply", action="store_true", help="write the collection and delete the rest (default: print the plan)")
    ap.add_argument("--logs", action="store_true", help="also rewrite old overlay logs without the per-frame noise")
    ap.add_argument("--keep-sources", action="store_true", help="write the collection but delete nothing")
    args = ap.parse_args(argv)
    root, logs_dir = Path(args.root), Path(args.logs_dir)
    sessions = scan_root(root)
    busy = current_session(root, sessions)
    logs = log_files(logs_dir)
    total_before = total_after = 0
    for sid, s in sorted(sessions.items(), key=lambda kv: kv[1].name):
        if args.session and sid not in args.session:
            continue
        if sid in busy:
            print(f"{s.name}: being recorded, skipped")
            continue
        if not s.chunks:
            continue
        already = (root / "glitches" / s.name).exists()
        source_bytes = sum(f.stat().st_size for f in set(s.chunks.values()))
        if s.archive:
            source_bytes = dir_size(s.archive)
        source_bytes += sum(dir_size(d) for d in s.events)
        overlay_target = None
        ov_any = overlay_json_for(s.events + s.mixed_events, 0)
        if ov_any:
            try:
                cals = json.loads(ov_any.read_text(encoding="utf-8")).get("calibrations", [])
                if cals and int(cals[0]["target"]["index"]) < 64:
                    overlay_target = int(cals[0]["target"]["index"])
            except (ValueError, KeyError, TypeError):
                pass
        moments, devices, labels, trust_rows, target = find_moments(s, overlay_target)
        first = min(s.chunks)
        last = max(s.chunks)
        with open(s.chunks[first], "rb") as fh:
            start = HEADER_STRUCT.unpack(fh.read(HEADER_SIZE))[7]
        end = read_chunk(s.chunks[last])
        end = end.header.mono_to_unix(end.records["t"][-1:])[0] if len(end.records) else start
        wins = windows_from(moments, start, end)
        kept_s = sum(w[1] - w[0] for w in wins)
        majors = sum(1 for m in moments if m.major)
        print(f"{s.name}: {len(s.chunks)} chunks, {source_bytes / 1e9:.2f} GB, {(end - start) / 3600:.1f} h; "
              f"{len(moments)} moments ({majors} major) -> {len(wins)} windows, {kept_s / 60:.0f} min kept")
        kinds = {}
        for w in wins:
            kinds[window_kind(w[2])] = kinds.get(window_kind(w[2]), 0) + 1
        print("   windows by kind: " + ", ".join(f"{k} {v}" for k, v in sorted(kinds.items(), key=lambda kv: -kv[1])))
        if s.mixed_events:
            print(f"   left alone (hold other sessions too): {', '.join(d.name for d in s.mixed_events)}")
        total_before += source_bytes
        if not args.apply:
            if already:
                print("   already in the glitch collection; only its leftovers would be deleted")
            continue
        dest = root / "glitches" / s.name
        if already:
            # collected by an earlier run that did not get to delete everything: only the leftovers go
            print("   already in the glitch collection; removing what is left of it")
            if not args.keep_sources:
                delete_sources(s, sid)
            continue
        # written next to the collection first and moved in place only when every window reads back,
        # so an interrupted run leaves nothing half done (and deletes nothing)
        partial = root / "glitches" / (s.name + ".partial")
        if partial.exists():
            shutil.rmtree(partial)
        folders = [partial / f"{stamp(min(m.unix for m in ms))}_{window_kind(ms)}" for _, _, ms in wins]
        records = write_windows(s, wins, folders)
        texts = log_lines_by_window(logs, wins)
        for k, (w0, w1, ms) in enumerate(wins):
            write_window_texts(s, w0, w1, ms, folders[k], overlay_json_for(s.events + s.mixed_events, w0),
                               notes_for(s.events + s.mixed_events, w0, w1), texts[k])
        for folder in folders:
            for f in folder.glob(f"*{CHUNK_EXTENSION}"):
                read_chunk(f)
        partial.mkdir(parents=True, exist_ok=True)
        (partial / "README.txt").write_text(
            f"Glitch collection of SteamVR session {s.sid} ({s.name}), made {datetime.now():%Y-%m-%d %H:%M} by tools/glitch_collection.py.\n"
            f"One folder per window; summary.txt says what happened. Replay one with:\n"
            f"  bin-tests\\artifacts\\RelWithDebInfo\\spacecal-replay.exe run <folder> --guard %APPDATA%\\space-calibrator\\guard.json --hz 60\n",
            encoding="utf-8")
        os.replace(partial, dest)
        after = dir_size(dest)
        total_after += after
        print(f"   written: {len(wins)} windows, {records:,} records, {after / 1e6:.0f} MB")
        if args.keep_sources:
            continue
        delete_sources(s, sid)
        print(f"   deleted: {'archive, ' if s.archive else ''}{len(s.events)} event folder(s)")
    if args.apply:
        print(f"total: {total_before / 1e9:.2f} GB of recordings -> {total_after / 1e9:.2f} GB of glitch collection")
    if args.logs:
        current = logs[-1][2] if logs else None
        saved = 0
        for start, mtime, f in logs:
            if f == current or time.time() - mtime < 900:
                continue
            if args.apply:
                b, a = condense_log(f, start)
                saved += b - a
            else:
                saved += f.stat().st_size
        print(f"overlay logs: {'condensed, saved' if args.apply else 'would condense'} {saved / 1e6:.0f} MB{'' if args.apply else ' (upper bound)'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
