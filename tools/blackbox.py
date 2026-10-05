"""Reader for Space Calibrator black-box recordings (fork, docs/DESIGN.md section 4).

Mirrors src/common/blackbox_format.h byte for byte. A recording is a folder of ``.scb`` chunk
files: a saved event (plus ``driver.json`` / ``overlay.json``), ``live/``, or one session of the
keep-all archive (``sessions/<local start>_<session id>/``, NTFS-compressed, read as usual). Chunks may end in a
truncated record because they are copied while still growing; the tail is ignored.

Usage as a library::

    from tools.blackbox import load_event
    ev = load_event("C:/Users/me/AppData/Roaming/space-calibrator/blackbox/events/2026-09-18_21-04-11_hotkey")
    ev.poses            # pandas DataFrame, one row per POSE record
    ev.markers          # DataFrame of MARKER records
    ev.devices          # {index: {"class": .., "sys": .., "model": .., "serial": ..}}
    ev.calibrations     # DataFrame of CALIBRATION records: trigger, outcome, error, RMS, change vs active
    ev.device_state     # DataFrame of DEVICE_STATE records (connect / disconnect)
    ev.lifecycle        # DataFrame of LIFECYCLE records (recorder start/stop, overlay, settings)

Usage on the command line::

    python -m tools.blackbox info  <event folder or chunk file>
    python -m tools.blackbox csv   <event folder> out.csv
"""

from __future__ import annotations

import json
import struct
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import numpy as np

try:
    import pandas as pd
except ImportError:  # pandas is optional for the low-level reader
    pd = None

MAGIC = 0x42424353  # "SCBB"
FORMAT_VERSION = 2  # 2: CALIBRATION / DEVICE_STATE / LIFECYCLE, POSE result in `code`
RECORD_SIZE = 80
HEADER_SIZE = 64
TEXT_PAYLOAD_SIZE = 64
CHUNK_EXTENSION = ".scb"

HEADER_STRUCT = struct.Struct("<IHHHHIddII24s")
assert HEADER_STRUCT.size == HEADER_SIZE

RECORD_DTYPE = np.dtype(
    [
        ("t", "<f8"),
        ("type", "u1"),
        ("device", "u1"),
        ("a", "u1"),
        ("b", "u1"),
        ("code", "<u2"),
        ("reserved", "<u2"),
        ("f0", "<f4"),
        ("v", "<f4", (13,)),
        ("pad", "u1", (8,)),
    ]
)
assert RECORD_DTYPE.itemsize == RECORD_SIZE

RECORD_TYPES = {
    0: "NONE",
    1: "POSE",
    2: "WORLD_FROM_DRIVER",
    3: "APPLIED",
    4: "TRUST",
    5: "MARKER",
    6: "DEVICE",
    7: "TEXT",
    8: "SESSION",
    9: "CALIBRATION",
    10: "DEVICE_STATE",
    11: "LIFECYCLE",
}
TYPE_POSE, TYPE_WFD, TYPE_APPLIED, TYPE_TRUST, TYPE_MARKER, TYPE_DEVICE, TYPE_TEXT, TYPE_SESSION = range(1, 9)
RECORD_CARRIED = 1  # WORLD_FROM_DRIVER / APPLIED: b of a record repeated at a chunk start (blackbox_format.h)
TYPE_CALIBRATION, TYPE_DEVICE_STATE, TYPE_LIFECYCLE = 9, 10, 11

CALIB_TRIGGERS = {0: "unknown", 1: "standard", 2: "continuous", 3: "startup", 4: "trigger_hold", 5: "manual", 6: "playspace_jump", 7: "world_from_driver_jump", 8: "head_mount_fix"}
CALIB_OUTCOMES = {0: "rejected", 1: "applied", 2: "forced", 3: "skipped", 4: "corrected"}
CALIB_OUTCOME_MASK = 0x0F
CALIB_FLAG_CONTINUOUS = 1 << 4
CALIB_FLAG_RELATIVE = 1 << 5
# the overlay's CalibrationError (src/overlay/calibration.h)
CALIB_ERRORS = {
    0: "none", 1: "lack_of_rotational_variance", 2: "lack_of_translation_variance", 3: "rms_too_high",
    4: "worse_rms_than_last", 5: "axis_variance_too_high", 6: "worse_axis_variance_than_last",
    7: "bad_relative_calibration", 8: "device_untrusted", 9: "waiting_for_triggers", 10: "head_mount_disagrees", 11: "unknown",
}
LIFECYCLE_KINDS = {0: "none", 1: "recorder_start", 2: "recorder_stop", 3: "overlay_connected", 4: "params", 5: "archive_cap_reached", 6: "leftovers", 7: "ring_gap"}

MARKER_SOURCES = {0: "unknown", 1: "hotkey", 2: "overlay", 3: "triggers", 4: "auto", 5: "synthetic", 6: "shutdown"}
TEXT_KINDS = {0: "device_info", 1: "marker_folder", 2: "note", 3: "marker_note"}  # marker_note: "<marker t> <user's note>"
TRUST_STATES = {0: "TRUSTED", 1: "SUSPECT", 2: "UNTRUSTED", 3: "RECOVERING"}
TRUST_REASONS = {
    0: "none", 1: "jump_single", 2: "jump_ambiguous", 3: "jump_unverifiable", 4: "back_on_path",
    5: "confirm_timeout", 6: "residual_ok", 7: "jump_back", 8: "solve_agrees", 9: "solve_disagrees",
    10: "manual", 11: "device_lost", 12: "distance_odd_one_out", 13: "residual_high", 14: "quiet",
    15: "jump_while_recovering",
}
TRACKING_RESULTS = {
    1: "Uninitialized",
    100: "Calibrating_InProgress",
    101: "Calibrating_OutOfRange",
    200: "Running_OK",
    201: "Running_OutOfRange",
    300: "Fallback_RotationOnly",
}

POSE_FLAG_VALID = 1 << 0
POSE_FLAG_CONNECTED = 1 << 1
POSE_FLAG_DRIFT_IN_YAW = 1 << 2
POSE_FLAG_HEAD_MODEL = 1 << 3


@dataclass
class ChunkHeader:
    magic: int
    format_version: int
    record_size: int
    header_size: int
    chunk_index: int
    mono_start: float
    unix_start: float
    session_id: int
    chunk_seconds: int
    version: str

    def mono_to_unix(self, t: np.ndarray) -> np.ndarray:
        return self.unix_start + (t - self.mono_start)


@dataclass
class Chunk:
    path: Path
    header: ChunkHeader
    records: np.ndarray  # structured array with RECORD_DTYPE
    truncated_tail: bool = False


@dataclass
class Event:
    folder: Path
    chunks: List[Chunk]
    driver_json: Optional[dict] = None
    overlay_json: Optional[dict] = None
    records: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=RECORD_DTYPE))
    unix: np.ndarray = field(default_factory=lambda: np.zeros(0))
    devices: Dict[int, dict] = field(default_factory=dict)
    texts: List[dict] = field(default_factory=list)

    # ---- convenience frames (pandas) ------------------------------------------------------
    @property
    def poses(self):
        return records_to_frame(self.records, self.unix, TYPE_POSE)

    @property
    def markers(self):
        return records_to_frame(self.records, self.unix, TYPE_MARKER)

    @property
    def trust(self):
        return records_to_frame(self.records, self.unix, TYPE_TRUST)

    @property
    def applied(self):
        return records_to_frame(self.records, self.unix, TYPE_APPLIED)

    @property
    def world_from_driver(self):
        return records_to_frame(self.records, self.unix, TYPE_WFD)

    @property
    def session(self):
        return records_to_frame(self.records, self.unix, TYPE_SESSION)

    @property
    def calibrations(self):
        return records_to_frame(self.records, self.unix, TYPE_CALIBRATION)

    @property
    def device_state(self):
        return records_to_frame(self.records, self.unix, TYPE_DEVICE_STATE)

    @property
    def lifecycle(self):
        return records_to_frame(self.records, self.unix, TYPE_LIFECYCLE)

    @property
    def t_mark(self) -> Optional[float]:
        """Monotonic time of the first marker (from driver.json, else the first MARKER record)."""
        if self.driver_json and "t_mark_mono" in self.driver_json:
            return float(self.driver_json["t_mark_mono"])
        m = self.records[self.records["type"] == TYPE_MARKER]
        return float(m["t"][0]) if len(m) else None


def read_chunk(path: Path | str) -> Chunk:
    path = Path(path)
    data = path.read_bytes()
    if len(data) < HEADER_SIZE:
        raise ValueError(f"{path}: shorter than a chunk header")
    magic, fver, rsize, hsize, _res, cidx, mono, unix, sid, csec, ver = HEADER_STRUCT.unpack_from(data, 0)
    if magic != MAGIC:
        raise ValueError(f"{path}: bad magic 0x{magic:08x}")
    if rsize != RECORD_SIZE or hsize != HEADER_SIZE:
        raise ValueError(f"{path}: unsupported layout (record {rsize}, header {hsize})")
    header = ChunkHeader(magic, fver, rsize, hsize, cidx, mono, unix, sid, csec, ver.split(b"\0", 1)[0].decode("utf-8", "replace"))
    body = data[hsize:]
    n = len(body) // RECORD_SIZE
    records = np.frombuffer(body[: n * RECORD_SIZE], dtype=RECORD_DTYPE).copy()
    return Chunk(path, header, records, truncated_tail=(len(body) % RECORD_SIZE) != 0)


def record_text(rec: np.void) -> str:
    """Text payload of a DEVICE/TEXT record (bytes 16..80, NUL padded)."""
    raw = rec.tobytes()[16 : 16 + TEXT_PAYLOAD_SIZE]
    return raw.split(b"\0", 1)[0].decode("utf-8", "replace")


def iter_chunk_files(folder: Path) -> Iterable[Path]:
    return sorted(p for p in Path(folder).iterdir() if p.suffix == CHUNK_EXTENSION)


def _assemble_texts(records: np.ndarray, unix: np.ndarray) -> List[dict]:
    """Joins multi-record DEVICE/TEXT strings. Records of one string share t, device, kind."""
    texts: List[dict] = []
    mask = (records["type"] == TYPE_DEVICE) | (records["type"] == TYPE_TEXT)
    idx = np.nonzero(mask)[0]
    pending: Dict[tuple, dict] = {}
    for i in idx:
        r = records[i]
        key = (float(r["t"]), int(r["device"]), int(r["type"]), int(r["a"]))
        entry = pending.get(key)
        if entry is None:
            entry = {"t": key[0], "unix": float(unix[i]), "device": key[1], "type": RECORD_TYPES[key[2]], "kind": TEXT_KINDS.get(key[3], str(key[3])), "parts": {}, "total": int(r["code"])}
            pending[key] = entry
        entry["parts"][int(r["b"])] = record_text(r)
    for entry in pending.values():
        entry["text"] = "".join(entry["parts"][k] for k in sorted(entry["parts"]))
        del entry["parts"]
        texts.append(entry)
    texts.sort(key=lambda e: e["t"])
    return texts


def _parse_device_info(text: str) -> dict:
    out = {}
    for part in text.split(";"):
        if "=" in part:
            k, v = part.split("=", 1)
            out[k] = v
    if "class" in out:
        try:
            out["class"] = int(out["class"])
        except ValueError:
            pass
    return out


def load_event(folder: Path | str) -> Event:
    """Loads every chunk of an event (or live) folder into one time-sorted record array."""
    folder = Path(folder)
    chunks = [read_chunk(p) for p in iter_chunk_files(folder)]
    ev = Event(folder=folder, chunks=chunks)
    for name in ("driver.json", "overlay.json"):
        p = folder / name
        if p.exists():
            try:
                setattr(ev, name.replace(".", "_"), json.loads(p.read_text(encoding="utf-8")))
            except json.JSONDecodeError:
                pass
    if not chunks:
        return ev
    parts = [c.records for c in chunks]
    unix_parts = [c.header.mono_to_unix(c.records["t"]) for c in chunks]
    records = np.concatenate(parts)
    unix = np.concatenate(unix_parts)
    order = np.argsort(records["t"], kind="stable")
    ev.records = records[order]
    ev.unix = unix[order]
    ev.texts = _assemble_texts(ev.records, ev.unix)
    for t in ev.texts:
        if t["type"] == "DEVICE":
            ev.devices[t["device"]] = _parse_device_info(t["text"])
    return ev


def pose_results(r: np.ndarray) -> np.ndarray:
    """Tracking result per POSE record: `code` since format 2 (`a` wraps above 255)."""
    return np.where(r["code"] != 0, r["code"], r["a"]).astype(int)


def _clock(unix_s: float) -> str:
    from datetime import datetime

    return datetime.fromtimestamp(float(unix_s)).strftime("%Y-%m-%d %H:%M:%S")


def records_to_frame(records: np.ndarray, unix: np.ndarray, rtype: int):
    """One DataFrame per record type with named columns."""
    if pd is None:
        raise ImportError("pandas is required for DataFrame access (pip install -r tools/requirements.txt)")
    mask = records["type"] == rtype
    r = records[mask]
    u = unix[mask]
    base = {"t": r["t"], "unix": u, "device": r["device"]}
    if rtype == TYPE_POSE:
        v = r["v"]
        result = pose_results(r)
        cols = {
            **base,
            "result": result,
            "result_name": [TRACKING_RESULTS.get(int(x), str(x)) for x in result],
            "valid": (r["b"] & POSE_FLAG_VALID) != 0,
            "connected": (r["b"] & POSE_FLAG_CONNECTED) != 0,
            "drift_in_yaw": (r["b"] & POSE_FLAG_DRIFT_IN_YAW) != 0,
            "pose_time_offset": r["f0"],
            "px": v[:, 0], "py": v[:, 1], "pz": v[:, 2],
            "qw": v[:, 3], "qx": v[:, 4], "qy": v[:, 5], "qz": v[:, 6],
            "vx": v[:, 7], "vy": v[:, 8], "vz": v[:, 9],
            "wx": v[:, 10], "wy": v[:, 11], "wz": v[:, 12],
        }
    elif rtype in (TYPE_WFD, TYPE_APPLIED):
        v = r["v"]
        cols = {**base, "tx": v[:, 0], "ty": v[:, 1], "tz": v[:, 2], "qw": v[:, 3], "qx": v[:, 4], "qy": v[:, 5], "qz": v[:, 6],
                # repeated by the recorder at a chunk start (every chunk carries the transforms in effect), not a change
                "carried": r["b"] == RECORD_CARRIED}
        if rtype == TYPE_APPLIED:
            cols["blend"] = r["a"]
            cols["scale"] = r["f0"]
    elif rtype == TYPE_TRUST:
        v = r["v"]
        cols = {
            **base,
            "old_state": [TRUST_STATES.get(int(x), str(x)) for x in r["a"]],
            "new_state": [TRUST_STATES.get(int(x), str(x)) for x in r["b"]],
            "reason": r["code"],
            "reason_name": [TRUST_REASONS.get(int(x), str(x)) for x in r["code"]],
            "residual_m": r["f0"],
            "x0": v[:, 0], "x1": v[:, 1], "x2": v[:, 2],
        }
    elif rtype == TYPE_MARKER:
        cols = {"t": r["t"], "unix": u, "source": [MARKER_SOURCES.get(int(x), str(x)) for x in r["a"]], "label": r["code"]}
    elif rtype == TYPE_SESSION:
        v = r["v"]
        cols = {"t": r["t"], "unix": u, "dropped": v[:, 0], "ring_high_water": v[:, 1], "flush_interval_s": v[:, 2]}
    elif rtype == TYPE_CALIBRATION:
        v = r["v"]
        prev = v[:, 0].astype(np.float64)
        cols = {
            "t": r["t"], "unix": u,
            "target": r["device"], "reference": r["reserved"],
            "trigger": [CALIB_TRIGGERS.get(int(x), str(x)) for x in r["a"]],
            "outcome": [CALIB_OUTCOMES.get(int(x) & CALIB_OUTCOME_MASK, str(x)) for x in r["b"]],
            "continuous": (r["b"] & CALIB_FLAG_CONTINUOUS) != 0,
            "relative": (r["b"] & CALIB_FLAG_RELATIVE) != 0,
            "error": r["code"],
            "error_name": [CALIB_ERRORS.get(int(x), str(x)) for x in r["code"]],
            "rms_m": r["f0"],
            "prev_rms_m": np.where(prev < 0, np.nan, prev),
            "samples": v[:, 1].astype(int),
            "delta_trans_m": v[:, 2], "delta_rot_deg": v[:, 3],
            "tx": v[:, 4], "ty": v[:, 5], "tz": v[:, 6],
            "qw": v[:, 7], "qx": v[:, 8], "qy": v[:, 9], "qz": v[:, 10],
            "axis_variance": v[:, 11], "prev_axis_variance": v[:, 12],
        }
    elif rtype == TYPE_DEVICE_STATE:
        cols = {
            **base,
            "connected": r["a"] == 1,
            "previous": [None if int(x) == 255 else bool(x) for x in r["b"]],
            "result": r["code"],
            "result_name": [TRACKING_RESULTS.get(int(x), str(x)) for x in r["code"]],
        }
    elif rtype == TYPE_LIFECYCLE:
        v = r["v"]
        cols = {"t": r["t"], "unix": u, "kind": [LIFECYCLE_KINDS.get(int(x), str(x)) for x in r["a"]], "code": r["code"], "f0": r["f0"]}
        for i in range(7):
            cols[f"v{i}"] = v[:, i]
    else:
        cols = base
    return pd.DataFrame(cols)


# ---- command line --------------------------------------------------------------------------

def _info(target: Path) -> int:
    if target.is_file():
        c = read_chunk(target)
        h = c.header
        print(f"{target.name}: session {h.session_id:08x} chunk {h.chunk_index} version '{h.version}' records {len(c.records)} truncated_tail {c.truncated_tail}")
        counts = {RECORD_TYPES.get(int(t), str(t)): int(n) for t, n in zip(*np.unique(c.records["type"], return_counts=True))}
        print("  by type:", counts)
        return 0
    ev = load_event(target)
    print(f"{target}: {len(ev.chunks)} chunks, {len(ev.records)} records")
    if ev.driver_json:
        print(f"  driver.json: source={ev.driver_json.get('source')} incomplete={ev.driver_json.get('incomplete')} t_mark_mono={ev.driver_json.get('t_mark_mono')}")
    if ev.overlay_json:
        print(f"  overlay.json: note={ev.overlay_json.get('note')!r} calibrations={len(ev.overlay_json.get('calibrations', []))}")
    if len(ev.records):
        counts = {RECORD_TYPES.get(int(t), str(t)): int(n) for t, n in zip(*np.unique(ev.records["type"], return_counts=True))}
        print("  by type:", counts)
        span = float(ev.records["t"][-1] - ev.records["t"][0])
        print(f"  span: {span:.1f} s")
        poses = ev.records[ev.records["type"] == TYPE_POSE]
        for dev, n in zip(*np.unique(poses["device"], return_counts=True)):
            info = ev.devices.get(int(dev), {})
            rate = n / span if span > 0 else 0.0
            print(f"  device {int(dev):2d}: {int(n):7d} poses ({rate:6.1f} Hz)  {info.get('sys', '?')} {info.get('model', '')} {info.get('serial', '')}")
        for m in ev.records[ev.records["type"] == TYPE_MARKER]:
            print(f"  marker at t={float(m['t']):.3f} source={MARKER_SOURCES.get(int(m['a']))} label={int(m['code'])}")
        for t in ev.texts:
            if t["kind"] == "note":
                print(f"  {_clock(t['unix'])} note: {t['text']}")
            elif t["kind"] == "marker_note":
                _, _, note = t["text"].partition(" ")
                print(f"  {_clock(t['unix'])} your note: {note}")
        _info_calibrations(ev)
        mask = ev.records["type"] == TYPE_DEVICE_STATE
        for r, u in list(zip(ev.records[mask], ev.unix[mask]))[:30]:
            info = ev.devices.get(int(r["device"]), {})
            state = "connected" if int(r["a"]) == 1 else "disconnected"
            print(f"  {_clock(u)} device {int(r['device']):2d} {state:12s} {info.get('model', '')} {info.get('serial', '')}")
        mask = ev.records["type"] == TYPE_LIFECYCLE
        for r, u in zip(ev.records[mask], ev.unix[mask]):
            print(f"  {_clock(u)} {LIFECYCLE_KINDS.get(int(r['a']), str(int(r['a'])))} code={int(r['code'])}")
    return 0


def _info_calibrations(ev: Event) -> None:
    cal = ev.records[ev.records["type"] == TYPE_CALIBRATION]
    if not len(cal):
        return
    outcomes: Dict[str, int] = {}
    triggers: Dict[str, int] = {}
    for r in cal:
        o = CALIB_OUTCOMES.get(int(r["b"]) & CALIB_OUTCOME_MASK, "?")
        outcomes[o] = outcomes.get(o, 0) + 1
        tr = CALIB_TRIGGERS.get(int(r["a"]), "?")
        triggers[tr] = triggers.get(tr, 0) + 1
    print(f"  calibration attempts: {len(cal)}  outcomes {outcomes}  triggers {triggers}")
    unix = ev.unix[ev.records["type"] == TYPE_CALIBRATION]
    shown = 0
    for r, u in zip(cal, unix):
        o = CALIB_OUTCOMES.get(int(r["b"]) & CALIB_OUTCOME_MASK, "?")
        if o == "rejected" and CALIB_TRIGGERS.get(int(r["a"])) == "continuous":
            continue  # the routine rejections are in the frame; the listing shows what moved the calibration
        prev = float(r["v"][0])
        prev_s = f"{prev * 100:.2f} cm" if prev >= 0 else "none"
        rms = float(r["f0"])
        rms_s = f"{rms * 100:.2f} cm" if np.isfinite(rms) else "-"
        print(f"  {_clock(u)} {CALIB_TRIGGERS.get(int(r['a']), '?'):22s} {o:9s} rms {rms_s:>9s} (was {prev_s}) change {float(r['v'][2]) * 100:.2f} cm / {float(r['v'][3]):.2f} deg  {CALIB_ERRORS.get(int(r['code']), '')}")
        shown += 1
        if shown >= 40:
            print("  ... (more in Event.calibrations)")
            break


def _csv(target: Path, out: Path) -> int:
    ev = load_event(target)
    ev.poses.to_csv(out, index=False)
    print(f"wrote {out} ({len(ev.poses)} pose rows)")
    return 0


def main(argv: List[str]) -> int:
    if len(argv) < 2 or argv[0] not in ("info", "csv"):
        print(__doc__)
        return 2
    if argv[0] == "info":
        return _info(Path(argv[1]))
    if argv[0] == "csv":
        if len(argv) < 3:
            print("csv needs <event folder> <out.csv>")
            return 2
        return _csv(Path(argv[1]), Path(argv[2]))
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
