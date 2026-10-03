"""Replay a whole SteamVR session through spacecal-replay, in pieces, with a per-tick CSV each.

The session's chunks (keep-all archive, saved events and live/, the largest copy of each index) are
hard-linked into folders of ``--per`` chunks (no copies), each with an ``overlay.json`` from one of the
session's saved events, and every folder is replayed with ``--csv`` at below-normal priority. The CSVs
are what ``tools.session_report`` reads for the head tracker error.

Usage::

    python -m tools.replay_session 1a2b3c4d --exe bin-tests/artifacts/RelWithDebInfo/spacecal-replay.exe
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
BELOW_NORMAL = 0x00004000 if os.name == "nt" else 0


def chunk_files(root: Path, sid: str) -> dict:
    best = {}
    for f in root.rglob(f"{sid}_*.scb"):
        try:
            idx = int(f.stem.split("_")[1])
            size = f.stat().st_size
        except (ValueError, IndexError, OSError):
            continue
        rank = (size, 1 if f.parent.parent.name == "sessions" else 0)  # the archive copy wins ties
        if idx not in best or rank > best[idx][1]:
            best[idx] = (f, rank)
    return {i: v[0] for i, v in sorted(best.items())}


def overlay_json(root: Path, files: dict, wanted: str | None) -> Path:
    events = root / "events"
    if wanted:
        return events / wanted / "overlay.json"
    # a saved event of this session: a folder under events/ that holds one of its chunks
    sid = next(iter(files.values())).stem.split("_")[0]
    for d in sorted(events.iterdir()) if events.is_dir() else []:
        if (d / "overlay.json").exists() and any(d.glob(f"{sid}_*.scb")):
            return d / "overlay.json"
    raise SystemExit("no saved event of this session with an overlay.json; pass --overlay-event")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--exe", default=str(REPO / "bin-tests" / "artifacts" / "RelWithDebInfo" / "spacecal-replay.exe"))
    ap.add_argument("--root", default=str(Path(os.environ.get("APPDATA", "")) / "space-calibrator" / "blackbox"))
    ap.add_argument("--guard", default=str(Path(os.environ.get("APPDATA", "")) / "space-calibrator" / "guard.json"))
    ap.add_argument("--overlay-event", default=None, help="event folder whose overlay.json names the calibration's devices")
    ap.add_argument("--per", type=int, default=20, help="chunks per piece")
    ap.add_argument("--hz", default="60")
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--segments", default=None, help="default logs/replay_segments/<session>")
    ap.add_argument("--out", default=None, help="default logs/replay_csv/<session>")
    args = ap.parse_args(argv)

    root = Path(args.root)
    files = chunk_files(root, args.session)
    if not files:
        raise SystemExit(f"no chunks of session {args.session} under {root}")
    ov = overlay_json(root, files, args.overlay_event)
    seg_root = Path(args.segments or REPO / "logs" / "replay_segments" / args.session)
    out = Path(args.out or REPO / "logs" / "replay_csv" / args.session)
    out.mkdir(parents=True, exist_ok=True)
    segs = []
    for start in range(0, max(files) + 1, args.per):
        members = [i for i in files if start <= i < start + args.per]
        if not members:
            continue
        d = seg_root / f"seg_{start:04d}"
        d.mkdir(parents=True, exist_ok=True)
        for i in members:
            dst = d / files[i].name
            if not dst.exists():
                try:
                    os.link(files[i], dst)
                except OSError:
                    shutil.copy2(files[i], dst)
        shutil.copy2(ov, d / "overlay.json")
        segs.append(d)

    def one(seg: Path):
        t0 = time.time()
        cmd = [args.exe, "run", str(seg), "--guard", args.guard, "--hz", args.hz, "--csv", str(out / f"{seg.name}.csv")]
        r = subprocess.run(cmd, capture_output=True, text=True, creationflags=BELOW_NORMAL)
        (out / f"{seg.name}.txt").write_text(r.stdout + ("\nSTDERR:\n" + r.stderr if r.stderr else ""), encoding="utf-8")
        return seg.name, r.returncode, time.time() - t0

    failed = 0
    with ThreadPoolExecutor(max_workers=args.workers) as ex:
        for name, rc, dur in ex.map(one, segs):
            print(name, "ok" if rc == 0 else f"exit {rc}", f"{dur:.0f} s", flush=True)
            failed += rc != 0
    print(f"{len(segs)} pieces replayed into {out}, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
