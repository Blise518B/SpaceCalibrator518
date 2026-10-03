"""Standard plot for one saved black-box event (fork, docs/DESIGN.md section 10).

    python tools/plot_event.py <event folder> [--out plot.png] [--devices 0,3,5] [--window 60]

One panel per device with its position (x, y, z) over time relative to the marker, the
marker as a vertical line, TRUST transitions as shaded bands (once the trust layer exists),
and the pairwise distance between the HMD (device 0) and every other device in the last panel.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from tools.blackbox import TRUST_STATES, load_event  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("--out", help="save to this PNG instead of showing a window")
    ap.add_argument("--devices", help="comma separated device indices (default: all with poses)")
    ap.add_argument("--window", type=float, default=60.0, help="seconds to show on each side of the marker")
    args = ap.parse_args()

    import matplotlib

    if args.out:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ev = load_event(args.folder)
    poses = ev.poses
    if poses.empty:
        print("no POSE records in", args.folder)
        return 1

    t_mark = ev.t_mark
    if t_mark is None:
        t_mark = float(poses["t"].iloc[0])
    poses = poses.assign(rel=poses["t"] - t_mark)
    poses = poses[(poses["rel"] >= -args.window) & (poses["rel"] <= args.window)]

    devices = sorted(int(d) for d in poses["device"].unique())
    if args.devices:
        wanted = {int(x) for x in args.devices.split(",")}
        devices = [d for d in devices if d in wanted]

    trust = ev.trust

    n = len(devices) + 1
    fig, axes = plt.subplots(n, 1, figsize=(14, 2.2 * n), sharex=True)
    if n == 1:
        axes = [axes]

    for ax, dev in zip(axes, devices):
        p = poses[poses["device"] == dev]
        info = ev.devices.get(dev, {})
        label = f"device {dev} {info.get('sys', '')} {info.get('model', '')} {info.get('serial', '')}".strip()
        ax.plot(p["rel"], p["px"], lw=0.8, label="x")
        ax.plot(p["rel"], p["py"], lw=0.8, label="y")
        ax.plot(p["rel"], p["pz"], lw=0.8, label="z")
        bad = p[~p["valid"] | (p["result"] != 200)]
        if len(bad):
            ax.scatter(bad["rel"], bad["py"], s=6, color="red", label="not Running_OK", zorder=3)
        ax.axvline(0.0, color="black", ls="--", lw=1)
        for m in ev.markers.itertuples():
            ax.axvline(m.t - t_mark, color="black", ls=":", lw=0.8)
        if not trust.empty:
            tt = trust[trust["device"] == dev]
            for row in tt.itertuples():
                color = {"TRUSTED": "green", "SUSPECT": "orange", "UNTRUSTED": "red", "RECOVERING": "blue"}.get(row.new_state, "gray")
                ax.axvline(row.t - t_mark, color=color, lw=1.2, alpha=0.8)
        ax.set_ylabel("m")
        ax.set_title(label, fontsize=9, loc="left")
        ax.legend(loc="upper right", fontsize=7, ncol=4)
        ax.grid(alpha=0.3)

    # last panel: distance from device 0 (usually the HMD) to every other device
    ax = axes[-1]
    ref = poses[poses["device"] == devices[0]] if devices else poses.iloc[0:0]
    for dev in devices[1:]:
        p = poses[poses["device"] == dev]
        if ref.empty or p.empty:
            continue
        # resample the reference to this device's timestamps
        rx = np.interp(p["t"], ref["t"], ref["px"])
        ry = np.interp(p["t"], ref["t"], ref["py"])
        rz = np.interp(p["t"], ref["t"], ref["pz"])
        d = np.sqrt((p["px"] - rx) ** 2 + (p["py"] - ry) ** 2 + (p["pz"] - rz) ** 2)
        ax.plot(p["rel"], d, lw=0.8, label=f"|d0 - d{dev}|")
    ax.axvline(0.0, color="black", ls="--", lw=1)
    ax.set_ylabel("m")
    ax.set_xlabel("seconds relative to marker")
    ax.set_title("distance to device 0 (same universe only is meaningful)", fontsize=9, loc="left")
    ax.legend(loc="upper right", fontsize=7, ncol=4)
    ax.grid(alpha=0.3)

    fig.suptitle(f"{Path(args.folder).name}  ({len(ev.chunks)} chunks, {len(ev.records)} records)", fontsize=10)
    fig.tight_layout()
    if args.out:
        fig.savefig(args.out, dpi=120)
        print("wrote", args.out)
    else:
        plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
