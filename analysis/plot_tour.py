#!/usr/bin/env python3
"""Plot one mission tour over its frozen map: the ground-truth track, the executive's
belief, the raw odometry anchored in the map frame (odom_*.csv: rf2o since 2026-09-24, vio_*.csv
before), the goals with their verdict, and the impact / claim events. The picture the
numbers of verify_missions.py describe.

  python3 analysis/plot_tour.py --missions runs/raw/missions_<seed>_<ts>.csv
      [--gt <a7_gt csv>] [--vio <vio csv>] [--map worlds/maps/world_<seed>.yaml] [--out png]

The ground-truth file is paired by wall-clock span like verify_missions.py does; the
odometry and IMU-excitation files are taken from the same run id as the missions file.
"""
import argparse
import csv
import glob
import math
import os
import sys
import warnings

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "missions"))
from gridmap import GridMap  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ARRIVE_M = 1.5


def pick_gt(missions_path, rows):
    seed = os.path.basename(missions_path).split("_")[1]
    t0 = min(float(r["epoch_start"]) for r in rows)
    t1 = max(float(r["epoch_end"]) for r in rows)
    for c in sorted(glob.glob(os.path.join(os.path.dirname(missions_path), f"*_gt_{seed}_*.csv")), reverse=True):
        w = np.genfromtxt(c, delimiter=",", skip_header=2, usecols=(1,))
        if len(w) and w[0] - 60 <= t0 and t1 <= w[-1] + 60:
            return c
    raise SystemExit("no ground-truth CSV covers this tour; pass --gt")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--missions", required=True)
    ap.add_argument("--gt", default=None)
    ap.add_argument("--vio", default=None)
    ap.add_argument("--map", default=None)
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    rows = [r for r in csv.DictReader(open(a.missions))]
    base = os.path.basename(a.missions)[len("missions_"):-4]          # <seed>_<ts>
    seed = base.split("_")[0]
    gt_path = a.gt or pick_gt(a.missions, rows)
    vio_path = a.vio or next((c for c in (os.path.join(os.path.dirname(a.missions), f"odom_{base}.csv"),
                                          os.path.join(os.path.dirname(a.missions), f"vio_{base}.csv")) if os.path.isfile(c)), "")
    belief_path = os.path.join(os.path.dirname(a.missions), f"belief_{base}.csv")
    imu_path = os.path.join(os.path.dirname(a.missions), f"imu_excitation_{base}.csv")
    map_path = a.map or os.path.join(ROOT, "worlds", "maps", f"world_{seed}.yaml")
    out = a.out or os.path.join(ROOT, "analysis", f"tour_{base}.png")

    gm = GridMap(map_path)
    gt = np.genfromtxt(gt_path, delimiter=",", skip_header=2)
    gw, gx, gy = gt[:, 1], gt[:, 2], gt[:, 3]
    t_first = min(float(r["epoch_start"]) for r in rows) - 30
    t_last = max(float(r["epoch_end"]) for r in rows) + 30
    sel = (gw >= t_first) & (gw <= t_last)

    fig, ax = plt.subplots(figsize=(10, 10))
    img = np.full(gm.cls.shape, 0.5)
    img[gm.cls == 0] = 1.0
    img[gm.cls == 1] = 0.0
    ext = [gm.origin[0], gm.origin[0] + gm.w * gm.resolution, gm.origin[1], gm.origin[1] + gm.h * gm.resolution]
    ax.imshow(img, cmap="gray", origin="lower", extent=ext, vmin=0, vmax=1)
    ax.plot(gx[sel], gy[sel], "-", color="#1f77b4", lw=1.8, label="ground truth")
    if os.path.isfile(belief_path):
        b = np.genfromtxt(belief_path, delimiter=",", skip_header=2)
        if b.ndim == 2 and len(b):
            ok = (b[:, 1] >= t_first) & (b[:, 1] <= t_last)
            ax.plot(b[ok, 2], b[ok, 3], "-", color="#d62828", lw=1.3, alpha=0.9, label="belief consumed by the executive")
    if os.path.isfile(vio_path):
        v = np.genfromtxt(vio_path, delimiter=",", skip_header=2)
        if v.ndim == 2 and len(v):
            ok = ~np.isnan(v[:, 6]) & (v[:, 1] >= t_first) & (v[:, 1] <= t_last)
            ax.plot(v[ok, 6], v[ok, 7], "-", color="#e0a400", lw=0.8, alpha=0.8,
                    label="odometry dead reckoning (anchored at the spawn)")
    ax.set_xlim(ext[0], ext[1]); ax.set_ylim(ext[2], ext[3])
    imu = None
    if os.path.isfile(imu_path):
        with warnings.catch_warnings():                 # the wheeled robot's file has its header only
            warnings.simplefilter("ignore", UserWarning)
            imu = np.genfromtxt(imu_path, delimiter=",", skip_header=2, ndmin=2)
    if imu is not None and imu.shape[0] and imu.shape[1] > 2:
        for row in imu[imu[:, 2] > 5.0]:
            i = int(np.clip(np.searchsorted(gw, row[1]), 0, len(gw) - 1))
            ax.plot(gx[i], gy[i], "x", color="#e0a400", ms=9, mew=2)
        ax.plot([], [], "x", color="#e0a400", ms=9, mew=2, label="impact (accel std > 5)")
    for r in rows:
        gxg, gyg = float(r["goal_x"]), float(r["goal_y"])
        i = int(np.clip(np.searchsorted(gw, float(r["epoch_end"])), 0, len(gw) - 1))
        d = math.hypot(gx[i] - gxg, gy[i] - gyg)
        col = "#2a9d3b" if (r["result"] == "SUCCEEDED" and d <= ARRIVE_M) else ("#d62828" if r["result"] == "SUCCEEDED" else "#888888")
        ax.plot(gxg, gyg, "o", color=col, ms=10, mec="k")
        ax.annotate(f'{r["mission"]} {r["result"][:4]} true {d:.1f} m', (gxg, gyg), xytext=(6, 6),
                    textcoords="offset points", fontsize=8)
        cx, cy = float(r.get("claim_x", "nan")), float(r.get("claim_y", "nan"))
        if not math.isnan(cx):
            ax.plot([cx, gx[i]], [cy, gy[i]], "-", color="#d62828", lw=0.8)
            ax.plot(cx, cy, "s", color="#d62828", ms=5)
    ax.plot(0, 0, "o", color="#1f77b4", ms=6)
    ax.set_title(f"tour {base} on the frozen map of world {seed}\n"
                 f"goals: green = real arrival, red = false arrival, grey = not claimed; red squares = belief at claim")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
