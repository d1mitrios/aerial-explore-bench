#!/usr/bin/env python3
"""Plot one exploration run (missions/aerial_explore_runner.py): the final frozen map with
the true world geometry drawn over it (the map-frame offset is visible at a glance), the
ground-truth track, the belief the runner consumed, the mapper's corrected pose, the raw
odometry dead reckoning (rf2o since 2026-09-24, the VIO before), the frontier goals with their
result, the impacts; a second panel with
coverage vs policy time (the checkpoints marked) and the belief / mapper-pose error against
the truth. The picture the numbers of the run manifest describe.

  python3 analysis/plot_explore.py --run runs/raw/explore_<seed>_<ts>
      [--gt <a7_gt csv>] [--budget 10] [--out png]

The ground-truth file is paired by wall-clock span (the run's policy clock must fall inside
the GT file's span), as verify_missions.py does for the tours.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import matplotlib.transforms as mtransforms

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "missions"))
from gridmap import GridMap  # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_csv(path):
    with open(path) as f:
        return [r for r in csv.DictReader(l for l in f if not l.startswith("#"))]


def pick_gt(run_dir, seed, t0, t1):
    """The GT file covering the run: in the run directory itself (batch layout: the Isaac
    side writes into the run directory) or next to it (manual runs: runs/raw/)."""
    run_dir = run_dir.rstrip("/")
    cands = glob.glob(os.path.join(run_dir, f"*_gt_{seed}_*.csv")) + \
        glob.glob(os.path.join(os.path.dirname(run_dir), f"*_gt_{seed}_*.csv"))
    for c in sorted(cands, reverse=True):
        w = np.genfromtxt(c, delimiter=",", skip_header=2, usecols=(1,))
        if len(w) and w[0] - 60 <= t0 and t1 <= w[-1] + 60:
            return c
    raise SystemExit("no ground-truth CSV covers this run; pass --gt")


def draw_world(ax, seed):
    """The manifest's geometry (boxes, cylinders, the arena walls, the doors) in the truth frame."""
    path = os.path.join(ROOT, "worlds", "manifests", f"world_{seed}.csv")
    if not os.path.isfile(path):
        return
    for (x, y, w, h) in ((0, 10, 20, 0.5), (0, -10, 20, 0.5), (10, 0, 0.5, 20), (-10, 0, 0.5, 20)):
        ax.add_patch(Rectangle((x - w / 2, y - h / 2), w, h, fill=False, ec="#2a9d3b", lw=1.0, alpha=0.9))
    with open(path) as f:
        for line in f:
            p = line.strip().split(",")
            if len(p) < 7 or p[0] in ("type",) or line.startswith("#"):
                continue
            typ, x, y = p[0], float(p[2]), float(p[3])
            if typ == "box":
                w, h, yaw = float(p[4]), float(p[5]), float(p[6])
                r = Rectangle((-w / 2, -h / 2), w, h, fill=False, ec="#2a9d3b", lw=1.0, alpha=0.9)
                r.set_transform(mtransforms.Affine2D().rotate_deg(yaw).translate(x, y) + ax.transData)
                ax.add_patch(r)
            elif typ == "cyl":
                ax.add_patch(Circle((x, y), float(p[4]), fill=False, ec="#2a9d3b", lw=1.0, alpha=0.9))
            elif typ == "door":
                ax.plot(x, y, "|" if p[5] == "y" else "_", color="#2a9d3b", ms=10, mew=1.5)
    ax.plot([], [], "-", color="#2a9d3b", lw=1.0, label="true geometry (manifest)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", required=True, help="runs/raw/explore_<seed>_<ts> directory")
    ap.add_argument("--gt", default=None)
    ap.add_argument("--budget", default=None, help="which map_b<budget> to draw (default: the last)")
    ap.add_argument("--out", default=None)
    a = ap.parse_args()
    run = a.run.rstrip("/")
    man = json.load(open(glob.glob(os.path.join(run, "manifest_*.json"))[0]))
    seed = str(man["seed"])
    base = f"{seed}_{man['run_id']}"                                # <seed>_<ts> (any directory name)
    fr = load_csv(glob.glob(os.path.join(run, "frontiers_*.csv"))[0]) if glob.glob(os.path.join(run, "frontiers_*.csv")) else []
    budgets = man.get("budgets_min", [10.0])
    b = float(a.budget) if a.budget else budgets[-1]
    map_yaml = os.path.join(run, f"map_b{b:g}.yaml")
    ev = {e["event"]: e for e in man["events"]}
    t0w = ev.get("policy_clock_start", man["events"][0])["wall"]
    t1w = ev.get("land", man["events"][-1])["wall"]
    gt_path = a.gt or pick_gt(run, seed, t0w, t1w)
    out = a.out or os.path.join(ROOT, "analysis", f"explore_{base}.png")

    gt = np.genfromtxt(gt_path, delimiter=",", skip_header=2)
    gw, gx, gy = gt[:, 1], gt[:, 2], gt[:, 3]
    sel = (gw >= t0w - 5) & (gw <= t1w + 5)

    fig = plt.figure(figsize=(17, 9))
    ax = fig.add_subplot(1, 2, 1)
    if os.path.isfile(map_yaml):
        gm = GridMap(map_yaml)
        img = np.full(gm.cls.shape, 0.5)
        img[gm.cls == 0] = 1.0
        img[gm.cls == 1] = 0.0
        ext = [gm.origin[0], gm.origin[0] + gm.w * gm.resolution, gm.origin[1], gm.origin[1] + gm.h * gm.resolution]
        ax.imshow(img, cmap="gray", origin="lower", extent=ext, vmin=0, vmax=1)
    draw_world(ax, seed)
    ax.plot(gx[sel], gy[sel], "-", color="#1f77b4", lw=1.8, label="ground truth")
    bel_path = os.path.join(run, f"belief_{base}.csv")
    if os.path.isfile(bel_path):
        bl = np.genfromtxt(bel_path, delimiter=",", skip_header=2)
        if bl.ndim == 2 and len(bl):
            ok = (bl[:, 1] >= t0w) & (bl[:, 1] <= t1w)
            ax.plot(bl[ok, 2], bl[ok, 3], "-", color="#d62828", lw=1.2, alpha=0.9, label="belief consumed by the runner")
    pose_path = os.path.join(run, f"amcl_{base}.csv")
    if os.path.isfile(pose_path):
        ps = np.genfromtxt(pose_path, delimiter=",", skip_header=2)
        if ps.ndim == 2 and len(ps):
            ok = (ps[:, 1] >= t0w) & (ps[:, 1] <= t1w)
            ax.plot(ps[ok, 2], ps[ok, 3], ".", color="#7b2cbf", ms=3, alpha=0.8, label="mapper pose (/pose)")
    vio_path = next((c for c in (os.path.join(run, f"odom_{base}.csv"), os.path.join(run, f"vio_{base}.csv"))
                     if os.path.isfile(c)), "")
    if os.path.isfile(vio_path):
        v = np.genfromtxt(vio_path, delimiter=",", skip_header=2)
        if v.ndim == 2 and len(v):
            ok = ~np.isnan(v[:, 6]) & (v[:, 1] >= t0w) & (v[:, 1] <= t1w)
            ax.plot(v[ok, 6], v[ok, 7], "-", color="#e0a400", lw=0.8, alpha=0.7, label="odometry dead reckoning (anchored)")
    imu_path = os.path.join(run, f"imu_excitation_{base}.csv")
    if os.path.isfile(imu_path):
        imu = np.genfromtxt(imu_path, delimiter=",", skip_header=2)
        if imu.ndim == 2:
            for row in imu[imu[:, 2] > 5.0]:
                i = int(np.clip(np.searchsorted(gw, row[1]), 0, len(gw) - 1))
                ax.plot(gx[i], gy[i], "x", color="#e0a400", ms=10, mew=2)
            ax.plot([], [], "x", color="#e0a400", ms=10, mew=2, label="impact (accel std > 5)")
    for r in fr:
        fx, fy = float(r["goal_x"]), float(r["goal_y"])
        col = "#2a9d3b" if r["result"] == "SUCCEEDED" else "#d62828"
        ax.plot(fx, fy, "o", color=col, ms=8, mec="k")
        ax.annotate(f'{r["frontier"]} {r["result"][:4]}', (fx, fy), xytext=(5, 5), textcoords="offset points", fontsize=8)
    ax.plot(0, 0, "o", color="#1f77b4", ms=6)
    ax.set_xlim(-10.5, 10.5); ax.set_ylim(-10.5, 10.5)
    ax.set_aspect("equal")
    s = man.get("summary", {})
    ax.set_title(f"exploration {base}, map frozen at b{b:g}: {s.get('reason')} after {s.get('t_sim')} sim-s, "
                 f"{s.get('frontiers')} frontiers, coverage {s.get('coverage_m2')} m²\n"
                 f"frontier goals: green = reached, red = failed; the map should sit on the green geometry")
    ax.legend(loc="upper right", fontsize=8)

    # right: coverage vs policy time + error vs truth
    ax2 = fig.add_subplot(2, 2, 2)
    t0s = ev.get("policy_clock_start", {}).get("t0_sim", 0.0)
    if fr:
        ts = [float(r["sim_end"]) - t0s for r in fr]
        cov = [float(r["coverage_m2"]) for r in fr]
        ax2.step([0] + ts, [ev.get("policy_clock_start", {}).get("coverage_m2", cov[0])] + cov, where="post", color="#1f77b4", lw=1.8)
    for c in man.get("checkpoints", []):
        ax2.axvline(c["t_sim"], color="#888", lw=0.8, ls="--")
        ax2.annotate(f'b{c["budget_min"]:g}: {c["coverage_m2"]:.0f} m²' + (" (final)" if c.get("final") else ""),
                     (c["t_sim"], c["coverage_m2"]), xytext=(3, -12), textcoords="offset points", fontsize=8)
    ax2.set_xlabel("policy time, sim s"); ax2.set_ylabel("known-free area, m²")
    ax2.set_title("coverage vs budget (checkpoints dashed)")
    ax2.grid(alpha=0.3)

    ax3 = fig.add_subplot(2, 2, 4)

    def err_series(arr, xi, yi):
        tt, ee = [], []
        for row in arr:
            if row[1] < t0w or row[1] > t1w:
                continue
            i = int(np.clip(np.searchsorted(gw, row[1]), 0, len(gw) - 1))
            tt.append(gt[i, 0] - gt[int(np.clip(np.searchsorted(gw, t0w), 0, len(gw) - 1)), 0])
            ee.append(math.hypot(row[xi] - gx[i], row[yi] - gy[i]))
        return tt, ee
    if os.path.isfile(bel_path) and bl.ndim == 2:
        tt, ee = err_series(bl, 2, 3)
        ax3.plot(tt, ee, "-", color="#d62828", lw=1.2, label="belief vs truth")
    if os.path.isfile(pose_path) and ps.ndim == 2:
        tt, ee = err_series(ps, 2, 3)
        ax3.plot(tt, ee, ".", color="#7b2cbf", ms=3, label="mapper pose vs truth")
    if os.path.isfile(vio_path) and v.ndim == 2:
        vv = v[~np.isnan(v[:, 6])]
        tt, ee = err_series(vv, 6, 7)
        ax3.plot(tt, ee, "-", color="#e0a400", lw=0.8, alpha=0.7, label="raw odometry vs truth")
    ax3.set_ylim(0, 3.0)
    ax3.set_xlabel("policy time, sim s (Isaac clock)"); ax3.set_ylabel("position error, m")
    ax3.set_title("localization error against the ground truth")
    ax3.grid(alpha=0.3); ax3.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
