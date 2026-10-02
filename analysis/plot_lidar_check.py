#!/usr/bin/env python3
"""Offline geometry check of the quadrotor's raycast lidar against the world manifest.

Reads a scan-sample CSV written by sim/lidar_raycast.py (every k-th scan with its TRUE
origin pose, out-of-band) and the world's ground-truth manifest, projects every hit into
the world frame, and measures how far each hit lies from the nearest obstacle surface
(walls, partitions, furniture) rasterized at 0.05 m. A correct port puts (nearly) every
hit within one cell of a surface; phantom hits (self-hits, tilt into the floor) and
escapes (rays leaving over the 2.0 m walls) show up as outliers and misses.

Usage:
  python3 analysis/plot_lidar_check.py --scans runs/raw/a7_scans_20260723001_<ts>.csv
      [--manifest worlds/manifests/world_20260723001.csv] [--out analysis/lidar_check_<seed>.png]
      [--clip 12.0]

The manifest defaults to worlds/manifests/world_<seed>.csv with the seed read from the CSV
header. Hits beyond --clip (the SLAM front end's max range, 12 m) are ignored, misses
(range == rmax) are counted. Assumes the stabilized (horizontal) scan plane, or level flight.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Rectangle

RES = 0.05
HALF = 10.5                      # raster covers [-10.5, 10.5]^2 (walls included)
WALL_INNER = 9.75                # boundary boxes: centers at +-10, thickness 0.5
WALL_OUTER = 10.25


def read_manifest(path):
    obs = []
    for r in csv.reader(open(path)):
        if not r or r[0].startswith("#") or r[0] == "type":
            continue
        t = r[0]
        if t == "box":
            obs.append(dict(kind="box", name=r[1], x=float(r[2]), y=float(r[3]),
                            sx=float(r[4]), sy=float(r[5]), yaw=float(r[6])))
        elif t == "cyl":
            obs.append(dict(kind="cyl", name=r[1], x=float(r[2]), y=float(r[3]), r=float(r[4])))
    return obs


def rasterize(obs):
    n = int(round(2 * HALF / RES))
    xs = -HALF + RES * (np.arange(n) + 0.5)
    X, Y = np.meshgrid(xs, xs)                       # X varies along columns, Y along rows
    occ = (np.abs(X) >= WALL_INNER) & (np.abs(X) <= WALL_OUTER) & (np.abs(Y) <= WALL_OUTER)
    occ |= (np.abs(Y) >= WALL_INNER) & (np.abs(Y) <= WALL_OUTER) & (np.abs(X) <= WALL_OUTER)
    for o in obs:
        if o["kind"] == "box":
            c, s = math.cos(math.radians(o["yaw"])), math.sin(math.radians(o["yaw"]))
            u = (X - o["x"]) * c + (Y - o["y"]) * s
            v = -(X - o["x"]) * s + (Y - o["y"]) * c
            occ |= (np.abs(u) <= o["sx"] / 2) & (np.abs(v) <= o["sy"] / 2)
        else:
            occ |= (X - o["x"]) ** 2 + (Y - o["y"]) ** 2 <= o["r"] ** 2
    return occ, X, Y


def read_scans(path):
    hdr = open(path).readline().strip("# \n")
    meta = dict(kv.split("=", 1) for kv in hdr.split() if "=" in kv)
    n = int(meta["n"])
    a_min, a_inc = float(meta["angle_min"]), float(meta["angle_inc"])
    rmax = float(meta["rmax"])
    data = np.genfromtxt(path, delimiter=",", skip_header=2)
    if data.ndim == 1:
        data = data[None, :]
    return meta, n, a_min, a_inc, rmax, data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scans", required=True)
    ap.add_argument("--manifest", default=None)
    ap.add_argument("--out", default=None)
    ap.add_argument("--clip", type=float, default=12.0)
    a = ap.parse_args()
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    meta, n, a_min, a_inc, rmax, data = read_scans(a.scans)
    seed = meta.get("seed", "unknown")
    manifest = a.manifest or os.path.join(root, "worlds", "manifests", f"world_{seed}.csv")
    out = a.out or os.path.join(root, "analysis", f"lidar_check_{seed}.png")
    obs = read_manifest(manifest)
    occ, X, Y = rasterize(obs)
    occ_pts = np.stack([X[occ], Y[occ]], axis=1)

    angles = a_min + a_inc * np.arange(n)
    pts, origins, misses, total = [], [], 0, 0
    for row in data:
        sim_t, ox, oy, oz, yaw_deg = row[:5]
        ranges = row[5:5 + n]
        origins.append((ox, oy))
        total += n
        misses += int(np.sum(ranges >= rmax - 1e-6))
        keep = (ranges < a.clip) & (ranges > 0)
        th = math.radians(yaw_deg) + angles[keep]
        pts.append(np.stack([ox + ranges[keep] * np.cos(th), oy + ranges[keep] * np.sin(th)], axis=1))
    pts = np.concatenate(pts) if pts else np.zeros((0, 2))
    origins = np.array(origins)

    # exact nearest-occupied-cell distance, chunked (surface ~ cell center - half a cell)
    d = np.empty(len(pts))
    for i in range(0, len(pts), 400):
        blk = pts[i:i + 400]
        dd = np.sqrt(((blk[:, None, :] - occ_pts[None, :, :]) ** 2).sum(axis=2))
        d[i:i + 400] = np.maximum(dd.min(axis=1) - RES / 2, 0.0)

    def frac(t):
        return float(np.mean(d <= t)) if len(d) else float("nan")
    summary = (f"seed {seed}: {len(data)} sampled scans, {total} rays, {len(pts)} hits <{a.clip:g} m, "
               f"{misses} misses (range=rmax)\n"
               f"hit-to-surface distance: median {np.median(d):.3f} m, mean {np.mean(d):.3f} m, "
               f"p90 {np.percentile(d, 90):.3f} m, p95 {np.percentile(d, 95):.3f} m, max {d.max():.3f} m\n"
               f"within 0.05 m: {100 * frac(0.05):.1f}%  within 0.10 m: {100 * frac(0.10):.1f}%  "
               f"within 0.20 m: {100 * frac(0.20):.1f}%  beyond 0.30 m: {int(np.sum(d > 0.30))} hits")
    print(summary)

    fig, ax = plt.subplots(figsize=(10, 10))
    ax.set_xlim(-HALF, HALF); ax.set_ylim(-HALF, HALF); ax.set_aspect("equal")
    for o in obs:
        if o["kind"] == "box":
            col = "#6b4c7a" if o["name"].startswith("partition") else "#b0b0b0"
            rect = Rectangle((-o["sx"] / 2, -o["sy"] / 2), o["sx"], o["sy"], color=col, alpha=0.6)
            tr = matplotlib.transforms.Affine2D().rotate_deg(o["yaw"]).translate(o["x"], o["y"])
            rect.set_transform(tr + ax.transData)
            ax.add_patch(rect)
        else:
            ax.add_patch(Circle((o["x"], o["y"]), o["r"], color="#b0b0b0", alpha=0.6))
    for sgn in (-1, 1):
        ax.add_patch(Rectangle((-WALL_OUTER, sgn * WALL_INNER if sgn > 0 else -WALL_OUTER),
                               2 * WALL_OUTER, 0.5, color="#555555", alpha=0.6))
        ax.add_patch(Rectangle((sgn * WALL_INNER if sgn > 0 else -WALL_OUTER, -WALL_OUTER),
                               0.5, 2 * WALL_OUTER, color="#555555", alpha=0.6))
    good, mid, bad = d <= 0.10, (d > 0.10) & (d <= 0.25), d > 0.25
    ax.scatter(pts[good, 0], pts[good, 1], s=3, c="#2a9d3b", label=f"hit within 0.10 m ({good.sum()})")
    ax.scatter(pts[mid, 0], pts[mid, 1], s=6, c="#e0a400", label=f"0.10-0.25 m ({mid.sum()})")
    ax.scatter(pts[bad, 0], pts[bad, 1], s=10, c="#d62828", label=f"beyond 0.25 m ({bad.sum()})")
    ax.plot(origins[:, 0], origins[:, 1], "o", color="#1f77b4", ms=4, label="scan origins (true pose)")
    ax.legend(loc="upper right", fontsize=8)
    ax.set_title(f"raycast lidar vs manifest, world {seed}\n" + summary.replace("\n", "\n"), fontsize=8)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f"wrote {out}")
    return 0 if frac(0.10) >= 0.95 and misses == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
