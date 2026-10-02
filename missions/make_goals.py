#!/usr/bin/env python3
"""Deterministic goal sets for the frozen-map missions (identical for both embodiments).

Rule (every number is a parameter below):
  - 10 goals per world, drawn with a random generator seeded by the world seed;
  - the four rooms of the cross partition, ordered by floor area (largest first), are
    served round-robin: goal 1 -> room 1, goal 2 -> room 2, ..., goal 5 -> room 1, ...
    so the tour alternates rooms and the two largest rooms get 3 goals, the others 2;
  - a goal is accepted when it lies inside its room at least WALL_MARGIN from the
    partition and boundary walls, at least CLEARANCE from every obstacle of the
    ground-truth manifest (the wheeled baseline's rule), and at least SPACING from the
    spawn (0, 0) and from every earlier goal;
  - the tour order is the sampling order. No "home" goal is scored; the executive
    returns to the spawn after the tour on its own.

Output: missions/goals/goals_<seed>.csv  (mission,room,x,y) + an optional overview PNG.

  python3 missions/make_goals.py 20260723008 [--plot]
  python3 missions/make_goals.py --all            # the 10 benchmark worlds
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MANIFESTS = os.path.join(ROOT, "worlds", "manifests")
OUT_DIR = os.path.join(ROOT, "missions", "goals")
WORLDS = ["20260723001", "20260723002", "20260723003", "20260723004", "20260723005",
          "20260723008", "20260723013", "20260723016", "20260723018", "20260723023"]

N_GOALS = 10
WALL_MARGIN = 0.9       # metres from partition and boundary walls (baseline: adjust_goal)
CLEARANCE = 0.6         # metres from every obstacle surface (baseline: adjust_goal)
SPACING = 2.0           # metres between goals, and from the spawn
ARENA_INNER = 9.75      # inner face of the boundary walls
MAX_TRIES = 20000


def read_manifest(path):
    obs, pxw, pyw = [], None, None
    for r in csv.reader(open(path)):
        if not r or r[0].startswith("#") or r[0] == "type":
            continue
        t = r[0]
        if t == "door":
            if r[5] == "x":
                pxw = float(r[2])
            else:
                pyw = float(r[3])
        elif t == "box":
            obs.append(("box", r[1], float(r[2]), float(r[3]), float(r[4]), float(r[5]), float(r[6])))
        elif t == "cyl":
            obs.append(("cyl", r[1], float(r[2]), float(r[3]), float(r[4]), 0.0, 0.0))
    if pxw is None or pyw is None:
        raise SystemExit(f"{path}: no cross partition (door rows) found")
    return obs, pxw, pyw


def clearance(x, y, obs):
    """Distance from (x, y) to the nearest obstacle surface (boxes exact for rotated rectangles)."""
    best = float("inf")
    for kind, _name, ox, oy, p1, p2, yaw in obs:
        if kind == "cyl":
            d = math.hypot(x - ox, y - oy) - p1
        else:
            c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
            u = (x - ox) * c + (y - oy) * s
            v = -(x - ox) * s + (y - oy) * c
            du, dv = max(abs(u) - p1 / 2, 0.0), max(abs(v) - p2 / 2, 0.0)
            d = math.hypot(du, dv) if (du > 0 or dv > 0) else -min(p1 / 2 - abs(u), p2 / 2 - abs(v))
        best = min(best, d)
    return best


def rooms_of(pxw, pyw):
    """The four rooms as (tag, xmin, xmax, ymin, ymax), largest area first."""
    xs = [(-ARENA_INNER, pxw, "W"), (pxw, ARENA_INNER, "E")]
    ys = [(-ARENA_INNER, pyw, "S"), (pyw, ARENA_INNER, "N")]
    rooms = []
    for x0, x1, tx in xs:
        for y0, y1, ty in ys:
            rooms.append((ty + tx, x0, x1, y0, y1, (x1 - x0) * (y1 - y0)))
    rooms.sort(key=lambda r: -r[5])
    return [r[:5] for r in rooms]


def sample_goals(seed, obs, pxw, pyw, n=N_GOALS):
    rng = np.random.default_rng(int(seed))
    rooms = rooms_of(pxw, pyw)
    goals = []
    quota = [0] * len(rooms)
    for i in range(n):
        quota[i % len(rooms)] += 1
    order = []
    for i in range(n):
        order.append(i % len(rooms))
    for ridx in order:
        placed = False
        for cand in [ridx] + [k for k in range(len(rooms)) if k != ridx]:
            tag, x0, x1, y0, y1 = rooms[cand]
            lo_x, hi_x = x0 + WALL_MARGIN, x1 - WALL_MARGIN
            lo_y, hi_y = y0 + WALL_MARGIN, y1 - WALL_MARGIN
            if hi_x <= lo_x or hi_y <= lo_y:
                continue
            for _ in range(MAX_TRIES):
                x = float(rng.uniform(lo_x, hi_x))
                y = float(rng.uniform(lo_y, hi_y))
                if math.hypot(x, y) < SPACING:
                    continue
                if any(math.hypot(x - gx, y - gy) < SPACING for _t, gx, gy in goals):
                    continue
                if clearance(x, y, obs) < CLEARANCE:
                    continue
                goals.append((tag, round(x, 2), round(y, 2)))
                placed = True
                break
            if placed:
                break
        if not placed:
            raise SystemExit(f"seed {seed}: could not place goal {len(goals) + 1}")
    return goals


def write_goals(seed, goals, out_dir=OUT_DIR):
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"goals_{seed}.csv")
    with open(path, "w", newline="") as f:
        f.write(f"# world {seed}: {len(goals)} goals, rooms round-robin by area, "
                f"wall_margin={WALL_MARGIN} clearance={CLEARANCE} spacing={SPACING}, "
                f"rng=default_rng({seed})\n")
        w = csv.writer(f)
        w.writerow(["mission", "room", "x", "y"])
        for i, (tag, x, y) in enumerate(goals, 1):
            w.writerow([f"g{i:02d}", tag, f"{x:.2f}", f"{y:.2f}"])
    return path


def plot_goals(seed, obs, pxw, pyw, goals, out_png):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Rectangle
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_xlim(-10.3, 10.3); ax.set_ylim(-10.3, 10.3); ax.set_aspect("equal")
    for kind, name, ox, oy, p1, p2, yaw in obs:
        if kind == "box":
            col = "#6b4c7a" if name.startswith("partition") else "#9a9a9a"
            rect = Rectangle((-p1 / 2, -p2 / 2), p1, p2, color=col)
            tr = matplotlib.transforms.Affine2D().rotate_deg(yaw).translate(ox, oy)
            rect.set_transform(tr + ax.transData)
            ax.add_patch(rect)
        else:
            ax.add_patch(Circle((ox, oy), p1, color="#9a9a9a"))
    ax.add_patch(Rectangle((-ARENA_INNER, -ARENA_INNER), 2 * ARENA_INNER, 2 * ARENA_INNER, fill=False, lw=2))
    ax.add_patch(Circle((0, 0), 2.0, fill=False, ls="--", color="#1f77b4"))
    ax.plot(0, 0, "o", color="#1f77b4")
    for i, (tag, x, y) in enumerate(goals, 1):
        ax.plot(x, y, "o", color="#d62828", ms=7)
        ax.annotate(f"g{i:02d} {tag}", (x, y), xytext=(5, 5), textcoords="offset points", fontsize=8)
    ax.set_title(f"world {seed}: mission goals (round-robin rooms, clearance {CLEARANCE} m, spacing {SPACING} m)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seed", nargs="?")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--plot", action="store_true")
    a = ap.parse_args()
    seeds = WORLDS if a.all else ([a.seed] if a.seed else [])
    if not seeds:
        ap.error("give a seed or --all")
    for seed in seeds:
        obs, pxw, pyw = read_manifest(os.path.join(MANIFESTS, f"world_{seed}.csv"))
        goals = sample_goals(seed, obs, pxw, pyw)
        path = write_goals(seed, goals)
        per_room = {}
        for tag, _x, _y in goals:
            per_room[tag] = per_room.get(tag, 0) + 1
        print(f"{seed}: {len(goals)} goals {per_room} -> {os.path.relpath(path, ROOT)}")
        if a.plot:
            png = os.path.join(OUT_DIR, f"goals_{seed}.png")
            plot_goals(seed, obs, pxw, pyw, goals, png)
            print(f"   plot -> {os.path.relpath(png, ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
