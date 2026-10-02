#!/usr/bin/env python3
"""Where each robot's collisions happened on the shared executive: against geometry the frozen
map contains or against geometry it does not, and with the localizer right or lost.

  python3 analysis/collision_sources.py [--aerial runs/raw/batch] [--wheeled runs/raw/wbatch3]
                                        [--goals 6] [--out analysis/results]

The two robots' collision counts (goal_outcomes.py, summarize_arms.py) come from different
instruments and stay apart here too: the quadrotor's are the executive's IMU impacts, counted up
to the end of goal N's last attempt as summarize_arms.py counts them; the wheeled robot's are the
episodes of its true body (box + wheels) within CONTACT_M of the manifest's geometry, from the
tour's first attempt to goal N's last, merged as goal_outcomes.py merges them. Each event is
placed at the robot's true position (the start of a wheeled episode) and gets:
  object        the nearest piece of the manifest: outer wall, partition (the walls between the
                rooms, whose ends are the door frames) or furniture
  on_map        yes when most of the true surface within NEAR_M of the robot lies within
                DILATE_M of an occupied cell of the frozen map the tour used, no when it does not,
                none when no surface lies within NEAR_M (the impact detector fires up to 2 s late)
  belief_err_m  the executive's belief against the truth at that moment (above 1 m: lost)
  goal, attempt_end, goal_outcome
                the attempt in progress, how it ended, and the goal's class (goal_outcomes.py)
and, for the wheeled robot, the episode's length and the motion and body part at its start:
moved_3s_m and turned_3s_deg over the 3 sim-s before it, body_part (corner, wheel, side, or
front/back of the chassis) the point of the outline nearest the geometry.

Output collision_sources.csv in --out (one row per event); the summary below it is printed.
Needs runs/raw/ (not part of the repository); without batch data nothing is written.
"""
import argparse
import csv
import json
import math
import os
import sys
from collections import Counter

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import compare_footprint as cf  # noqa: E402
import goal_outcomes as go  # noqa: E402
import summarize_arms as sa  # noqa: E402

NEAR_M = 0.6          # true surface searched around the robot's position
SURFACE_M = 0.10      # depth of the surface layer, m
DILATE_M = 0.10       # an occupied map cell this close to a surface point counts as mapping it
LOST_M = 1.0          # belief error above which the localizer counts as lost
PRE_S = 3.0           # motion window before a wheeled episode, sim-s


def manifest_objects(seed):
    """[(kind, rect or cyl)] of the world: outer wall, partition, furniture."""
    objs = [("outer wall", ("rect", r)) for r in cf.ARENA]
    for ln in open(os.path.join(ROOT, "worlds", "manifests", f"world_{seed}.csv")):
        p = ln.strip().split(",")
        if len(p) < 7 or p[0] not in ("box", "cyl"):
            continue
        kind = "partition" if p[1].startswith("partition") else "furniture"
        if p[0] == "box":
            objs.append((kind, ("rect", (float(p[2]), float(p[3]), float(p[4]), float(p[5]),
                                         math.radians(float(p[6]))))))
        else:
            objs.append((kind, ("cyl", (float(p[2]), float(p[3]), float(p[4])))))
    return objs


def nearest_object(objs, px, py):
    best, kind = math.inf, ""
    for k, (shape, g) in objs:
        d = cf.clearance(np.array([px]), np.array([py]), [g] if shape == "rect" else [],
                         [g] if shape == "cyl" else [])[0]
        if d < best:
            best, kind = d, k
    return kind


def read_map(yaml_path):
    """(occupied mask dilated by DILATE_M, resolution, origin); row 0 is the lowest y."""
    meta = {}
    for ln in open(yaml_path):
        if ":" in ln:
            k, v = ln.split(":", 1)
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    org = [float(s) for s in meta["origin"].strip("[]").split(",")[:2]]
    with open(os.path.join(os.path.dirname(yaml_path), meta["image"]), "rb") as f:
        if f.readline().strip() != b"P5":
            raise ValueError(f"{yaml_path}: not a binary PGM")
        ln = f.readline()
        while ln.startswith(b"#"):
            ln = f.readline()
        w, h = map(int, ln.split())
        f.readline()
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)[::-1]
    occ = img < 100
    n = int(round(DILATE_M / res))
    for axis in (0, 1):
        grown = occ.copy()
        for s in range(1, n + 1):
            grown |= np.roll(occ, s, axis=axis) | np.roll(occ, -s, axis=axis)
        occ = grown
    return occ, res, org


def on_map(occ_map, geo, px, py):
    occ, res, org = occ_map
    g = np.arange(-NEAR_M, NEAR_M, res / 2)
    X, Y = np.meshgrid(px + g, py + g)
    X, Y = X.ravel(), Y.ravel()
    c = cf.clearance(X, Y, *geo)
    s = (c <= 0) & (c > -SURFACE_M) & (np.hypot(X - px, Y - py) <= NEAR_M)
    if not s.any():
        return "none"
    i = np.floor((X[s] - org[0]) / res).astype(int)
    j = np.floor((Y[s] - org[1]) / res).astype(int)
    ok = (i >= 0) & (j >= 0) & (i < occ.shape[1]) & (j < occ.shape[0])
    hit = np.zeros(len(i), bool)
    hit[ok] = occ[j[ok], i[ok]]
    return "yes" if hit.mean() >= 0.5 else "no"


def read_truth(run_dir):
    """(sim t, wall, x, y, yaw) of the robot's true pose, either vehicle's ground-truth file."""
    f = sa.newest(os.path.join(run_dir, "a7_gt_*.csv")) or sa.newest(os.path.join(run_dir, "wheeled_gt_*.csv"))
    if not f:
        return None
    a = np.genfromtxt(f, delimiter=",", skip_header=2, ndmin=2)
    qx, qy, qz, qw = a[:, 5], a[:, 6], a[:, 7], a[:, 8]
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return a[:, 0], a[:, 1], a[:, 2], a[:, 3], yaw


def read_belief(run_dir):
    f = sa.newest(os.path.join(run_dir, "belief_*.csv"))
    if not f:
        return None
    b = np.genfromtxt(f, delimiter=",", skip_header=2, ndmin=2)
    return b if b.shape[0] and b.shape[1] >= 4 else None


def at(arr, value):
    """The index of the first sample at or after value, clipped to the array."""
    return int(np.clip(np.searchsorted(arr, value), 0, len(arr) - 1))


def body_part(seed_geo, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    wx = x + cf.OUTLINE[:, 0] * c - cf.OUTLINE[:, 1] * s
    wy = y + cf.OUTLINE[:, 0] * s + cf.OUTLINE[:, 1] * c
    px, py = cf.OUTLINE[int(np.argmin(cf.clearance(wx, wy, *seed_geo)))]
    if abs(py) > 0.152:
        return "wheel"
    if abs(abs(px) - 0.20) < 0.03 and abs(abs(py) - 0.15) < 0.03:
        return "corner"
    return "front/back" if abs(abs(px) - 0.20) < 0.01 else "side"


def episodes(t, d):
    """[(first, last)] sample indices of the contact episodes (goal_outcomes.py's merge)."""
    hit = np.where(d <= cf.CONTACT_M)[0]
    out = []
    for h in hit:
        if out and t[h] - t[out[-1][1]] <= go.MERGE_S:
            out[-1][1] = h
        else:
            out.append([h, h])
    return out


def tour_events(arm, seed, budget, run_dir, explore_dir, ids):
    """(event rows, metres driven over the scored window) of one tour."""
    _, rows = sa.tour_rows(run_dir, set(ids))
    truth = read_truth(run_dir)
    if not rows or truth is None or not explore_dir:
        return [], None
    t, wall, x, y, yaw = truth
    w0 = min(float(r["epoch_start"]) for r in rows)
    w1 = max(float(r["epoch_end"]) for r in rows)
    k = np.where((wall >= w0) & (wall <= w1))[0]
    metres = float(np.hypot(np.diff(x[k]), np.diff(y[k])).sum()) if len(k) > 1 else 0.0
    geo = cf.geometry(seed)
    objs = manifest_objects(seed)
    occ_map = read_map(os.path.join(explore_dir, f"map_b{budget:g}.yaml"))
    belief = read_belief(run_dir)
    outcomes = {r["goal"]: r["outcome"] for r in go.tour_outcomes(arm, seed, budget, run_dir, ids,
                                                                  geo if arm in go.WHEELED else None)[0]}
    starts = []                                          # (index into the truth, wall time, extra)
    if arm == "aerial":
        m = sa.newest(os.path.join(run_dir, "manifest_*.json"))
        for e in json.load(open(m)).get("events", []) if m else []:
            if e.get("event") == "impact" and float(e.get("wall", 0.0)) <= w1:
                starts.append((at(wall, float(e["wall"])), float(e["wall"]), {}))
    else:
        d = cf.body_clearance(x[k], y[k], yaw[k], *geo)
        dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.1
        for a, b in episodes(t[k], d):
            i, j = k[a], k[b]
            i0 = at(t, t[i] - PRE_S)
            starts.append((i, float(wall[i]), dict(
                duration_s=round(float(t[j] - t[i]) + dt, 1),
                moved_3s_m=round(float(np.hypot(np.diff(x[i0:i + 1]), np.diff(y[i0:i + 1])).sum()), 2),
                turned_3s_deg=round(math.degrees(float(np.abs(np.angle(np.exp(1j * np.diff(yaw[i0:i + 1])))).sum()))),
                body_part=body_part(geo, x[i], y[i], yaw[i]))))
    out = []
    for i, w, extra in starts:
        err = ""
        if belief is not None:
            bi = at(belief[:, 1], w)
            err = round(math.hypot(belief[bi, 2] - x[i], belief[bi, 3] - y[i]), 2)
        att = [r for r in rows if float(r["epoch_start"]) <= w <= float(r["epoch_end"])]
        a = att[-1] if att else None
        row = dict(arm=arm, world=seed, budget_min=budget, tour_try=os.path.basename(run_dir),
                   sim_t=round(float(t[i]), 1), x=round(float(x[i]), 2), y=round(float(y[i]), 2),
                   object=nearest_object(objs, x[i], y[i]), on_map=on_map(occ_map, geo, x[i], y[i]),
                   belief_err_m=err, goal=a["mission"] if a else "",
                   attempt_end=f"{a['result']} {a['reason'].strip()}" if a else "",
                   goal_outcome=outcomes.get(a["mission"], "") if a else "",
                   duration_s="", moved_3s_m="", turned_3s_deg="", body_part="")
        row.update(extra)
        out.append(row)
    return out, metres


def summary(rows, metres, budgets):
    for arm in ("aerial", "wheeled"):
        ra = [r for r in rows if r["arm"] == arm]
        if not ra and not any(k[0] == arm for k in metres):
            continue
        what = "IMU impacts" if arm == "aerial" else "contact episodes"
        print(f"\n{arm}: {what} per budget")
        print(f"  {'budget':>6s} {'events':>7s} {'on map':>7s} {'not on':>7s} {'none':>5s} {'lost':>5s} {'metres':>7s}")
        for b in budgets:
            rb = [r for r in ra if r["budget_min"] == b]
            c = Counter(r["on_map"] for r in rb)
            lost = sum(1 for r in rb if r["belief_err_m"] != "" and r["belief_err_m"] > LOST_M)
            print(f"  {b:6g} {len(rb):7d} {c['yes']:7d} {c['no']:7d} {c['none']:5d} {lost:5d} "
                  f"{metres.get((arm, b), 0.0):7.0f}")
        if arm != "wheeled":
            continue
        m = [r for r in ra if r["on_map"] == "yes" and r["belief_err_m"] != "" and r["belief_err_m"] <= LOST_M]
        if not m:
            continue
        errs = sorted(r["belief_err_m"] for r in m)
        goals = {(r["world"], r["budget_min"], r["goal"]): r["goal_outcome"] for r in m if r["goal"]}
        print(f"  on the map with the localizer within {LOST_M:g} m: {len(m)} episodes "
              f"(per budget {', '.join(str(sum(1 for r in m if r['budget_min'] == b)) for b in budgets)}), "
              f"median belief error {errs[len(errs) // 2]:.2f} m; "
              f"objects {dict(Counter(r['object'] for r in m))}; "
              f"driving (>= 0.3 m in the {PRE_S:g} s before) {sum(1 for r in m if r['moved_3s_m'] >= 0.3)}; "
              f"body parts {dict(Counter(r['body_part'] for r in m))}; "
              f"during {len(goals)} goals, {sum(1 for v in goals.values() if v != 'real')} of them not reached")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aerial", default=os.path.join(ROOT, "runs", "raw", "batch"))
    ap.add_argument("--wheeled", default=os.path.join(ROOT, "runs", "raw", "wbatch3"),
                    help="the wheeled arm on the shared A* executive")
    ap.add_argument("--budgets", default="1,2.5,5,10")
    ap.add_argument("--goals", type=int, default=6, help="each tour's first N goals (default 6; 0 = all)")
    ap.add_argument("--out", default=os.path.join(ROOT, "analysis", "results"))
    a = ap.parse_args()
    budgets = [float(b) for b in a.budgets.split(",")]
    rows, metres = [], {}
    for arm, d in (("aerial", a.aerial), ("wheeled", a.wheeled)):
        if not d or not os.path.isdir(d):
            print(f"{arm}: no batch directory {d}")
            continue
        for seed in sa.WORLDS:
            ids = sa.goal_order(seed)
            ids = ids[:a.goals] if a.goals else ids
            ex = sa.final_try(os.path.join(d, seed, "explore"))
            for b in budgets:
                td = sa.final_try(os.path.join(d, seed, f"tour_b{b:g}"))
                if not td:
                    print(f"{arm}/{seed}/b{b:g}: no FINAL tour try")
                    continue
                ev, m = tour_events(arm, seed, b, td, ex, ids)
                rows += ev
                if m is not None:
                    metres[(arm, b)] = metres.get((arm, b), 0.0) + m
    if not metres:                                 # no run data: leave the committed tables alone
        print("no batch data found (runs/raw/ is not part of the repository); nothing written")
        return 1
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, "collision_sources.csv")
    fields = ["arm", "world", "budget_min", "tour_try", "sim_t", "x", "y", "object", "on_map", "belief_err_m",
              "goal", "attempt_end", "goal_outcome", "duration_s", "moved_3s_m", "turned_3s_deg", "body_part"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)
    summary(rows, metres, budgets)
    print(f"\n-> {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
