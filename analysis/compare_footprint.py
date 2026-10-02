#!/usr/bin/env python3
"""Footprint sensitivity: the wheeled baseline with the circular body model of v9 (the predecessor's
Nav2 configuration) against the same stack with the robot's true outline, on the worlds of the
sensitivity batch.

  python3 analysis/compare_footprint.py [--base runs/raw/wbatch2] [--variant runs/raw/wbatch_fp_full]
                                        [--worlds 20260723002,...] [--goals 6] [--out analysis/results]

The variant batches are runs/raw/wbatch_fp_full (the default: full tours on 002, 004 and 005) and
runs/raw/wbatch_fp_test (the shorter first test); --variant "" with --worlds measures the base
batch alone.

v9 models the body as two circles - the local costmap's robot_radius 0.22 (0.23 with Nav2's default
footprint_padding) for DWB and the behaviours, the collision monitor's CircleStop 0.23 m - while
the body is a 0.40 x 0.30 m box with the wheels out to |y| 0.195 m (corners 0.25 m from base_link).
The variant (policies/nav2/nav2_params_polygon.yaml) replaces both by the true outline + 0.02 m.
For every world and phase, from the FINAL try only, per configuration:

  coverage_m2   the exploration's frozen map at the budget (the exploration moves on Nav2 too)
  real / false  the tour's first N goals by verify_missions.py's rule (as summarize_arms.py);
  void          claims the stack's own belief contradicts, neither real nor false
  still_max_s   the longest time the robot stayed within 0.10 m of one point (sim-s)
  zone_stop_s   sim-seconds the collision monitor held the robot for its stop zone: from a
                "Robot to stop due to <zone> polygon" line of the Nav2 log to the next "continue
                normal operation" line, in the log's line order (the state lines' stamps are not
                monotonic); "invalid source" stops inside such an episode belong to it - the robot
                stays stopped - and outside one they are counted apart (src_stops, lines: a TF
                lookup that failed on the two clocks, or a late scan)
  goals_frozen  goals not reached whose every attempt ended inside a freeze - a still stretch
                >= 60 sim-s the stop zone held for most of its length - whatever the result code
                (a frozen robot's attempts time out, or the planner refuses to plan from inside its
                own inflation: 208); a goal refused as outside the map (204) would have failed
                anywhere and is not counted
  contact_s     sim-seconds with the true body (box + wheels) within 0.01 m of the manifest's
                geometry, from the ground-truth pose; min_clear_m its minimum
  body          the body model the run's own params copy and Nav2 log show: 'circle' or 'outline',
                '!' when the log shows it did not load as written (an invalid footprint string
                falls back to the radius; a polygon whose points did not parse is left empty)

A tour's columns cover its first N goals' attempts only (from the first one's start to the last
one's end), so a tour cut with --goals compares like for like with a shorter replay.

Output: a table on stdout and <out>/footprint_compare.csv. A phase without a FINAL try is
listed as missing, never guessed.
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import summarize_arms as sa  # noqa: E402

FREEZE_MIN_S = 60.0       # sim-s within STILL_M of one point, held by the stop zone
STILL_M = 0.10
CONTACT_M = 0.01          # true body to the manifest's geometry
# the body about base_link (the lidar at the chassis centre; sim/wheeled/payloads/base.usda):
# the chassis box and the two wheels as the rectangles (x0, x1, y0, y1) they cover on the floor
BODY = ((-0.20, 0.20, -0.15, 0.15), (-0.15, -0.05, 0.155, 0.195), (-0.15, -0.05, -0.195, -0.155))
ARENA = ((0, 10, 20, 0.5, 0.0), (0, -10, 20, 0.5, 0.0), (10, 0, 0.5, 20, 0.0), (-10, 0, 0.5, 20, 0.0))


def geometry(seed):
    """Boxes (x, y, w, h, yaw rad) incl. the arena walls, and cylinders (x, y, r), from the manifest."""
    rects, cyls = list(ARENA), []
    for ln in open(os.path.join(ROOT, "worlds", "manifests", f"world_{seed}.csv")):
        p = ln.strip().split(",")
        if len(p) < 7 or p[0] not in ("box", "cyl"):
            continue
        if p[0] == "box":
            rects.append((float(p[2]), float(p[3]), float(p[4]), float(p[5]), math.radians(float(p[6]))))
        else:
            cyls.append((float(p[2]), float(p[3]), float(p[4])))
    return rects, cyls


def clearance(px, py, rects, cyls):
    """Signed distance of points to the nearest obstacle surface (negative inside)."""
    d = np.full(np.shape(px), np.inf)
    for (x, y, w, h, yaw) in rects:
        c, s = math.cos(-yaw), math.sin(-yaw)
        lx = (px - x) * c - (py - y) * s
        ly = (px - x) * s + (py - y) * c
        dx, dy = np.abs(lx) - w / 2, np.abs(ly) - h / 2
        d = np.minimum(d, np.hypot(np.maximum(dx, 0), np.maximum(dy, 0)) + np.minimum(np.maximum(dx, dy), 0))
    for (x, y, r) in cyls:
        d = np.minimum(d, np.hypot(px - x, py - y) - r)
    return d


def body_outline(step=0.005):
    pts = []
    for (x0, x1, y0, y1) in BODY:
        for x in np.arange(x0, x1 + 1e-9, step):
            pts += [(x, y0), (x, y1)]
        for y in np.arange(y0, y1 + 1e-9, step):
            pts += [(x0, y), (x1, y)]
    return np.array(pts)


OUTLINE = body_outline()


def read_gt(run_dir):
    g = sa.newest(os.path.join(run_dir, "wheeled_gt_*.csv"))
    if not g:
        return None
    a = np.genfromtxt(g, delimiter=",", skip_header=2)
    t, wall, x, y = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
    qx, qy, qz, qw = a[:, 5], a[:, 6], a[:, 7], a[:, 8]
    yaw = np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return t, wall, x, y, yaw


def still_stretches(t, x, y):
    """[(t_start, t_end)] of the stretches with the robot within STILL_M of their start."""
    out, i, n = [], 0, len(t)
    while i < n:
        j = i
        while j + 1 < n and math.hypot(x[j + 1] - x[i], y[j + 1] - y[i]) < STILL_M:
            j += 1
        out.append((float(t[i]), float(t[j])))
        i = j + 1
    return out


def body_clearance(x, y, yaw, rects, cyls):
    """The true body's (box + wheels) clearance to the geometry at every pose (m; inf beyond
    0.05 m of reach - only poses whose centre is within 0.30 m are resolved)."""
    cc = clearance(x, y, rects, cyls)
    near = np.where(cc < 0.30)[0]                     # the body reaches 0.25 m from base_link
    dmin = np.full(len(x), np.inf)
    for k in range(0, len(near), 400):
        idx = near[k:k + 400]
        c, s = np.cos(yaw[idx])[:, None], np.sin(yaw[idx])[:, None]
        wx = x[idx][:, None] + OUTLINE[:, 0] * c - OUTLINE[:, 1] * s
        wy = y[idx][:, None] + OUTLINE[:, 0] * s + OUTLINE[:, 1] * c
        dmin[idx] = clearance(wx, wy, rects, cyls).min(axis=1)
    return dmin


def contact(t, x, y, yaw, rects, cyls):
    """(contact sim-s, min clearance m) of the true body against the geometry."""
    dmin = body_clearance(x, y, yaw, rects, cyls)
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.1
    return float((dmin <= CONTACT_M).sum() * dt), (float(dmin.min()) if len(t) else float("nan"))


def body_model(run_dir):
    """('circle' | 'outline' | '?', ok, zone names) from the run's params copy and its Nav2 log."""
    p = sa.newest(os.path.join(run_dir, "nav2_params_*.yaml"))
    log = sa.newest(os.path.join(run_dir, "wnav_*.log"))
    if not p:
        return "?", False, []
    y = yaml.safe_load(open(p))
    lc = y["local_costmap"]["local_costmap"]["ros__parameters"]
    cm = y["collision_monitor"]["ros__parameters"]
    zones = list(cm.get("polygons", []))
    kind = "outline" if str(lc.get("footprint", "[]")).strip() not in ("", "[]") else "circle"
    ok = True
    if log:
        txt = open(log, errors="replace").read()
        if "The footprint parameter is invalid" in txt:
            ok = False
        for z in zones:
            if f"[{z}]: Creating Polygon" not in txt or re.search(rf"\[{z}\][^\n]*(ubscri|rror|nvalid)", txt):
                ok = False
    else:
        ok = False
    return kind, ok, zones


STATE_RE = re.compile(r"\[(\d+\.\d+)\] \[collision_monitor\]: Robot to (?:stop due to (invalid source|\S+) polygon"
                      r"|stop due to (invalid source)|(continue) normal operation)")


def clock_offset(run_dir):
    """Windows minus WSL wall clock (clock_offset.txt of the run; the Nav2 log is on WSL's). An offset
    beyond 5 s is a failed measurement (PowerShell gave no answer: 0 - the WSL epoch) and is not used."""
    p = os.path.join(run_dir, "clock_offset.txt")
    m = re.search(r"offset_windows_minus_wsl=([+-]?[0-9.]+)", open(p).read()) if os.path.isfile(p) else None
    off = float(m.group(1)) if m else 0.0
    if abs(off) > 5.0:
        print(f"WARNING: {p}: offset {off:+.1f} s is not a measurement - using 0", file=sys.stderr)
        off = 0.0
    return off


def stops(run_dir, zones, gt_wall, gt_sim):
    """([(sim_start, sim_end)] in the zone-stop state, [sim-s of each invalid-source stop line
    outside them]) from the collision monitor's log; without the ground truth (None, the count)."""
    log = sa.newest(os.path.join(run_dir, "wnav_*.log"))
    if not log:
        return None, ([] if gt_wall is not None else "")
    ev = []
    for m in STATE_RE.finditer(open(log, errors="replace").read()):
        what = m.group(2) or m.group(3) or m.group(4)
        ev.append((float(m.group(1)), "zone" if what in zones else "continue" if what == "continue" else "source"))
    off = clock_offset(run_dir) if gt_wall is not None else 0.0
    t_end = float(gt_wall[-1]) - off if gt_wall is not None else float("inf")
    spans, src, t0 = [], [], None         # in the log's line order: the stamps are not monotonic
    for t, st in ev:
        if st == "zone" and t0 is None:
            t0 = t
        elif st == "continue" and t0 is not None:
            spans.append((t0, max(t, t0)))
            t0 = None
        elif st == "source" and t0 is None:
            src.append(t)
    if t0 is not None:
        spans.append((t0, max(t_end, t0)))
    if gt_wall is None:
        return None, len(src)
    sim = lambda w: float(np.interp(w + off, gt_wall, gt_sim))  # noqa: E731
    return [(sim(a), sim(b)) for a, b in spans], [sim(w) for w in src]


def tour_window(run_dir, goal_ids, gt_wall, gt_sim):
    """(sim_start, sim_end) of the kept goals' attempts - a tour cut to its first N goals is
    measured over them only - or None."""
    mpath, rows = sa.tour_rows(run_dir, set(goal_ids))
    if not rows:
        return None
    off = clock_offset(run_dir)          # the runner stamps with WSL's clock, the truth Windows'
    w0 = min(float(r["epoch_start"]) for r in rows) + off
    w1 = max(float(r["epoch_end"]) for r in rows) + off
    return float(np.interp(w0, gt_wall, gt_sim)), float(np.interp(w1, gt_wall, gt_sim))


def overlap(a, b, spans):
    return sum(max(0.0, min(b, y) - max(a, x)) for x, y in spans)


def freezes(t, x, y, held):
    """[(sim_start, sim_end)] of the freezes: still stretches >= FREEZE_MIN_S the stop zone held
    for more than half their length."""
    return [(a, b) for a, b in still_stretches(t, x, y)
            if b - a >= FREEZE_MIN_S and overlap(a, b, held or []) > 0.5 * (b - a)]


def frozen_goal_ids(run_dir, goal_ids, frz):
    """The goals in goal_ids lost to a freeze: not reached, none refused as outside the map (204),
    every attempt ended inside a freeze (+5 s)."""
    m = sa.newest(os.path.join(run_dir, "missions_*.csv"))
    if not m or not frz:
        return set()
    att = {}
    for r in csv.DictReader(open(m)):
        if r["mission"] in goal_ids:
            att.setdefault(r["mission"], []).append(r)
    lost = set()
    for g, rows in att.items():
        if any(r["result"] == "SUCCEEDED" for r in rows) or any(r["reason"].strip() == "aborted 204" for r in rows):
            continue
        if all(any(a <= float(r["sim_end"]) <= b + 5.0 for a, b in frz) for r in rows):
            lost.add(g)
    return lost


def goals_frozen(run_dir, goal_ids, frz):
    return len(frozen_goal_ids(run_dir, goal_ids, frz))


def phase_row(cfg, seed, phase, run_dir, goal_ids, geo, cov=""):
    row = dict(config=cfg, world=seed, phase=phase, try_=os.path.basename(run_dir) if run_dir else "",
               coverage_m2=cov, real="", false="", void="", still_max_s="", zone_stop_s="", goals_frozen="",
               src_stops="", contact_s="", min_clear_m="", body="")
    if not run_dir:
        row["body"] = "missing"
        return row
    kind, ok, zones = body_model(run_dir)
    row["body"] = kind + ("" if ok else " !")
    g = read_gt(run_dir)
    frz = []
    if g is not None:
        t, wall, x, y, yaw = g
        held, src = stops(run_dir, zones, wall, t)
        win = tour_window(run_dir, goal_ids, wall, t) if phase.startswith("tour_") else None
        if win:                           # a tour: the kept goals' attempts only
            s0, s1 = win
            k = (t >= s0) & (t <= s1)
            t, x, y, yaw = t[k], x[k], y[k], yaw[k]
            held = None if held is None else [(max(a, s0), min(b, s1)) for a, b in held if b > s0 and a < s1]
            src = [s for s in src if s0 <= s <= s1]
        st = still_stretches(t, x, y)
        row["still_max_s"] = round(max(b - a for a, b in st)) if st else 0
        row["src_stops"] = len(src)
        row["zone_stop_s"] = round(sum(b - a for a, b in held)) if held is not None else ""
        frz = freezes(t, x, y, held)
        cs, dm = contact(t, x, y, yaw, *geo)
        row["contact_s"], row["min_clear_m"] = round(cs, 1), round(dm, 3)
    else:
        row["src_stops"] = stops(run_dir, zones, None, None)[1]
    if phase.startswith("tour_"):
        v = sa.tour_verdict(run_dir, set(goal_ids))
        if v:
            row["real"], row["false"], row["void"] = v["real"], v["false"], v["void"]
        row["goals_frozen"] = goals_frozen(run_dir, set(goal_ids), frz)
    return row


def collect(cfg, batch_dir, worlds, budgets, keep_goals):
    rows = []
    for seed in worlds:
        geo = geometry(seed)
        goal_ids = sa.goal_order(seed)[:keep_goals] if keep_goals else sa.goal_order(seed)
        wd = os.path.join(batch_dir, seed)
        ex = sa.final_try(os.path.join(wd, "explore"))
        cps = {}
        if ex:
            man = json.load(open(sa.newest(os.path.join(ex, "manifest_*.json"))))
            cps = {float(c["budget_min"]): round(c["coverage_m2"]) for c in man.get("checkpoints", [])}
        rows.append(phase_row(cfg, seed, "explore", ex, goal_ids, geo,
                              "/".join(str(cps.get(b, "-")) for b in budgets)))
        for b in budgets:
            td = sa.final_try(os.path.join(wd, f"tour_b{b:g}"))
            rows.append(phase_row(cfg, seed, f"tour_b{b:g}", td, goal_ids, geo, cps.get(b, "")))
    return rows


def summary(rows, cfg, n_goals):
    tours = [r for r in rows if r["config"] == cfg and r["phase"].startswith("tour_") and r["real"] != ""]
    runs = [r for r in rows if r["config"] == cfg and r["try_"]]
    if not tours:
        return f"{cfg}: no tours"
    real = sum(r["real"] for r in tours)
    false = sum(r["false"] for r in tours)
    void = sum(r["void"] for r in tours)
    n = len(tours) * n_goals - void
    frz = sum(1 for r in tours if r["goals_frozen"])
    lost = sum(r["goals_frozen"] or 0 for r in tours)
    held = sum(r["zone_stop_s"] or 0 for r in runs)
    cont = sum(1 for r in runs if r["contact_s"] not in ("", 0, 0.0))
    return (f"{cfg}: {len(tours)} tours, real {real}/{n} ({100.0 * real / n:.1f} %), false {false}"
            f"{f', void {void} (out of the count)' if void else ''}; "
            f"tours frozen {frz}, goals lost to freezes {lost}; held by the stop zone {held} sim-s in all; "
            f"runs with body contact {cont}/{len(runs)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--base", default=os.path.join(ROOT, "runs", "raw", "wbatch2"))
    ap.add_argument("--variant", default=os.path.join(ROOT, "runs", "raw", "wbatch_fp_full"))
    ap.add_argument("--worlds", default="", help="comma list (default: the worlds in --variant)")
    ap.add_argument("--budgets", default="1,2.5,5,10")
    ap.add_argument("--goals", type=int, default=6, help="keep each tour's first N goals (default 6; 0 = all)")
    ap.add_argument("--out", default=os.path.join(ROOT, "analysis", "results"))
    a = ap.parse_args()
    budgets = [float(b) for b in a.budgets.split(",")]
    worlds = [w for w in a.worlds.split(",") if w] or sorted(
        os.path.basename(d) for d in glob.glob(os.path.join(a.variant, "2026*")) if os.path.isdir(d))
    if not worlds:
        raise SystemExit(f"no worlds in {a.variant}; pass --worlds")
    configs = [("v9", a.base)] + ([("outline", a.variant)] if os.path.isdir(a.variant) else [])
    rows = []
    for cfg, d in configs:
        rows += collect(cfg, d, worlds, budgets, a.goals)
    cols = ["world", "phase", "config", "try_", "body", "coverage_m2", "real", "false", "void", "still_max_s",
            "zone_stop_s", "goals_frozen", "src_stops", "contact_s", "min_clear_m"]
    order = {c: i for i, (c, _) in enumerate(configs)}
    rows.sort(key=lambda r: (r["world"], r["phase"] != "explore",
                             float(r["phase"][6:]) if r["phase"].startswith("tour_b") else 0, order[r["config"]]))
    w = {c: max(len(c), *(len(str(r[c])) for r in rows)) for c in cols}
    print("  ".join(c.ljust(w[c]) for c in cols))
    for r in rows:
        print("  ".join(str(r[c]).ljust(w[c]) for c in cols))
    print()
    n = a.goals or len(sa.goal_order(worlds[0]))
    for cfg, _ in configs:
        print(summary(rows, cfg, n))
    expect = {"v9": "circle", "outline": "outline"}
    copied = {c: os.path.isfile(os.path.join(d, "COPIED_FROM.txt")) for c, d in configs}
    odd = [f"{r['world']} {r['phase']} ({r['config']}: {r['body']})" for r in rows
           if r["try_"] and r["body"] != expect[r["config"]]
           and not (copied[r["config"]] and r["phase"] == "explore" and r["body"].startswith("?"))]
    if any(copied.values()):
        print("(explorations copied into a batch from another, COPIED_FROM.txt, are that batch's: maps only)")
    if odd:
        print("WARNING - runs whose own params / Nav2 log show another body model, or one that did not load:")
        print("  " + "; ".join(odd))
    os.makedirs(a.out, exist_ok=True)
    out = os.path.join(a.out, "footprint_compare.csv")
    with open(out, "w", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=cols)
        wr.writeheader()
        for r in rows:
            wr.writerow({c: r[c] for c in cols})
    print(f"-> {os.path.relpath(out, ROOT)}")


if __name__ == "__main__":
    main()
