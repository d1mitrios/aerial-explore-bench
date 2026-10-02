#!/usr/bin/env python3
"""Verify a mission tour against simulator ground truth (the baseline's verdict rule).

Joins the executive's missions CSV (runs/raw/missions_<seed>_<ts>.csv, one row per
attempt with wall-clock epochs) with the Isaac-side ground-truth CSV (runs/raw/<tag>_gt_
<seed>_<ts>.csv, 10 Hz, both clocks) by WALL time, and applies the verdict:

  real arrival  = result SUCCEEDED and true distance to the goal <= ARRIVE_M (1.5 m)
  false arrival = result SUCCEEDED and true distance > ARRIVE_M  (the stack's belief lied)
  void claim    = result SUCCEEDED while the claim's own belief was beyond ARRIVE_M of the goal:
                  a success reported without the estimate behind it (Nav2 1.3.12 after a failed
                  goal transform), neither real nor false

Both sides log wall time from their own clock (Windows for Isaac, WSL2 for the executive), and
the join takes the two as they are (CLOCK_WARN_S below). The batch scripts measure Windows minus
WSL before every run (clock_offset.txt next to the missions CSV); the script prints it, warns when
it exceeds CLOCK_WARN_S, and also reports the lag it can see (the executive's takeoff event vs the
ground truth's altitude rise), so a drifted clock is caught rather than silently mis-joined. Also
prints, per claim, the belief-vs-truth error: the estimator's drift at the moment of the claim,
which is data.

  python3 analysis/verify_missions.py --missions runs/raw/missions_20260723008_<ts>.csv \\
      [--gt runs/raw/a7_gt_20260723008_<ts>.csv] [--manifest runs/raw/manifest_..json]
"""
import argparse
import csv
import glob
import json
import math
import os
import re
import sys
import time

import numpy as np

ARRIVE_M = 1.5          # unchanged from the baseline
# The joins of every script take the runner's epochs (WSL) and the ground truth's wall clock
# (Windows) as they are. The offsets measured before the published runs were about 0.1 s, below
# the measurement's own uncertainty (0.12-0.13 s) and one ground-truth sample (10 Hz), so no
# correction is applied; a run whose offset exceeds CLOCK_WARN_S is reported instead.
CLOCK_WARN_S = 0.5
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_gt(path):
    rows = np.genfromtxt(path, delimiter=",", skip_header=2)
    if rows.ndim != 2 or rows.shape[1] < 5:
        raise SystemExit(f"{path}: unexpected ground-truth format")
    return rows          # sim_t, wall_t, x, y, z, qx, qy, qz, qw


def read_missions(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def clock_offset(run_dir):
    """Windows minus WSL wall clock in s from the run's clock_offset.txt (0 without one): the check
    of the joins (CLOCK_WARN_S). An offset beyond 5 s is a failed measurement (PowerShell gave no
    answer: 0 - the WSL epoch) and is not used."""
    p = os.path.join(run_dir, "clock_offset.txt")
    if not os.path.isfile(p):
        return 0.0
    with open(p) as f:
        m = re.search(r"offset_windows_minus_wsl=([+-]?[0-9.]+)", f.read())
    off = float(m.group(1)) if m else 0.0
    if abs(off) > 5.0:
        print(f"WARNING: {p}: offset {off:+.1f} s is not a measurement - using 0", file=sys.stderr)
        off = 0.0
    return off


def belief_miss(r):
    """The claim's own belief to the goal (m), nan when the runner logged no belief."""
    try:
        return math.hypot(float(r.get("claim_x") or "nan") - float(r["goal_x"]),
                          float(r.get("claim_y") or "nan") - float(r["goal_y"]))
    except ValueError:
        return float("nan")


def nearest(wall, t):
    """Index of the ground-truth sample nearest to wall time t (both in the truth's clock)."""
    i = int(np.clip(np.searchsorted(wall, t), 0, len(wall) - 1))
    if i > 0 and abs(wall[i - 1] - t) < abs(wall[i] - t):
        i -= 1
    return i


def pick_gt(missions_path, gt_arg, rows):
    """The ground-truth CSV of the same flight: its wall-clock span must cover the tour."""
    if gt_arg:
        return gt_arg
    seed = os.path.basename(missions_path).split("_")[1]
    cands = sorted(glob.glob(os.path.join(os.path.dirname(missions_path), f"*_gt_{seed}_*.csv")))
    if not cands:
        raise SystemExit("no ground-truth CSV found next to the missions file; pass --gt")
    t_first = min(float(r["epoch_start"]) for r in rows)
    t_last = max(float(r["epoch_end"]) for r in rows)
    for c in reversed(cands):
        try:
            w = np.genfromtxt(c, delimiter=",", skip_header=2, usecols=(1,))
        except Exception:  # noqa: BLE001
            continue
        if len(w) and w[0] - 60 <= t_first and t_last <= w[-1] + 60:
            return c
    raise SystemExit(f"no ground-truth CSV covers the tour's wall-clock span "
                     f"({time.strftime('%H:%M:%S', time.localtime(t_first))}–"
                     f"{time.strftime('%H:%M:%S', time.localtime(t_last))}); this missions file "
                     f"belongs to another flight; pass --gt explicitly if you mean it")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--missions", required=True)
    ap.add_argument("--gt", default=None)
    ap.add_argument("--manifest", default=None)
    a = ap.parse_args()
    rows = read_missions(a.missions)
    if not rows:
        raise SystemExit("missions file has no attempts")
    gt_path = pick_gt(a.missions, a.gt, rows)
    gt = read_gt(gt_path)
    wall, x, y, z = gt[:, 1], gt[:, 2], gt[:, 3], gt[:, 4]
    manifest_path = a.manifest or a.missions.replace("missions_", "manifest_").replace(".csv", ".json")
    manifest = json.load(open(manifest_path)) if os.path.isfile(manifest_path) else None

    off = clock_offset(os.path.dirname(os.path.abspath(a.missions)))
    print(f"missions: {os.path.relpath(a.missions, ROOT)}")
    print(f"ground truth: {os.path.relpath(gt_path, ROOT)} ({len(gt)} rows, wall {wall[0]:.0f}..{wall[-1]:.0f})")
    print(f"clock offset (Windows minus WSL, clock_offset.txt): {off:+.3f} s"
          + ("" if abs(off) <= CLOCK_WARN_S else f"   <-- WARNING: beyond {CLOCK_WARN_S} s, the join is suspect"))

    # clock sanity: the takeoff command (executive clock) must precede the first altitude
    # rise seen in ground truth (Isaac clock) by a small, RTF-dependent lag (0-15 s wall)
    if manifest:
        ev = {e["event"]: e for e in manifest.get("events", [])}
        if "takeoff_cmd" in ev:
            up = np.nonzero(z > 0.2)[0]
            if len(up):
                lag = wall[up[0]] - ev["takeoff_cmd"]["wall"]
                flag = "" if -1.0 <= lag <= 15.0 else "   <-- WARNING: clocks differ, join is suspect"
                print(f"clock check: GT first z>0.2 m comes {lag:+.2f} s after the executive's takeoff "
                      f"command (expected 0-15 s wall, RTF-dependent){flag}")

    print(f"\n{'mission':8s} {'att':>3s} {'result':10s} {'claim(x,y)':>16s} {'truth(x,y)':>16s} "
          f"{'true_dist':>9s} {'belief_err':>10s}  verdict")
    real = fake = void = succ = 0
    for r in rows:
        i = nearest(wall, float(r["epoch_end"]))
        gx, gy = float(r["goal_x"]), float(r["goal_y"])
        tx, ty = x[i], y[i]
        tdist = math.hypot(tx - gx, ty - gy)
        cx, cy = float(r.get("claim_x", "nan")), float(r.get("claim_y", "nan"))
        berr = math.hypot(cx - tx, cy - ty) if not math.isnan(cx) else float("nan")
        verdict = ""
        if r["result"] == "SUCCEEDED":
            succ += 1
            if belief_miss(r) > ARRIVE_M:         # nan compares False: no belief logged, a normal claim
                void += 1
                verdict = "VOID (the belief itself was beyond ARRIVE_M of the goal)"
            elif tdist <= ARRIVE_M:
                real += 1
                verdict = "REAL"
            else:
                fake += 1
                verdict = "FALSE ARRIVAL"
        print(f"{r['mission']:8s} {r['attempt']:>3s} {r['result']:10s} {cx:8.2f},{cy:7.2f} {tx:8.2f},{ty:7.2f} "
              f"{tdist:9.2f} {berr:10.2f}  {verdict}")
    goals = len({r["mission"] for r in rows})
    print(f"\n{goals} goals, {len(rows)} attempts: {succ} SUCCEEDED -> {real} real, {fake} false"
          + (f", {void} void" if void else "") + f" (ARRIVE_M = {ARRIVE_M} m)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
