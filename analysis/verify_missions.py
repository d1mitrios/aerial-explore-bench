#!/usr/bin/env python3
"""Verify a mission tour against simulator ground truth (the baseline's verdict rule).

Joins the executive's missions CSV (runs/raw/missions_<seed>_<ts>.csv, one row per
attempt with wall-clock epochs) with the Isaac-side ground-truth CSV (runs/raw/<tag>_gt_
<seed>_<ts>.csv, 10 Hz, both clocks) by WALL time, and applies the verdict:

  real arrival  = result SUCCEEDED and true distance to the goal <= ARRIVE_M (1.5 m)
  false arrival = result SUCCEEDED and true distance > ARRIVE_M  (the stack's belief lied)

Both sides log wall time from their own clock (Windows for Isaac, WSL2 for the executive);
WSL2 normally tracks the host within milliseconds, and the script reports the offset it
can see (the executive's takeoff event vs the ground truth's altitude rise) so a drifted
clock is caught rather than silently mis-joined. Also prints, per claim, the belief-vs-
truth error: the estimator's drift at the moment of the claim, which is data.

  python3 analysis/verify_missions.py --missions runs/raw/missions_20260723008_<ts>.csv \\
      [--gt runs/raw/a7_gt_20260723008_<ts>.csv] [--manifest runs/raw/manifest_..json]
"""
import argparse
import csv
import glob
import json
import math
import os
import sys
import time

import numpy as np

ARRIVE_M = 1.5          # unchanged from the baseline
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def read_gt(path):
    rows = np.genfromtxt(path, delimiter=",", skip_header=2)
    if rows.ndim != 2 or rows.shape[1] < 5:
        raise SystemExit(f"{path}: unexpected ground-truth format")
    return rows          # sim_t, wall_t, x, y, z, qx, qy, qz, qw


def read_missions(path):
    return [r for r in csv.DictReader(open(path))]


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

    print(f"missions: {os.path.relpath(a.missions, ROOT)}")
    print(f"ground truth: {os.path.relpath(gt_path, ROOT)} ({len(gt)} rows, wall {wall[0]:.0f}..{wall[-1]:.0f})")

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
    real = fake = succ = 0
    for r in rows:
        t_end = float(r["epoch_end"])
        i = int(np.clip(np.searchsorted(wall, t_end), 0, len(wall) - 1))
        if i > 0 and abs(wall[i - 1] - t_end) < abs(wall[i] - t_end):
            i -= 1
        gx, gy = float(r["goal_x"]), float(r["goal_y"])
        tx, ty = x[i], y[i]
        tdist = math.hypot(tx - gx, ty - gy)
        cx, cy = float(r.get("claim_x", "nan")), float(r.get("claim_y", "nan"))
        berr = math.hypot(cx - tx, cy - ty) if not math.isnan(cx) else float("nan")
        verdict = ""
        if r["result"] == "SUCCEEDED":
            succ += 1
            if tdist <= ARRIVE_M:
                real += 1
                verdict = "REAL"
            else:
                fake += 1
                verdict = "FALSE ARRIVAL"
        print(f"{r['mission']:8s} {r['attempt']:>3s} {r['result']:10s} {cx:8.2f},{cy:7.2f} {tx:8.2f},{ty:7.2f} "
              f"{tdist:9.2f} {berr:10.2f}  {verdict}")
    goals = len({r["mission"] for r in rows})
    print(f"\n{goals} goals, {len(rows)} attempts: {succ} SUCCEEDED -> {real} real, {fake} false "
          f"(ARRIVE_M = {ARRIVE_M} m)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
