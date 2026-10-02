"""The mission mocks' own judge: every claim of this run against the mock's true trajectory.

The benchmark counts a claim real within ARRIVE_M = 1.5 m of the goal (analysis/verify_missions.py);
on the mock's noise-free truth a working executive claims far closer, so the mocks hold every claim
to JUDGE_MAX_M (default 0.5 m) and fail the run otherwise.
"""
import csv
import math
import os

import numpy as np

JUDGE_MAX_M = float(os.environ.get("JUDGE_MAX_M", "0.5"))
EXIT_FAR_CLAIM = 6


def judge(csv_path, traj):
    """Print each SUCCEEDED row of csv_path (this run's missions CSV) against the truth in traj
    ((t_sim, x, y, ...) samples); return the number of claims beyond JUDGE_MAX_M."""
    if not csv_path or not os.path.isfile(csv_path):
        print(f"judge: no missions CSV for this run ({csv_path})")
        return 0
    if not traj:
        print("judge: no trajectory recorded")
        return 0
    T = np.array([p[:3] for p in traj])
    far = 0
    with open(csv_path, newline="") as f:
        for r in csv.DictReader(f):
            if r["result"] != "SUCCEEDED":
                continue
            i = int(np.argmin(np.abs(T[:, 0] - float(r["sim_end"]))))
            d = math.hypot(T[i, 1] - float(r["goal_x"]), T[i, 2] - float(r["goal_y"]))
            far += d > JUDGE_MAX_M
            print(f"claim {r['mission']}: truth {d:.2f} m from the goal")
    if far:
        print(f"judge: {far} claim(s) beyond {JUDGE_MAX_M} m of the goal -> exit {EXIT_FAR_CLAIM}")
    return far
