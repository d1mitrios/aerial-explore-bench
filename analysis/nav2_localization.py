#!/usr/bin/env python3
"""Where and why AMCL lost the wheeled robot on Nav2: the reference arm (the predecessor's revised
configuration) against the first configuration (v8), on the same frozen-map protocol.

  python3 analysis/nav2_localization.py [--revised runs/raw/wbatch_nav2rev] [--first runs/raw/wbatch2]
                                        [--goals 6] [--out analysis/results]

On Nav2 the belief is AMCL's pose as TF map->base_link (belief_*.csv, which also carries the wheel
odometry AMCL used), and nothing stands between it and Nav2's success signal. From the FINAL tries
and each tour's first --goals goals, the belief and the truth are sampled every DT_S over the scored
window (the first goal's first attempt to the last goal's last, summarize_arms.tour_window), and
every sample is placed by where the robot truly was on the tour's frozen map: on a known-free cell
or off one (loc_guard_compare.dist_to_known_free). Per arm:

  lost_tours            tours in which the belief was more than LOST_M from the truth for ONSET_S
                        without a break; the loss's onset is the first sample of that stretch
  onsets_off_map        onsets with the robot truly off the known-free cells
  metres_on_map, metres_off_map
                        the robot's true path over the scored windows, on and off the known-free cells
  losses_per_100m_off   onsets_off_map per 100 m driven off the known-free cells
  odom_err_median_m, odom_err_max_m
                        over the ODOM_WINDOW_S before each onset, the wheel odometry's own motion laid
                        on the true pose at the window's start, against the true pose at the onset:
                        whether the odometry AMCL used had drifted (it had not when this is small)
  contact_before_onset  onsets with the true body within compare_footprint.CONTACT_M of the
                        geometry in the CONTACT_WINDOW_S before
  tours_lost_at_failed_goal, goals_after_loss, failed_after_loss
                        loc_guard_compare.py's rule: a goal failed with the belief more than LOST_M
                        from the truth at the end of its last attempt, and the tour's later goals
  false_claims, false_belief_err_min_m, false_belief_err_max_m
                        the claims beyond ARRIVE_M and how far the belief was from the truth at them

Output in --out: nav2_localization.csv (one row per arm); the summary is printed. Needs runs/raw/
(not part of the repository); without batch data nothing is written.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "missions"))
import collision_sources as cs  # noqa: E402
import compare_footprint as cf  # noqa: E402
import loc_guard_compare as lgc  # noqa: E402
import summarize_arms as sa  # noqa: E402
import verify_missions as vm  # noqa: E402
from gridmap import GridMap  # noqa: E402

LOST_M = lgc.LOST_M       # 1.0 m, as the guard's comparison
ONSET_S = 3.0             # a loss lasts at least this long (the guard's LOST_S)
DT_S = 0.5                # sampling step over the scored window
ODOM_WINDOW_S = 60.0
CONTACT_WINDOW_S = 20.0
BUDGETS = (1.0, 2.5, 5.0, 10.0)


def onset(err):
    """Index of the first sample of the first stretch of ONSET_S with err above LOST_M, or None."""
    need, run = int(round(ONSET_S / DT_S)), 0
    for i, v in enumerate(err):
        run = run + 1 if v > LOST_M else 0
        if run >= need:
            return i - need + 1
    return None


def tour(batch_dir, seed, budget, goal_ids):
    """The tour's samples and counts, or None when a file is missing."""
    td = sa.final_try(os.path.join(batch_dir, seed, f"tour_b{budget:g}"))
    ex = sa.final_try(os.path.join(batch_dir, seed, "explore"))
    if not td or not ex:
        return None
    mpath, rows = sa.tour_rows(td, set(goal_ids))
    truth, b = cs.read_truth(td), cs.read_belief(td)
    if not rows or truth is None or b is None or b.shape[1] < 8:
        print(f"left out: {td} (no attempts, ground truth or belief with odometry)")
        return None
    t, wall, x, y, yaw = truth
    gm = GridMap(os.path.join(ex, f"map_b{budget:g}.yaml"))
    dist = lgc.dist_to_known_free(gm)
    w0, w1 = sa.tour_window(rows)
    g = np.arange(w0, w1, DT_S)
    bi = np.clip(np.searchsorted(b[:, 1], g, side="right") - 1, 0, len(b) - 1)
    ti = np.clip(np.searchsorted(wall, g), 0, len(wall) - 1)
    err = np.hypot(b[bi, 2] - x[ti], b[bi, 3] - y[ti])
    on_map = np.array([gm.inside(r, c) and dist[r, c] == 0
                       for r, c in (gm.to_cell(px, py) for px, py in zip(x[ti], y[ti]))])
    step = np.r_[0.0, np.hypot(np.diff(x[ti]), np.diff(y[ti]))]
    out = dict(metres_on=float(step[on_map].sum()), metres_off=float(step[~on_map].sum()), lost=0)
    i = onset(err)
    if i is not None:
        out["lost"] = 1
        out["onset_off_map"] = int(not on_map[i])
        i0 = max(0, i - int(round(ODOM_WINDOW_S / DT_S)))
        th = yaw[ti[i0]] - b[bi[i0], 7]
        c, s = math.cos(th), math.sin(th)
        dx, dy = b[bi[i], 5] - b[bi[i0], 5], b[bi[i], 6] - b[bi[i0], 6]
        out["odom_err"] = math.hypot(x[ti[i0]] + c * dx - s * dy - x[ti[i]], y[ti[i0]] + s * dx + c * dy - y[ti[i]])
        k0 = int(np.searchsorted(wall, g[max(0, i - int(round(CONTACT_WINDOW_S / DT_S)))]))
        k1 = int(np.searchsorted(wall, g[i]))
        geo = cf.geometry(seed)
        dmin = cf.body_clearance(x[k0:k1 + 1], y[k0:k1 + 1], yaw[k0:k1 + 1], *geo)
        out["contact"] = int(len(dmin) > 0 and float(dmin.min()) <= cf.CONTACT_M)
    claims = sa.claim_verdicts(mpath, rows) if mpath else {}
    out["false_err"] = []
    for gid, (verdict, _) in claims.items():
        if verdict != "false":
            continue
        r = [q for q in rows if q["mission"] == gid and q["result"] == "SUCCEEDED"][0]
        j = vm.nearest(wall, float(r["epoch_end"]))
        out["false_err"].append(math.hypot(float(r["claim_x"]) - x[j], float(r["claim_y"]) - y[j]))
    lost_at, after, failed_after = None, 0, 0
    for n, gid in enumerate(goal_ids):
        att = [q for q in rows if q["mission"] == gid]
        if not att:
            continue
        real = claims.get(gid, ("",))[0] == "real"
        if lost_at is not None:
            after += 1
            failed_after += not real
            continue
        k = int(np.clip(np.searchsorted(b[:, 1], float(att[-1]["epoch_end"])), 0, len(b) - 1))
        j = vm.nearest(wall, float(att[-1]["epoch_end"]))
        if not real and math.hypot(b[k, 2] - x[j], b[k, 3] - y[j]) > LOST_M:
            lost_at = n
    out.update(lost_at_failed=int(lost_at is not None), after=after, failed_after=failed_after)
    return out


def arm_row(name, batch_dir, n_goals):
    tours = []
    for seed in sa.WORLDS:
        ids = sa.goal_order(seed)
        ids = ids[:n_goals] if n_goals else ids
        for b in BUDGETS:
            r = tour(batch_dir, seed, b, ids)
            if r is not None:
                tours.append(r)
    if not tours:
        return None
    lost = [r for r in tours if r["lost"]]
    on, off = sum(r["metres_on"] for r in tours), sum(r["metres_off"] for r in tours)
    onsets_off = sum(r["onset_off_map"] for r in lost)
    oe = [r["odom_err"] for r in lost]
    fe = [e for r in tours for e in r["false_err"]]
    return dict(arm=name, batch=os.path.relpath(batch_dir, ROOT), tours=len(tours), lost_tours=len(lost),
                onsets_off_map=onsets_off, metres_on_map=round(on), metres_off_map=round(off),
                losses_per_100m_off=round(100 * onsets_off / off, 2) if off else "",
                odom_err_median_m=round(float(np.median(oe)), 3) if oe else "",
                odom_err_max_m=round(float(max(oe)), 3) if oe else "",
                contact_before_onset=sum(r["contact"] for r in lost),
                tours_lost_at_failed_goal=sum(r["lost_at_failed"] for r in tours),
                goals_after_loss=sum(r["after"] for r in tours), failed_after_loss=sum(r["failed_after"] for r in tours),
                false_claims=len(fe), false_belief_err_min_m=round(min(fe), 2) if fe else "",
                false_belief_err_max_m=round(max(fe), 2) if fe else "")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--revised", default=os.path.join(ROOT, "runs", "raw", "wbatch_nav2rev"),
                    help="the reference arm: Nav2 with the predecessor's revised configuration")
    ap.add_argument("--first", default=os.path.join(ROOT, "runs", "raw", "wbatch2"),
                    help="the first configuration (v8); '' leaves it out")
    ap.add_argument("--goals", type=int, default=6, help="each tour's first N goals (default 6; 0 = all)")
    ap.add_argument("--out", default=os.path.join(ROOT, "analysis", "results"))
    a = ap.parse_args()
    rows = []
    for name, d in (("revised", a.revised), ("first", a.first)):
        if not d:
            continue
        if not os.path.isdir(d):
            print(f"{name}: no batch directory {d}")
            continue
        r = arm_row(name, d, a.goals)
        if r:
            rows.append(r)
    if not rows:
        print("no batch data found (runs/raw/ is not part of the repository); nothing written")
        return 1
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "nav2_localization.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()), lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    for r in rows:
        print(f"{r['arm']:8s} ({r['batch']}): AMCL more than {LOST_M:g} m off for {ONSET_S:g} s in {r['lost_tours']} of "
              f"{r['tours']} tours, {r['onsets_off_map']} of the onsets off the known-free cells; driven {r['metres_on_map']} m "
              f"on them and {r['metres_off_map']} m off, {r['losses_per_100m_off']} losses per 100 m off; odometry error over "
              f"the {ODOM_WINDOW_S:g} s before an onset median {r['odom_err_median_m']} m, max {r['odom_err_max_m']} m; "
              f"contact in the {CONTACT_WINDOW_S:g} s before {r['contact_before_onset']}; a goal failed on a lost belief in "
              f"{r['tours_lost_at_failed_goal']} tours, {r['failed_after_loss']} of the {r['goals_after_loss']} goals after it "
              f"failed; false claims {r['false_claims']} (belief {r['false_belief_err_min_m']}-{r['false_belief_err_max_m']} m off)")
    print(f"-> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
