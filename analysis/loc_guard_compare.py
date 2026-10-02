#!/usr/bin/env python3
"""The localization guard against the first batches, which ran without it, on the same frozen maps.

  python3 analysis/loc_guard_compare.py [--aerial-without runs/raw/batch] [--aerial-with runs/raw/batch_lg]
                                        [--wheeled-without runs/raw/wbatch3] [--wheeled-with runs/raw/wbatch_lg]
                                        [--goals 6] [--out analysis/results]

The guard (missions/aerial_mission_runner.py --loc-guard, the constants in LOC_GUARD) takes AMCL's
correction only while the filter is healthy and dead-reckons on the odometry from the last
healthy correction once the covariance stays high or the pose jumps. The batches with the
guard (`--*-with`) hold copies of the explorations of the first batches (`--*-without`;
explore/try1, FINAL), so every tour of the first batches was flown again with the guard on the
same frozen map, and the two differ in the guard and in run-to-run variation alone. From the FINAL tries and each tour's first
--goals goals, per arm (`aerial`, `wheeled`) and tour (world x budget):

  real_without / real_with    verified arrivals (summarize_arms.py's rule, ARRIVE_M = 1.5 m); real_strict_*
                              the same under a stricter judge, the truth within STRICT_M = 0.5 m of the goal
  err_gt1_pct_*              share of the scored window's belief samples more than 1 m from the truth
                              (the window: the first goal's first attempt to the last goal's last)
  lost_without / _with        1 when a goal failed with the belief more than 1 m from the truth at the
                              end of its last attempt (the localization was lost there); goals_after_loss
                              and failed_after_loss count the tour's later goals
  dr, amcl, reseed            the guard's switches to dead reckoning, returns to AMCL and reseeds of
                              AMCL (the tour manifest's summary; the tours with the guard only)

Outputs in --out: loc_guard_tours.csv (one row per arm x tour), loc_guard.csv (per arm x budget and an
`all` row: real arrivals without and with the guard, the tours that gained, lost or stayed, the exact two-sided
sign test over the tours that changed, the belief error's share above 1 m and 0.5 m, its median and
95th percentile, the tours with a loss and the goals after it, the real arrivals at goals outside the
frozen map's known-free cells, the real arrivals under the stricter judge, the guard's activity, and
how far AMCL's first pose after a reseed lay from the seed) and loc_guard.png; the summary is printed.

It also measures, on the first batches, what the guard's constants rest on
(loc_guard_calibration.csv, one row per arm): every /amcl_pose of the scored windows against the
truth, the share more than 1 m off by where the robot truly was (on a known-free cell of the
frozen map, or up to 0.5 m, 0.5-1.5 m or beyond 1.5 m from one), the tours in which AMCL was ever
more than 1 m off and how many of them the guard's covariance rule (above LOST_COV for LOST_S of
AMCL stamps, LOC_GUARD's values) flagged at all and before the error passed 1 m, and the median
error of the updates the guard counts as healthy (covariance below OK_COV). Needs runs/raw/ (not
part of the repository); without batch data nothing is written.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "missions"))
import summarize_arms as sa  # noqa: E402
import collision_sources as cs  # noqa: E402
from gridmap import GridMap  # noqa: E402

LOST_M = 1.0
STRICT_M = 0.5
LOST_COV, LOST_S, OK_COV = 0.5, 3.0, 0.1      # missions/aerial_mission_runner.py LOC_GUARD


def dist_to_known_free(gm, max_cells=60):
    """Per cell, the distance (m) to the nearest known-free cell (8-neighbour steps, inf beyond max_cells)."""
    cur = gm.cls == GridMap.FREE
    dist = np.where(cur, 0.0, np.inf)
    for it in range(1, max_cells + 1):
        n = cur.copy()
        n[1:] |= cur[:-1]
        n[:-1] |= cur[1:]
        n[:, 1:] |= cur[:, :-1]
        n[:, :-1] |= cur[:, 1:]
        dist[n & ~cur] = it * gm.resolution
        cur = n
    return dist


def calibrate(batch_dir, budgets, n_goals):
    """AMCL against the truth on the tours of one batch (see the module docstring)."""
    err_all, bins_all, cov_all = [], [], []
    lost = flagged = flagged_early = 0
    for seed in sa.WORLDS:
        ids = sa.goal_order(seed)
        ids = set(ids[:n_goals] if n_goals else ids)
        for b in budgets:
            td = sa.final_try(os.path.join(batch_dir, seed, f"tour_b{b:g}"))
            ex = sa.final_try(os.path.join(batch_dir, seed, "explore"))
            if not td or not ex:
                continue
            _, rows = sa.tour_rows(td, ids)
            f = [p for p in sorted(glob.glob(os.path.join(td, "amcl_*.csv"))) if not p.endswith(".log")]
            if not rows or not f:
                continue
            a = np.genfromtxt(f[-1], delimiter=",", comments="#", skip_header=2, ndmin=2, invalid_raise=False)
            w0 = min(float(r["epoch_start"]) for r in rows)
            w1 = max(float(r["epoch_end"]) for r in rows)
            a = a[(a[:, 1] >= w0) & (a[:, 1] <= w1)]
            if not len(a):
                continue
            _, wall, x, y, _ = cs.read_truth(td)
            j = np.clip(np.searchsorted(wall, a[:, 1]), 0, len(wall) - 1)
            err = np.hypot(a[:, 2] - x[j], a[:, 3] - y[j])
            cov = np.maximum(a[:, 5], a[:, 6])
            gm = GridMap(os.path.join(ex, f"map_b{b:g}.yaml"))
            dist = dist_to_known_free(gm)
            d = []
            for xi, yi in zip(x[j], y[j]):
                r_, c_ = gm.to_cell(xi, yi)
                d.append(dist[r_, c_] if gm.inside(r_, c_) else np.inf)
            err_all.append(err)
            cov_all.append(cov)
            bins_all.append(np.array(d))
            if (err > LOST_M).any():
                lost += 1
                since, det = None, None
                for i in range(len(a)):
                    if cov[i] > LOST_COV:
                        since = a[i, 0] if since is None else since
                        if a[i, 0] - since >= LOST_S:
                            det = i
                            break
                    else:
                        since = None
                if det is not None:
                    flagged += 1
                    flagged_early += a[det, 1] <= a[np.argmax(err > LOST_M), 1]
    e, d, c = np.concatenate(err_all), np.concatenate(bins_all), np.concatenate(cov_all)
    row = dict(updates=len(e), err_gt1_pct=round(100 * float(np.mean(e > LOST_M)), 1))
    for name, m in (("known_free", d == 0), ("within_0.5m", (d > 0) & (d <= 0.5)), ("0.5_to_1.5m", (d > 0.5) & (d <= 1.5)),
                    ("beyond_1.5m", d > 1.5)):
        row[f"share_{name}_pct"] = round(100 * float(np.mean(m)), 1)
        row[f"err_gt1_{name}_pct"] = round(100 * float(np.mean(e[m] > LOST_M)), 1) if m.any() else ""
    row.update(tours_lost=lost, flagged=flagged, flagged_before_loss=int(flagged_early),
               healthy_median_err_m=round(float(np.median(e[c < OK_COV])), 3))
    return row


def tour(batch_dir, seed, budget, goal_ids):
    """One tour's numbers, or None when the tour has no FINAL try."""
    td = sa.final_try(os.path.join(batch_dir, seed, f"tour_b{budget:g}"))
    ex = sa.final_try(os.path.join(batch_dir, seed, "explore"))
    if not td or not ex:
        return None
    mpath, rows = sa.tour_rows(td, set(goal_ids))
    if not rows:
        return None
    claims = sa.claim_verdicts(mpath, rows) if mpath else {}
    gm = GridMap(os.path.join(ex, f"map_b{budget:g}.yaml"))
    t, wall, x, y, _ = cs.read_truth(td)
    b = cs.read_belief(td)
    w0 = min(float(r["epoch_start"]) for r in rows)
    w1 = max(float(r["epoch_end"]) for r in rows)
    k = (b[:, 1] >= w0) & (b[:, 1] <= w1)
    j = np.clip(np.searchsorted(wall, b[k, 1]), 0, len(wall) - 1)
    err = np.hypot(b[k, 2] - x[j], b[k, 3] - y[j])
    out = dict(real=0, real_strict=0, real_unmapped=0, lost=0, goals_after_loss=0, failed_after_loss=0, err=err)
    lost = False
    for g in goal_ids:
        att = [r for r in rows if r["mission"] == g]
        if not att:
            continue
        real = g in claims and claims[g][0] == "real"
        gx, gy = float(att[-1]["goal_x"]), float(att[-1]["goal_y"])
        r_, c_ = gm.to_cell(gx, gy)
        if real:
            out["real"] += 1
            out["real_strict"] += claims[g][1] <= STRICT_M
            out["real_unmapped"] += (not gm.inside(r_, c_)) or gm.cls[r_, c_] != GridMap.FREE
        if lost:
            out["goals_after_loss"] += 1
            out["failed_after_loss"] += not real
            continue
        we = float(att[-1]["epoch_end"])
        bi, ti = cs.at(b[:, 1], we), cs.at(wall, we)
        if not real and math.hypot(b[bi, 2] - x[ti], b[bi, 3] - y[ti]) > LOST_M:
            lost = True
            out["lost"] = 1
    man = json.load(open(sa.newest(os.path.join(td, "manifest_*.json"))))
    g = (man.get("summary") or {}).get("loc_guard") or {}
    out.update(dr=g.get("dr", ""), amcl=g.get("amcl", ""), reseed=g.get("reseed", ""))
    # how far AMCL's first pose after each reseed of the scored window lay from the seed
    seeds = [e for e in man.get("events", []) if e.get("event") == "loc_guard_reseed" and w0 <= e["wall"] <= w1]
    out["seed_next_m"] = []
    if seeds:
        f = [p for p in sorted(glob.glob(os.path.join(td, "amcl_*.csv"))) if not p.endswith(".log")]
        a = np.genfromtxt(f[-1], delimiter=",", comments="#", skip_header=2, ndmin=2, invalid_raise=False)
        for e in seeds:
            i = int(np.searchsorted(a[:, 1], e["wall"]))
            if i < len(a):
                out["seed_next_m"].append(math.hypot(a[i, 2] - e["x"], a[i, 3] - e["y"]))
    return out


def sign_test(pos, neg):
    """Exact two-sided sign test over the tours that changed."""
    n = pos + neg
    if n == 0:
        return 1.0
    return min(1.0, 2.0 * sum(math.comb(n, i) for i in range(min(pos, neg) + 1)) / 2 ** n)


def summary(arm, label, pairs):
    """One loc_guard.csv row from (without, with the guard) tour pairs."""
    d = [a["real"] - b["real"] for b, a in pairs]
    pos, neg = sum(v > 0 for v in d), sum(v < 0 for v in d)
    row = dict(arm=arm, budget_min=label, tours=len(pairs),
               real_without=sum(b["real"] for b, _ in pairs), real_with=sum(a["real"] for _, a in pairs))
    row.update(tours_better=pos, tours_worse=neg, tours_equal=len(d) - pos - neg, sign_test_p=float(f"{sign_test(pos, neg):.2g}"))
    for tag, idx in (("without", 0), ("with", 1)):
        e = np.concatenate([p[idx]["err"] for p in pairs])
        row[f"err_gt1_pct_{tag}"] = round(100 * float(np.mean(e > 1.0)), 1)
        row[f"err_gt05_pct_{tag}"] = round(100 * float(np.mean(e > 0.5)), 1)
        row[f"err_median_m_{tag}"] = round(float(np.median(e)), 3)
        row[f"err_p95_m_{tag}"] = round(float(np.percentile(e, 95)), 2)
        row[f"lost_tours_{tag}"] = sum(p[idx]["lost"] for p in pairs)
        row[f"goals_after_loss_{tag}"] = sum(p[idx]["goals_after_loss"] for p in pairs)
        row[f"failed_after_loss_{tag}"] = sum(p[idx]["failed_after_loss"] for p in pairs)
        row[f"real_unmapped_{tag}"] = sum(p[idx]["real_unmapped"] for p in pairs)
        row[f"real_strict_{tag}"] = sum(p[idx]["real_strict"] for p in pairs)
    on = [a for _, a in pairs]
    row["tours_with_dr"] = sum(1 for a in on if a["dr"] not in ("", 0))
    for k in ("dr", "amcl", "reseed"):
        row[k] = sum(a[k] for a in on if a[k] != "")
    nxt = [v for a in on for v in a["seed_next_m"]]
    row["seed_next_median_m"] = round(float(np.median(nxt)), 2) if nxt else ""
    row["seed_next_over_1m"] = sum(v > 1.0 for v in nxt)
    return row


def plot(rows, budgets, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"aerial": "#1f77b4", "wheeled": "#d62728"}
    names = {"aerial": "quadrotor", "wheeled": "wheeled robot"}
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2))
    for ax, (key, ylabel) in zip(axes, (("real", "verified arrivals (of 60 goals)"),
                                        ("err_gt1_pct", "belief more than 1 m off (% of the tour time)"))):
        for arm in ("aerial", "wheeled"):
            rs = [r for r in rows if r["arm"] == arm and r["budget_min"] != "all"]
            for tag, ls, mk in (("without", "--", "o"), ("with", "-", "o")):
                col = f"{key}_{tag}"
                ys = [r[col] for r in rs]
                ax.plot(budgets, ys, ls, marker=mk, color=colors[arm], lw=2 if tag == "with" else 1.2,
                        mfc=colors[arm] if tag == "with" else "white", ms=6,
                        label=f"{names[arm]}, {'with the guard' if tag == 'with' else 'without the guard'}")
        ax.set_xscale("log")
        ax.set_xticks(budgets)
        ax.set_xticklabels([f"{b:g}" for b in budgets])
        ax.set_xticks([], minor=True)
        ax.set_xlabel("exploration budget (simulated minutes)")
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
        ax.set_ylim(bottom=0)
    axes[0].legend(fontsize=8)
    axes[0].set_title("Verified arrivals, without and with the localization guard", fontsize=10)
    axes[1].set_title("How often the belief was lost", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "loc_guard.png"), dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    raw = os.path.join(ROOT, "runs", "raw")
    ap.add_argument("--aerial-without", default=os.path.join(raw, "batch"))
    ap.add_argument("--aerial-with", default=os.path.join(raw, "batch_lg"))
    ap.add_argument("--wheeled-without", default=os.path.join(raw, "wbatch3"))
    ap.add_argument("--wheeled-with", default=os.path.join(raw, "wbatch_lg"))
    ap.add_argument("--budgets", default="1,2.5,5,10")
    ap.add_argument("--goals", type=int, default=6, help="each tour's first N goals (default 6; 0 = all)")
    ap.add_argument("--out", default=os.path.join(ROOT, "analysis", "results"))
    a = ap.parse_args()
    budgets = [float(b) for b in a.budgets.split(",")]
    dirs = {"aerial": (a.aerial_without, a.aerial_with), "wheeled": (a.wheeled_without, a.wheeled_with)}
    if not all(os.path.isdir(d) for pair in dirs.values() for d in pair):
        print("no batch data found (runs/raw/ is not part of the repository); nothing written")
        return 1
    tours, rows = [], []
    for arm, (off, on) in dirs.items():
        pairs = {}
        for seed in sa.WORLDS:
            ids = sa.goal_order(seed)
            ids = ids[:a.goals] if a.goals else ids
            for b in budgets:
                tb, ta = tour(off, seed, b, ids), tour(on, seed, b, ids)
                if tb is None or ta is None:
                    print(f"{arm} {seed} b{b:g}: missing a FINAL tour, left out")
                    continue
                pairs[(seed, b)] = (tb, ta)
                tours.append(dict(arm=arm, world=seed, budget_min=b, real_without=tb["real"], real_with=ta["real"],
                                  err_gt1_pct_without=round(100 * float(np.mean(tb["err"] > 1.0)), 1),
                                  err_gt1_pct_with=round(100 * float(np.mean(ta["err"] > 1.0)), 1),
                                  lost_without=tb["lost"], lost_with=ta["lost"],
                                  dr=ta["dr"], amcl=ta["amcl"], reseed=ta["reseed"]))
        for b in budgets:
            rows.append(summary(arm, b, [v for k, v in pairs.items() if k[1] == b]))
        rows.append(summary(arm, "all", list(pairs.values())))
    os.makedirs(a.out, exist_ok=True)
    for name, data in (("loc_guard_tours.csv", tours), ("loc_guard.csv", rows)):
        with open(os.path.join(a.out, name), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(data[0].keys()))
            w.writeheader()
            w.writerows(data)
    plot(rows, budgets, a.out)
    cal = []
    for arm, (off, _) in dirs.items():
        cal.append(dict(arm=arm, batch=os.path.relpath(off, ROOT), **calibrate(off, budgets, a.goals)))
    with open(os.path.join(a.out, "loc_guard_calibration.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(cal[0].keys()))
        w.writeheader()
        w.writerows(cal)
    for r in cal:
        print(f"{r['arm']:8s} without the guard: {r['updates']} AMCL updates, {r['err_gt1_pct']}% more than 1 m off;"
              f" on a known-free cell {r['err_gt1_known_free_pct']}%, within 0.5 m of one {r['err_gt1_within_0.5m_pct']}%,"
              f" 0.5-1.5 m {r['err_gt1_0.5_to_1.5m_pct']}%, beyond 1.5 m {r['err_gt1_beyond_1.5m_pct']}%"
              f" | tours with AMCL > 1 m off {r['tours_lost']}, flagged {r['flagged']} ({r['flagged_before_loss']} before the"
              f" error passed 1 m) | healthy updates: median error {r['healthy_median_err_m']} m")
    print("per arm and budget, without / with the guard:")
    for r in rows:
        lab = r["budget_min"] if r["budget_min"] == "all" else f"b{r['budget_min']:g}"
        print(f"{r['arm']:8s} {lab:5s} real {r['real_without']:3d} / {r['real_with']:3d}"
              f" | tours +{r['tours_better']} -{r['tours_worse']} ={r['tours_equal']} p={r['sign_test_p']}"
              f" | belief >1 m {r['err_gt1_pct_without']}% / {r['err_gt1_pct_with']}%,"
              f" p95 {r['err_p95_m_without']} / {r['err_p95_m_with']} m"
              f" | lost tours {r['lost_tours_without']} / {r['lost_tours_with']}"
              f" (goals after {r['goals_after_loss_without']}/{r['failed_after_loss_without']} failed /"
              f" {r['goals_after_loss_with']}/{r['failed_after_loss_with']})"
              f" | real at unmapped goals {r['real_unmapped_without']} / {r['real_unmapped_with']}"
              f" | guard: {r['tours_with_dr']} tours, dr {r['dr']}, back {r['amcl']}, reseeds {r['reseed']}"
              f" (AMCL after a reseed {r['seed_next_median_m']} m from the seed, {r['seed_next_over_1m']} beyond 1 m)"
              f" | under a {STRICT_M} m judge {r['real_strict_without']} / {r['real_strict_with']}")
    print(f"-> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
