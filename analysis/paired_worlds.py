#!/usr/bin/env python3
"""Paired comparisons of the arms' verified arrivals, world by world, with the uncertainty ten worlds allow.

  python3 analysis/paired_worlds.py [--runs analysis/results/runs.csv] [--first runs/raw/wbatch2]
                                    [--out analysis/results]

The worlds, not the tours, are the units: the four tours of a world share its exploration, and
its goals are the same for every arm. For each pair of arms (`a_minus_b`) and each budget, the
difference per world of the real arrivals, in goals and in points (each tour's real arrivals as a
share of its scored goals, x100); over the four budgets (`all`) the difference per world of the
summed real arrivals and of the pooled share (the world's real arrivals over its scored goals,
x100). Per row: the mean over the worlds and a percentile bootstrap 95 % interval of it (DRAWS
resamples of the worlds, seed SEED), the worlds above, equal and below, and the exact two-sided
sign test over the worlds that differ.

Pairs: `aerial_minus_wheeled` (the two robots on the shared executive), `wheeled_minus_wheeled_nav2`
(the executive against Nav2's revised configuration, the reference arm) and
`wheeled_nav2_minus_first` (the revised configuration against the first, v8, read from --first, the
batch summarize_arms.py reads with --wheeled-nav2; '' leaves this pair out, and without that batch
nothing is written). The three arms come from --runs (summarize_arms.py's runs.csv).
Output paired_worlds.csv in --out; the table is printed.
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
import summarize_arms as sa  # noqa: E402

DRAWS = 20000
SEED = 20261014
BUDGETS = (1.0, 2.5, 5.0, 10.0)
PAIRS = (("aerial", "wheeled"), ("wheeled", "wheeled_nav2"), ("wheeled_nav2", "first"))


def sign_test(up, down):
    """Exact two-sided sign test over the worlds that differ."""
    n = up + down
    if n == 0:
        return 1.0
    k = min(up, down)
    return min(1.0, 2.0 * sum(math.comb(n, i) for i in range(k + 1)) / 2.0 ** n)


def interval(d, rng):
    """(lo, hi) of the percentile bootstrap 95 % interval of the mean of d over the worlds."""
    idx = rng.integers(0, len(d), size=(DRAWS, len(d)))
    m = d[idx].mean(axis=1)
    return float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))


def table(rows):
    """{(arm, world, budget): (real, scored)} of the rows with a tour."""
    out = {}
    for r in rows:
        if r["real"] == "" or r["goals_scored"] in ("", 0, "0"):
            continue
        out[(r["embodiment"], r["world"], float(r["budget_min"]))] = (int(r["real"]), int(r["goals_scored"]))
    return out


def row(name, budget, a, b, rng):
    ga, gb = np.array([x[0] for x in a], float), np.array([x[0] for x in b], float)
    pa = np.array([100.0 * x[0] / x[1] for x in a])
    pb = np.array([100.0 * x[0] / x[1] for x in b])
    dg, dp = ga - gb, pa - pb
    lo_g, hi_g = interval(dg, rng)
    lo_p, hi_p = interval(dp, rng)
    up, eq, down = int((dp > 1e-9).sum()), int((abs(dp) <= 1e-9).sum()), int((dp < -1e-9).sum())
    return dict(comparison=name, budget_min=budget, worlds=len(dg),
                mean_goals=round(float(dg.mean()), 2), lo_goals=round(lo_g, 2), hi_goals=round(hi_g, 2),
                mean_points=round(float(dp.mean()), 1), lo_points=round(lo_p, 1), hi_points=round(hi_p, 1),
                worlds_up=up, worlds_equal=eq, worlds_down=down, sign_test_p=round(sign_test(up, down), 4))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", default=os.path.join(ROOT, "analysis", "results", "runs.csv"))
    ap.add_argument("--first", default=os.path.join(ROOT, "runs", "raw", "wbatch2"),
                    help="the first Nav2 configuration's batch ('' leaves its pair out)")
    ap.add_argument("--goals", type=int, default=6, help="each tour's first N goals for --first (default 6)")
    ap.add_argument("--out", default=os.path.join(ROOT, "analysis", "results"))
    a = ap.parse_args()
    if not os.path.isfile(a.runs):
        print(f"no {a.runs}: run summarize_arms.py first; nothing written")
        return 1
    with open(a.runs, newline="") as f:
        t = table(list(csv.DictReader(f)))
    if a.first and not os.path.isdir(a.first):
        print(f"first: no batch directory {a.first} (runs/raw/ is not part of the repository); "
              "--first '' leaves its pair out; nothing written")
        return 1
    if a.first:
        t.update({("first",) + k[1:]: v for k, v in table(sa.collect("first", a.first, list(BUDGETS), a.goals)).items()})
    rng = np.random.default_rng(SEED)
    out = []
    for x, y in PAIRS:
        worlds = [w for w in sa.WORLDS if all((x, w, b) in t and (y, w, b) in t for b in BUDGETS)]
        if not worlds:
            continue
        name = f"{x}_minus_{y}"
        for b in BUDGETS:
            out.append(row(name, b, [t[(x, w, b)] for w in worlds], [t[(y, w, b)] for w in worlds], rng))
        sums = lambda arm, w: (sum(t[(arm, w, b)][0] for b in BUDGETS), sum(t[(arm, w, b)][1] for b in BUDGETS))  # noqa: E731
        out.append(row(name, "all", [sums(x, w) for w in worlds], [sums(y, w) for w in worlds], rng))
    if not out:
        print("no pair of arms with tours in every world and budget; nothing written")
        return 1
    os.makedirs(a.out, exist_ok=True)
    with open(os.path.join(a.out, "paired_worlds.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0].keys()), lineterminator="\n")
        w.writeheader()
        w.writerows(out)
    print(f"{'comparison':28s} {'budget':>6s}  {'goals per world (95 % interval)':>32s}  {'points (95 % interval)':>26s}  up/eq/down  sign p")
    for r in out:
        print(f"{r['comparison']:28s} {str(r['budget_min']):>6s}  {r['mean_goals']:+7.2f} ({r['lo_goals']:+6.2f} to {r['hi_goals']:+6.2f})"
              f"            {r['mean_points']:+6.1f} ({r['lo_points']:+6.1f} to {r['hi_points']:+6.1f})   "
              f"{r['worlds_up']:2d}/{r['worlds_equal']:2d}/{r['worlds_down']:2d}   {r['sign_test_p']:.4f}")
    print(f"-> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
