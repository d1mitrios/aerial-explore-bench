#!/usr/bin/env python3
"""The arms' batch results in one table and two curves.

  python3 analysis/summarize_arms.py [--aerial runs/raw/batch] [--wheeled runs/raw/wbatch3]
                                     [--wheeled-nav2 runs/raw/wbatch2] [--goals N] [--out analysis/results]

Three arms: `aerial` (the quadrotor, runs/raw/batch), `wheeled` (the wheeled robot on the
same A* executive as the quadrotor, runs/raw/wbatch3) and `wheeled_nav2` (the wheeled robot on the
baseline's Nav2 stack, runs/raw/wbatch2, kept as the reference arm). `--wheeled` (alias
`--wheeled-dir`) and `--wheeled-nav2` name their batch directories; an empty `--wheeled-nav2 ""`
leaves the reference arm out.

For every world and budget, from the FINAL try of each phase only:
  coverage_m2   the frozen map's known-free area (the exploration manifest's checkpoint;
                `final` = a run that ended early gave its last map to this budget)
  real / false  the tour's goals claimed with the truth within / beyond ARRIVE_M = 1.5 m of
                the goal at the claim (verify_missions.py's rule), out of the
                goals scored; claimed = any attempt SUCCEEDED
  void          claims the stack's own belief contradicts: a SUCCEEDED whose belief at
                the claim (claim_x, claim_y) was itself beyond ARRIVE_M of the goal - not a
                localization error but a success reported without the estimate behind it (the
                wheeled arm's Nav2 1.3.12 isGoalReached after a failed goal transform: 013
                tour_b5 g01 / g02, 016 tour_b10 g01-g03, the belief 5.4-9.2 m from the goal
                against at most 0.9 m for every other claim of either arm); the goal was never
                tried, so it is neither real nor false and leaves the denominator
                (goals_scored = goals - void)
  impacts       the quadrotor executive's IMU impacts (manifest events; blank for both wheeled
                arms: the Nav2 runner has no detector and the ground vehicle's executive logged
                no impact event)
  explore_end   the exploration's end reason and sim-seconds of policy time
--goals N keeps each tour's first N goals in the goals file's order (default 6; 0 = all):
the goals run in file order, one after the other, so the first N goals of a longer tour are
exactly the tour of N goals; the aerial impacts are then counted up to the
end of goal N's last attempt. The aerial batch ran 10 goals per tour, the Nav2 wheeled batch 6 from
world 008's tour_b2.5 on, the shared-executive wheeled batch 6 throughout.
Outputs in --out: runs.csv (one row per embodiment x world x budget), budgets.csv (per
embodiment x budget: mean real arrivals in % of the goals, min / max over worlds, false
arrivals, mean coverage), success_vs_budget.png, coverage_vs_budget.png. A world or phase
without a FINAL try is listed as missing, never guessed.
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
import verify_missions as vm  # noqa: E402

WORLDS = ["20260723008", "20260723001", "20260723002", "20260723003", "20260723004",
          "20260723005", "20260723013", "20260723016", "20260723018", "20260723023"]


def final_try(phase_dir):
    """The FINAL try of a phase (the last one, as the batch's final_try), or None."""
    tries = sorted(glob.glob(os.path.join(phase_dir, "try[0-9]*")),
                   key=lambda p: int(os.path.basename(p)[3:]))
    fin = [t for t in tries if os.path.isfile(os.path.join(t, "FINAL"))
           and not os.path.exists(os.path.join(t, "interrupted_by_user"))]
    return fin[-1] if fin else None


def newest(pattern):
    c = sorted(glob.glob(pattern), key=os.path.getmtime)
    return c[-1] if c else None


def belief_miss(r):
    """The claim's own belief to the goal (m), nan when the runner logged no belief."""
    try:
        return math.hypot(float(r.get("claim_x") or "nan") - float(r["goal_x"]),
                          float(r.get("claim_y") or "nan") - float(r["goal_y"]))
    except ValueError:
        return float("nan")


def tour_rows(run_dir, goal_ids):
    """(missions CSV path, its attempts of the goals in goal_ids) of one tour, or (None, [])."""
    mpath = newest(os.path.join(run_dir, "missions_*.csv"))
    if not mpath:
        return None, []
    return mpath, [r for r in vm.read_missions(mpath) if r["mission"] in goal_ids]


def claim_verdicts(mpath, rows):
    """{goal: (verdict, metres)} of each goal's first claim by the verifier's rule: 'real' / 'false'
    with the truth's distance to the goal at the claim, or 'void' with the belief's."""
    if not any(r["result"] == "SUCCEEDED" for r in rows):
        return {}
    gt = vm.read_gt(vm.pick_gt(mpath, None, rows))
    wall, x, y = gt[:, 1], gt[:, 2], gt[:, 3]
    out = {}
    for r in rows:
        if r["result"] != "SUCCEEDED" or r["mission"] in out:
            continue
        miss = belief_miss(r)
        if miss > vm.ARRIVE_M:                    # nan compares False: no belief logged, a normal claim
            out[r["mission"]] = ("void", miss)
            continue
        t_end = float(r["epoch_end"])
        i = int(np.clip(np.searchsorted(wall, t_end), 0, len(wall) - 1))
        if i > 0 and abs(wall[i - 1] - t_end) < abs(wall[i] - t_end):
            i -= 1
        d = math.hypot(x[i] - float(r["goal_x"]), y[i] - float(r["goal_y"]))
        out[r["mission"]] = ("real" if d <= vm.ARRIVE_M else "false", d)
    return out


def tour_verdict(run_dir, goal_ids):
    """(real, false, void, claimed, attempts, goals_seen, t_end) of one tour by the verifier's
    rule, over the goals in goal_ids only (t_end: the wall time the last kept attempt ended);
    void: first claims whose own belief was beyond ARRIVE_M of the goal."""
    mpath, rows = tour_rows(run_dir, goal_ids)
    if not mpath:
        return None
    if not rows:
        return dict(real=0, false=0, void=0, claimed=0, attempts=0, goals_seen=0, t_end=None)
    v = [k for k, _ in claim_verdicts(mpath, rows).values()]
    return dict(real=v.count("real"), false=v.count("false"), void=v.count("void"), claimed=len(v),
                attempts=len(rows), goals_seen=len({r["mission"] for r in rows}),
                t_end=max(float(r["epoch_end"]) for r in rows))


def impacts(run_dir, t_end=None):
    """The executive's impact events, up to wall time t_end when given (a truncated tour)."""
    m = newest(os.path.join(run_dir, "manifest_*.json"))
    if not m:
        return None
    ev = json.load(open(m)).get("events", [])
    return sum(1 for e in ev if e.get("event") == "impact"
               and (t_end is None or float(e.get("wall", 0.0)) <= t_end))


def goal_order(seed):
    """The world's goal ids in the goals file's order (the order the tours run them)."""
    p = os.path.join(ROOT, "missions", "goals", f"goals_{seed}.csv")
    ids = []
    for ln in open(p):
        ln = ln.strip()
        if ln and not ln.startswith("#") and not ln.startswith("mission,"):
            ids.append(ln.split(",")[0].strip())
    return ids


def collect(arm, batch_dir, budgets, keep_goals=0):
    out = []
    for seed in WORLDS:
        wd = os.path.join(batch_dir, seed)
        ex = final_try(os.path.join(wd, "explore"))
        man = json.load(open(newest(os.path.join(ex, "manifest_*.json")))) if ex else None
        cps = {float(c["budget_min"]): c for c in (man or {}).get("checkpoints", [])}
        summ = (man or {}).get("summary") or {}
        goal_ids = goal_order(seed)
        if keep_goals:
            goal_ids = goal_ids[:keep_goals]
        ng = len(goal_ids)
        for b in budgets:
            row = dict(embodiment=arm, world=seed, budget_min=b, goals=ng,
                       explore_try=os.path.basename(ex) if ex else "", explore_end=summ.get("reason", ""),
                       explore_t_sim=summ.get("t_sim", ""))
            cp = cps.get(b)
            row["coverage_m2"] = round(cp["coverage_m2"], 1) if cp else ""
            row["map_final"] = cp.get("final", "") if cp else ""
            td = final_try(os.path.join(wd, f"tour_b{b:g}"))
            row["tour_try"] = os.path.basename(td) if td else ""
            v = tour_verdict(td, set(goal_ids)) if td else None
            for k in ("real", "false", "void", "claimed", "attempts", "goals_seen"):
                row[k] = v[k] if v else ""
            row["goals_scored"] = ng - v["void"] if v else ""
            t_end = v["t_end"] if (v and keep_goals) else None
            row["impacts"] = impacts(td, t_end) if (td and arm == "aerial") else ""
            row["missing"] = ";".join(w for w, ok in (("explore", ex), ("tour", td)) if not ok)
            out.append(row)
    return out


def aggregate(rows, budgets):
    agg = []
    for arm in sorted({r["embodiment"] for r in rows}):
        for b in budgets:
            rs = [r for r in rows if r["embodiment"] == arm and r["budget_min"] == b and r["real"] != ""]
            if not rs:
                continue
            pct = [100.0 * r["real"] / r["goals_scored"] for r in rs if r["goals_scored"]]
            cov = [r["coverage_m2"] for r in rs if r["coverage_m2"] != ""]
            imp = [r["impacts"] for r in rs if r["impacts"] != ""]
            agg.append(dict(embodiment=arm, budget_min=b, worlds=len(rs),
                            real_pct_mean=round(float(np.mean(pct)), 1), real_pct_min=round(min(pct), 1),
                            real_pct_max=round(max(pct), 1), real_total=sum(r["real"] for r in rs),
                            goals_total=sum(r["goals_scored"] for r in rs), false_total=sum(r["false"] for r in rs),
                            void_total=sum(r["void"] for r in rs),
                            coverage_mean_m2=round(float(np.mean(cov)), 1) if cov else "",
                            impacts_total=sum(imp) if imp else ""))
    return agg


def plot(rows, agg, budgets, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    colors = {"aerial": "#1f77b4", "wheeled": "#d62728", "wheeled_nav2": "#8c8c8c"}
    for metric, ylabel, fname in (("real", "real arrivals (% of the goals)", "success_vs_budget.png"),
                                  ("coverage_m2", "frozen map, known-free area (m²)", "coverage_vs_budget.png")):
        fig, ax = plt.subplots(figsize=(7, 5))
        for arm in sorted({r["embodiment"] for r in rows}):
            for seed in WORLDS:
                pts = [(r["budget_min"], (100.0 * r["real"] / r["goals_scored"]) if metric == "real" else r[metric])
                       for r in rows if r["embodiment"] == arm and r["world"] == seed and r[metric] != ""
                       and (metric != "real" or r["goals_scored"])]
                if pts:
                    ax.plot(*zip(*pts), "-", color=colors.get(arm, "k"), alpha=0.18, lw=1)
            key = "real_pct_mean" if metric == "real" else "coverage_mean_m2"
            pts = [(a["budget_min"], a[key]) for a in agg if a["embodiment"] == arm and a[key] != ""]
            if pts:
                ax.plot(*zip(*pts), "o-", color=colors.get(arm, "k"), lw=2.5, label=f"{arm} (mean of worlds)")
        ax.set_xscale("log")
        ax.set_xticks(budgets)
        ax.set_xticklabels([f"{b:g}" for b in budgets])
        ax.set_xlabel("exploration budget (sim-minutes)")
        ax.set_ylabel(ylabel)
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(os.path.join(out, fname), dpi=120)
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aerial", default=os.path.join(ROOT, "runs", "raw", "batch"))
    ap.add_argument("--wheeled", "--wheeled-dir", dest="wheeled", default=os.path.join(ROOT, "runs", "raw", "wbatch3"),
                    help="the wheeled arm on the shared A* executive")
    ap.add_argument("--wheeled-nav2", default=os.path.join(ROOT, "runs", "raw", "wbatch2"),
                    help="the wheeled reference arm on Nav2 ('' leaves it out)")
    ap.add_argument("--budgets", default="1,2.5,5,10")
    ap.add_argument("--goals", type=int, default=6, help="keep each tour's first N goals (default 6; 0 = all)")
    ap.add_argument("--out", default=os.path.join(ROOT, "analysis", "results"))
    a = ap.parse_args()
    budgets = [float(b) for b in a.budgets.split(",")]
    os.makedirs(a.out, exist_ok=True)
    rows = []
    for arm, d in (("aerial", a.aerial), ("wheeled", a.wheeled), ("wheeled_nav2", a.wheeled_nav2)):
        if not d:
            continue
        if os.path.isdir(d):
            rows += collect(arm, d, budgets, a.goals)
        else:
            print(f"{arm}: no batch directory {d}")
    if not rows:                                  # no run data: leave the committed tables and figures alone
        print("no batch data found (runs/raw/ is not part of the repository); nothing written")
        return 1
    agg = aggregate(rows, budgets)
    for name, data in (("runs.csv", rows), ("budgets.csv", agg)):
        if data:
            with open(os.path.join(a.out, name), "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(data[0].keys()))
                w.writeheader()
                w.writerows(data)
    plot(rows, agg, budgets, a.out)
    print(f"{'arm':12s} {'budget':>6s} {'worlds':>6s} {'real %':>7s} {'min-max':>9s} {'real/goals':>10s} {'false':>5s} "
          f"{'void':>4s} {'cov m2':>7s} {'impacts':>7s}")
    for g in agg:
        print(f"{g['embodiment']:12s} {g['budget_min']:6g} {g['worlds']:6d} {g['real_pct_mean']:7.1f} "
              f"{g['real_pct_min']:4.0f}-{g['real_pct_max']:<4.0f} {g['real_total']:4d}/{g['goals_total']:<5d} "
              f"{g['false_total']:5d} {g['void_total']:4d} {g['coverage_mean_m2']!s:>7s} {g['impacts_total']!s:>7s}")
    voids = [f"{r['embodiment']}/{r['world']}/b{r['budget_min']:g}: {r['void']}" for r in rows if r["void"]]
    if voids:
        print("void claims (the belief itself beyond ARRIVE_M of the goal): " + ", ".join(voids))
    miss = [r for r in rows if r["missing"]]
    if miss:
        print("missing (no FINAL try): " + ", ".join(f"{r['embodiment']}/{r['world']}/b{r['budget_min']:g}:{r['missing']}" for r in miss))
    print(f"-> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
