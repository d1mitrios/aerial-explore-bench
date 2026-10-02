#!/usr/bin/env python3
"""Every tour goal's outcome in one class, for every arm, and the wheeled robot's contact.

  python3 analysis/goal_outcomes.py [--aerial runs/raw/batch] [--wheeled runs/raw/wbatch3]
                                    [--wheeled-nav2 runs/raw/wbatch2] [--goals 6] [--out analysis/results]

Three arms: `aerial` (the quadrotor), `wheeled` (the wheeled robot on the same A* executive
as the quadrotor, runs/raw/wbatch3) and `wheeled_nav2` (the wheeled robot on the baseline's Nav2
stack, runs/raw/wbatch2, the reference arm). The two arms on the executive share one set of
classes: the executive ends an attempt with the same reasons on either vehicle.

For every world and budget, from the FINAL tour try only, each of the tour's first N
goals gets one class, the first that applies:
  real         a claim (SUCCEEDED) with the truth within ARRIVE_M = 1.5 m of the goal
  false        a claim with the truth beyond ARRIVE_M (verify_missions.py's rule)
  void         a claim the stack's own belief contradicts
  frozen       wheeled_nav2: every attempt ended inside a freeze - a still stretch >= 60 sim-s the
               collision monitor's stop zone held for most of its length - and none was
               refused as outside the map (compare_footprint.py's rule)
  and otherwise the end of the goal's LAST attempt:
    aerial and wheeled (the executive's reasons):
               impact    the executive's IMU impact detector ended it (the quadrotor only; the
                         ground vehicle logged none)
               no_path   A* found no path to the goal on the frozen map
               blocked   no progress along the path for --blocked-s (a safety hold that lasted)
               contact   held against an obstacle for --blocked-s
               other     anything else (too_narrow: a narrow passage refused; odom_lost /
                         amcl_lost: an estimator loss; timeout)
    wheeled_nav2
               no_path   Nav2's planner found no valid path (208)
               off_map   the goal lies outside the frozen map (204)
               timeout   240 sim-s ran out with Nav2 still trying
               other     a controller abort (102 / 103) as the last end
  not_run      the goal was never sent (the tour ended before it)
Impacts are not a class: an aerial goal can see an impact on its first attempt and end on its
second as no_path; the arm's impacts are counted apart (summarize_arms.py).

Contact, each vehicle its own instrument: the quadrotor's IMU impacts (summarize_arms.py);
the wheeled robot, on either stack, from the ground truth - its true body (box + wheels)
within CONTACT_M = 0.01 m of the manifest's geometry (compare_footprint.py): episodes (contact
samples less than 2 sim-s apart merged) and seconds, from the tour's first goal to the end of
goal N's last attempt, as the impacts are cut.

Outputs in --out: goal_outcomes.csv (one row per arm x world x budget x goal),
outcomes_by_budget.csv (arm x budget x class counts), wheeled_contact.csv (per wheeled arm and tour),
outcomes_by_budget.png (stacked bars). A phase without a FINAL try is listed, never guessed.
"""
import argparse
import csv
import os
import sys
from collections import Counter, OrderedDict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import compare_footprint as cf  # noqa: E402
import summarize_arms as sa  # noqa: E402

CLASSES = {
    "aerial": ["real", "false", "void", "impact", "no_path", "blocked", "contact", "other", "not_run"],
    "wheeled": ["real", "false", "void", "impact", "no_path", "blocked", "contact", "other", "not_run"],
    "wheeled_nav2": ["real", "false", "void", "frozen", "no_path", "off_map", "timeout", "other", "not_run"],
}
EXECUTIVE = ("aerial", "wheeled")   # the arms driven by the A* executive
WHEELED = ("wheeled", "wheeled_nav2")
MERGE_S = 2.0              # contact samples closer than this (sim-s) are one episode


def last_end(arm, r):
    """The class of an attempt's end that was not a claim."""
    reason = r["reason"].strip()
    if arm in EXECUTIVE:
        return reason if reason in ("impact", "no_path", "blocked", "contact") else "other"
    if reason == "aborted 208":
        return "no_path"
    if reason == "aborted 204":
        return "off_map"
    if reason in ("timeout", "wall_cap"):
        return "timeout"
    return "other"


def wheeled_contact(run_dir, rows, geo):
    """(episodes, sim-s) of the true body within CONTACT_M of the geometry between the first
    attempt's start and the last kept attempt's end (wall clock, as the impacts are cut)."""
    g = cf.read_gt(run_dir)
    if g is None or not rows:
        return "", ""
    t, wall, x, y, yaw = g
    w0 = min(float(r["epoch_start"]) for r in rows)
    w1 = max(float(r["epoch_end"]) for r in rows)
    k = (wall >= w0) & (wall <= w1)
    if not k.any():
        return 0, 0.0
    tk = t[k]
    d = cf.body_clearance(x[k], y[k], yaw[k], *geo)
    hit = tk[d <= cf.CONTACT_M]
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.1
    episodes = int(len(hit) > 0) + int((np.diff(hit) > MERGE_S).sum()) if len(hit) else 0
    return episodes, round(len(hit) * dt, 1)


def tour_outcomes(arm, seed, budget, run_dir, goal_ids, geo):
    """[row per goal], contact row (wheeled) or None."""
    base = dict(arm=arm, world=seed, budget_min=budget, tour_try=os.path.basename(run_dir) if run_dir else "")
    if not run_dir:
        return [dict(base, goal=g, outcome="missing", attempts=0, last_end="", metres="") for g in goal_ids], None
    mpath, rows = sa.tour_rows(run_dir, set(goal_ids))
    claims = sa.claim_verdicts(mpath, rows) if mpath else {}
    frozen = set()
    if arm == "wheeled_nav2":
        g = cf.read_gt(run_dir)
        if g is not None:
            t, wall, x, y, yaw = g
            kind, ok, zones = cf.body_model(run_dir)
            held, _ = cf.stops(run_dir, zones, wall, t)
            frozen = cf.frozen_goal_ids(run_dir, set(goal_ids), cf.freezes(t, x, y, held))
    att = OrderedDict((g, []) for g in goal_ids)
    for r in rows:
        att[r["mission"]].append(r)
    out = []
    for g, rs in att.items():
        last = rs[-1] if rs else None
        if g in claims:
            cls, metres = claims[g]
        elif not rs:
            cls, metres = "not_run", ""
        elif g in frozen:
            cls, metres = "frozen", ""
        else:
            cls, metres = last_end(arm, last), ""
        out.append(dict(base, goal=g, outcome=cls, attempts=len(rs),
                        last_end=f"{last['result']} {last['reason'].strip()}" if last else "",
                        metres=round(metres, 2) if metres != "" else ""))
    contact = None
    if arm in WHEELED:
        ep, secs = wheeled_contact(run_dir, rows, geo)
        contact = dict(arm=arm, world=seed, budget_min=budget, tour_try=os.path.basename(run_dir),
                       contact_episodes=ep, contact_s=secs)
    return out, contact


# Display grouping of the native classes for the chart (the CSVs keep the native classes):
# (label, {arm: native classes}, colour, hatch). The same group has the same colour in both
# panels; a group absent from an arm is left out of that panel. Stacked bottom-up in this order.
GROUPS = [
    ("verified arrival", {"aerial": ["real"], "wheeled": ["real"], "wheeled_nav2": ["real"]}, "#1a7f37", None),
    ("not reachable on the explored map", {"aerial": ["no_path"], "wheeled": ["no_path"],
                                           "wheeled_nav2": ["no_path", "off_map"]}, "#8aa6c9", None),
    ("failed on the way", {"aerial": ["blocked", "contact", "other"], "wheeled": ["blocked", "contact", "other"],
                           "wheeled_nav2": ["timeout", "other"]}, "#cdb27a", None),
    ("collision abort", {"aerial": ["impact"], "wheeled": ["impact"], "wheeled_nav2": []}, "#b9755c", None),
    ("frozen by the safety stop", {"aerial": [], "wheeled": [], "wheeled_nav2": ["frozen"]}, "#9f86b8", None),
    ("false claim", {"aerial": ["false"], "wheeled": ["false"], "wheeled_nav2": ["false"]}, "#c0554f", None),
    ("excluded (Nav2 bug)", {"aerial": ["void"], "wheeled": ["void"], "wheeled_nav2": ["void"]}, "#e6e6e6", "////"),
    ("not run", {"aerial": ["not_run", "missing"], "wheeled": ["not_run", "missing"],
                 "wheeled_nav2": ["not_run", "missing"]}, "#ffffff", "..."),
]
PANELS = (("aerial", "quadrotor"), ("wheeled", "wheeled robot, shared executive"),
          ("wheeled_nav2", "wheeled robot, Nav2 (reference)"))
FIG_RC = {"font.family": "sans-serif", "font.sans-serif": ["Liberation Sans", "DejaVu Sans"],
          "font.size": 8, "axes.labelsize": 8, "xtick.labelsize": 8, "ytick.labelsize": 8,
          "legend.fontsize": 8, "axes.titlesize": 9, "pdf.fonttype": 42, "svg.fonttype": "none"}


def outcome_figure(counts, budgets, width=9.0, height=3.3):
    """The outcome chart as a matplotlib Figure: one panel per arm present, the native classes grouped
    as GROUPS (display only), verified arrivals in the one strong colour with their count
    written in the segment, one legend shared by both panels under them."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(budgets)
    x = np.arange(n)
    per_budget = max((sum(counts.get((arm, b), Counter()).values()) for arm, _ in PANELS for b in budgets),
                     default=0)
    with plt.rc_context(FIG_RC):
        panels = [(arm, name) for arm, name in PANELS if any((arm, b) in counts for b in budgets)]
        fig, axes = plt.subplots(1, len(panels), figsize=(width, height), sharey=True, squeeze=False)
        axes = axes[0]
        handles = {}
        for ax, (arm, name) in zip(axes, panels):
            bottom = np.zeros(n)
            for label, members, colour, hatch in GROUPS:
                vals = np.array([sum(counts.get((arm, b), Counter()).get(c, 0) for c in members[arm])
                                 for b in budgets], float)
                if not vals.any():
                    continue
                bars = ax.bar(x, vals, 0.62, bottom=bottom, color=colour, hatch=hatch,
                              edgecolor="white", linewidth=0.8, label=label)
                if hatch:
                    for bar in bars:
                        bar.set_edgecolor("#9a9a9a")
                        bar.set_linewidth(0.4)
                handles.setdefault(label, bars)
                if label == "verified arrival":
                    for xi, v, b0 in zip(x, vals, bottom):
                        if v > 0:
                            ax.text(xi, b0 + v / 2, f"{int(v)}", ha="center", va="center",
                                    color="white", fontsize=8.5, fontweight="bold")
                bottom += vals
            ax.set_xticks(x)
            ax.set_xticklabels([f"{b:g}" for b in budgets])
            ax.set_xlabel("exploration budget (simulated minutes)")
            ax.set_title(name, loc="left", fontweight="bold")
            ax.set_xlim(-0.55, n - 0.45)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            ax.spines["left"].set_color("#777777")
            ax.spines["bottom"].set_color("#777777")
            ax.tick_params(colors="#333333", length=2.5)
            ax.grid(axis="y", color="#dddddd", linewidth=0.5)
            ax.set_axisbelow(True)
        if per_budget:
            axes[0].set_ylim(0, per_budget)
            axes[0].set_yticks(np.linspace(0, per_budget, 5))
        axes[0].set_ylabel(f"goals per budget (of {per_budget})" if per_budget else "goals")
        order = [g[0] for g in GROUPS if g[0] in handles]
        fig.legend([handles[k] for k in order], order, loc="lower center", ncol=4, frameon=False,
                   handlelength=1.3, handleheight=0.9, columnspacing=1.0, handletextpad=0.5,
                   bbox_to_anchor=(0.5, 0.0))
        fig.tight_layout(rect=(0, 0.14, 1, 1), w_pad=1.5)
    return fig


def plot(counts, budgets, out):
    import matplotlib.pyplot as plt
    fig = outcome_figure(counts, budgets)
    fig.savefig(os.path.join(out, "outcomes_by_budget.png"), dpi=150, bbox_inches="tight", pad_inches=0.03)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--aerial", default=os.path.join(ROOT, "runs", "raw", "batch"))
    ap.add_argument("--wheeled", "--wheeled-dir", dest="wheeled", default=os.path.join(ROOT, "runs", "raw", "wbatch3"),
                    help="the wheeled arm on the shared A* executive")
    ap.add_argument("--wheeled-nav2", default=os.path.join(ROOT, "runs", "raw", "wbatch2"),
                    help="the wheeled reference arm on Nav2 ('' leaves it out)")
    ap.add_argument("--budgets", default="1,2.5,5,10")
    ap.add_argument("--goals", type=int, default=6, help="each tour's first N goals (default 6; 0 = all)")
    ap.add_argument("--out", default=os.path.join(ROOT, "analysis", "results"))
    a = ap.parse_args()
    budgets = [float(b) for b in a.budgets.split(",")]
    os.makedirs(a.out, exist_ok=True)
    goals, contacts, counts = [], [], {}
    for arm, d in (("aerial", a.aerial), ("wheeled", a.wheeled), ("wheeled_nav2", a.wheeled_nav2)):
        if not d:
            continue
        if not os.path.isdir(d):
            print(f"{arm}: no batch directory {d}")
            continue
        for seed in sa.WORLDS:
            ids = sa.goal_order(seed)
            ids = ids[:a.goals] if a.goals else ids
            geo = cf.geometry(seed) if arm in WHEELED else None
            for b in budgets:
                td = sa.final_try(os.path.join(d, seed, f"tour_b{b:g}"))
                rows, contact = tour_outcomes(arm, seed, b, td, ids, geo)
                goals += rows
                counts.setdefault((arm, b), Counter()).update(r["outcome"] for r in rows)
                if contact:
                    contacts.append(contact)
    if not goals:                                 # no run data: leave the committed tables and figure alone
        print("no batch data found (runs/raw/ is not part of the repository); nothing written")
        return 1
    with open(os.path.join(a.out, "goal_outcomes.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(goals[0].keys()))
        w.writeheader()
        w.writerows(goals)
    agg = [dict(arm=arm, budget_min=b, outcome=cls, goals=counts[(arm, b)].get(cls, 0))
           for (arm, b) in sorted(counts) for cls in CLASSES[arm] + ["missing"]
           if counts[(arm, b)].get(cls, 0)]
    with open(os.path.join(a.out, "outcomes_by_budget.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["arm", "budget_min", "outcome", "goals"])
        w.writeheader()
        w.writerows(agg)
    if contacts:
        with open(os.path.join(a.out, "wheeled_contact.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(contacts[0].keys()))
            w.writeheader()
            w.writerows(contacts)
    plot(counts, budgets, a.out)
    for arm in CLASSES:
        cls = CLASSES[arm]
        if not any((arm, b) in counts for b in budgets):
            continue
        print(f"{arm:12s} {'budget':>6s} " + " ".join(f"{c:>8s}" for c in cls) + f" {'goals':>6s}")
        for b in budgets:
            c = counts.get((arm, b), Counter())
            print(f"{'':12s} {b:6g} " + " ".join(f"{c.get(k, 0):8d}" for k in cls) + f" {sum(c.values()):6d}")
        tot = Counter()
        for b in budgets:
            tot.update(counts.get((arm, b), Counter()))
        print(f"{'':12s} {'all':>6s} " + " ".join(f"{tot.get(k, 0):8d}" for k in cls) + f" {sum(tot.values()):6d}")
    for arm in WHEELED:
        for b in budgets:
            cs = [c for c in contacts if c["arm"] == arm and c["budget_min"] == b and c["contact_episodes"] != ""]
            if not cs:
                continue
            print(f"{arm} contact b{b:g}: {sum(c['contact_episodes'] for c in cs)} episodes, "
                  f"{sum(c['contact_s'] for c in cs):.1f} sim-s, in {sum(1 for c in cs if c['contact_episodes'])} "
                  f"of {len(cs)} tours")
    miss = [r for r in goals if r["outcome"] == "missing"]
    if miss:
        print("missing (no FINAL try): " + ", ".join(sorted({f"{r['arm']}/{r['world']}/b{r['budget_min']:g}" for r in miss})))
    print(f"-> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
