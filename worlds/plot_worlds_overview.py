#!/usr/bin/env python3
"""Overview montage + statistics of v1's 25 candidate worlds (seeds 20260723001..025;
v1 = the predecessor project, sim-nav-benchmark).

Reads the predecessor's run data (not included in this repository): the ground-truth
manifests (runs/world_<seed>.csv), the frozen v1 SLAM maps (world_<seed>.pgm) and v1's
mission-result JSONs, and writes:
  worlds/v1_worlds_overview.png  - 5x5 top-down montage, door widths annotated
  worlds/v1_worlds_stats.csv     - one row per world (door widths, spawn room,
                                   room reachability vs footprint, clutter, v1 results)
Decision support for the fixed 10-world list. Pure analysis:
nothing here changes a world.

Usage (WSL or Windows python with numpy + matplotlib):
  python3 worlds/plot_worlds_overview.py [--runs <the predecessor's runs folder>]
"""
import argparse, csv, json, math, os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

ARENA = 10.0          # half-size of the 20x20 m arena
# door -> rooms it connects (v1's generate_world.py MODE="quad")
CONN = {"S": ("SW", "SE"), "N": ("NW", "NE"), "W": ("SW", "NW"), "E": ("SE", "NE")}
# Footprint thresholds (true door width, m): wheeled 0.42 m body + v1 margins;
# Iris spinning envelope 0.52 m (sideways) / 0.70 m (nose-first), same margins.
THRESHOLDS = (0.66, 0.75, 0.85)


def read_manifest(path):
    doors, obs, pxw, pyw = [], [], None, None
    for r in csv.reader(open(path)):
        if not r or r[0].startswith("#") or r[0] == "type":
            continue
        t, name, x, y, p1, p2, yaw = r[0], r[1], float(r[2]), float(r[3]), r[4], r[5], float(r[6])
        if t == "door":
            seg, tag = name.split("_")[1], name.split("_")[2]
            doors.append(dict(seg=seg, tag=tag, x=x, y=y, w=float(p1), axis=p2))
            pxw, pyw = (x, pyw) if p2 == "x" else (pxw, y)
        elif t == "box":
            obs.append(dict(kind="box", name=name, x=x, y=y, sx=float(p1), sy=float(p2), yaw=yaw))
        elif t == "cyl":
            obs.append(dict(kind="cyl", name=name, x=x, y=y, r=float(p1)))
    return doors, obs, pxw, pyw


def read_pgm(path):
    with open(path, "rb") as f:
        assert f.readline().strip() == b"P5"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        f.readline()
        data = f.read()
    return w, h, data


def rooms_reachable(spawn, doors, thr):
    seen, changed = {spawn}, True
    while changed:
        changed = False
        for d in doors:
            a, b = CONN[d["seg"]]
            if d["w"] >= thr and ((a in seen) != (b in seen)):
                seen |= {a, b}
                changed = True
    return len(seen)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", default=os.environ.get("V1_RUNS"),
                    help="the predecessor's runs folder (default: $V1_RUNS)")
    ap.add_argument("--out", default=os.path.dirname(os.path.abspath(__file__)))
    a = ap.parse_args()
    if not a.runs:
        ap.error("give the predecessor's runs folder with --runs or V1_RUNS")
    seeds = [f"20260723{i:03d}" for i in range(1, 26)]
    feas = {w["seed"]: w for w in json.load(open(f"{a.runs}/missions_feasibility.json"))}
    v8 = {w["seed"]: w for w in json.load(open(f"{a.runs}/missions_results_v8.json"))}
    v9 = {w["seed"]: w for w in json.load(open(f"{a.runs}/missions_results_v9.json"))}

    rows = []
    fig, axes = plt.subplots(5, 5, figsize=(20, 20))
    for ax, s in zip(axes.flat, seeds):
        doors, obs, pxw, pyw = read_manifest(f"{a.runs}/world_{s}.csv")
        spawn = ("S" if pyw > 0 else "N") + ("W" if pxw > 0 else "E")
        area = sum(o["sx"] * o["sy"] if o["kind"] == "box" else math.pi * o["r"] ** 2
                   for o in obs if not o["name"].startswith("partition"))
        _, _, data = read_pgm(f"{a.runs}/world_{s}.pgm")
        unk = sum(1 for b in data if 50 <= b <= 250) / len(data)
        sd = [d for d in doors if spawn in CONN[d["seg"]]]
        r = dict(seed=s, spawn_room=spawn, pxw=pxw, pyw=pyw,
                 door_S=next((d["w"] for d in doors if d["seg"] == "S"), None),
                 door_N=next((d["w"] for d in doors if d["seg"] == "N"), None),
                 door_W=next((d["w"] for d in doors if d["seg"] == "W"), None),
                 door_E=next((d["w"] for d in doors if d["seg"] == "E"), None),
                 anchors="".join(d["seg"] for d in doors if d["tag"] == "A"),
                 spawn_exit_min=min(d["w"] for d in sd), spawn_exit_max=max(d["w"] for d in sd),
                 rooms_at_0_66=rooms_reachable(spawn, doors, 0.66),
                 rooms_at_0_75=rooms_reachable(spawn, doors, 0.75),
                 rooms_at_0_85=rooms_reachable(spawn, doors, 0.85),
                 clutter_pct=round(100 * area / 400, 1),
                 n_obstacles=sum(1 for o in obs if not o["name"].startswith("partition")),
                 v1_map_unknown_pct=round(100 * unk, 1),
                 v1_wall_coverage_pct=v9[s]["coverage"],
                 v1_goals_truth_feasible=sum(g["truth_ok"] for g in feas[s]["goals"].values()),
                 v1_v8_status=v8[s]["status"], v1_v8_real=v8[s]["real_succ"], v1_v8_fake=v8[s]["fake_succ"],
                 v1_v9_status=v9[s]["status"], v1_v9_real=v9[s]["real_succ"], v1_v9_fake=v9[s]["fake_succ"])
        rows.append(r)

        # ---- draw ----
        ax.set_xlim(-ARENA, ARENA); ax.set_ylim(-ARENA, ARENA); ax.set_aspect("equal")
        ax.set_xticks([]); ax.set_yticks([])
        for o in obs:
            if o["kind"] == "box":
                col = "#6b4c7a" if o["name"].startswith("partition") else "#9a9a9a"
                rect = Rectangle((-o["sx"] / 2, -o["sy"] / 2), o["sx"], o["sy"], color=col)
                tr = matplotlib.transforms.Affine2D().rotate_deg(o["yaw"]).translate(o["x"], o["y"])
                rect.set_transform(tr + ax.transData)
                ax.add_patch(rect)
            else:
                ax.add_patch(Circle((o["x"], o["y"]), o["r"], color="#9a9a9a"))
        for d in doors:
            col = "#2a9d3b" if d["w"] >= 0.85 else ("#e0a400" if d["w"] >= 0.72 else "#d62828")
            ax.plot(d["x"], d["y"], "s", color=col, ms=9, mec="k", mew=0.5)
            ax.annotate(f'{d["w"]:.2f}{"*" if d["tag"] == "A" else ""}', (d["x"], d["y"]),
                        xytext=(4, 4), textcoords="offset points", fontsize=8, color=col, fontweight="bold")
        ax.add_patch(Circle((0, 0), 2.0, fill=False, ls="--", lw=0.8, color="#1f77b4"))
        ax.plot(0, 0, "o", color="#1f77b4", ms=6)
        ax.set_title(f'{s[-3:]}  spawn {spawn}  exits {r["spawn_exit_min"]:.2f}/{r["spawn_exit_max"]:.2f}'
                     f'  rooms@0.75={r["rooms_at_0_75"]}  v1:{r["v1_v9_status"]}', fontsize=9)
    fig.suptitle("v1 worlds 20260723001-025 (ground-truth manifests). Doors: red <0.72 m, amber 0.72-0.85, green >=0.85; "
                 "* = anchor door. Blue: spawn (0,0) + 2 m clear zone.", fontsize=12)
    fig.tight_layout(rect=(0, 0, 1, 0.98))
    png = os.path.join(a.out, "v1_worlds_overview.png")
    fig.savefig(png, dpi=110)
    out_csv = os.path.join(a.out, "v1_worlds_stats.csv")
    with open(out_csv, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    print(f"wrote {png}\nwrote {out_csv}")


if __name__ == "__main__":
    main()
