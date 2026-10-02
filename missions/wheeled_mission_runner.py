#!/usr/bin/env python3
"""The wheeled baseline's mission tour on a frozen map (the wheeled arm of the benchmark).

The mover and the localizer are the baseline's own: Nav2 (policies/nav2/nav2_params.yaml,
the predecessor's v9 Nav2 configuration + corrections) with AMCL on the frozen map, brought up by missions/wheeled_nav.sh tour <map>.
This runner only sends the goals, as the baseline's mission_runner.py did, with the
protocol the aerial arm flies: the world's goals in file order
(missions/goals/goals_<seed>.csv, the same goals as the quadrotor), one NavigateToPose per
attempt, the claim = Nav2's SUCCEEDED (its goal checker: 0.35 m, yaw free), a timeout of
--timeout SIM seconds per attempt (the goal is cancelled), retry once after --retry-wait
sim seconds, --breather sim seconds between goals. A rejection at send is a lifecycle race,
not a result: retried inside the attempt for up to 60 wall-s (the baseline's v4 lesson).
The clock is the Isaac run's /aeb/sim_time; the readiness gates (sim clock, Nav2, the first
AMCL pose, the belief TF) come before the policy clock and are not counted.

Outputs (the aerial executive's formats, so analysis/verify_missions.py and plot_tour.py
read them): missions_<seed>_<ts>.csv (one row per attempt), manifest_<seed>_<ts>.json,
belief_<seed>_<ts>.csv (TF map->base_link + the wheel odometry), tour_<seed>_<ts>.log.

  python3 missions/wheeled_mission_runner.py --seed 20260723008 --map <map_b5.yaml> [--out-dir DIR]
"""
import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wheeled_io as wio  # noqa: E402

ROOT = wio.ROOT
COLS = ["mission", "goal_x", "goal_y", "attempt", "result", "wall_s", "sim_s", "epoch_start", "epoch_end",
        "sim_start", "sim_end", "claim_x", "claim_y", "path_m", "reason", "odom_x", "odom_y", "recoveries"]


class SimStalled(Exception):
    pass


def read_goals(path):
    rows = [r for r in csv.DictReader(ln for ln in open(path) if not ln.startswith("#"))]
    return [(r["mission"], float(r["x"]), float(r["y"]), r.get("room", "")) for r in rows]


class Tour:
    def __init__(self, a, io, log):
        self.a, self.io, self.log = a, io, log
        self.goals = read_goals(a.goals)[: a.tour_limit or None]
        self.rl = wio.RunLog(a.seed, a.run_id, a.out_dir, log, {k: v for k, v in vars(a).items()}, "tour")
        self.rl.io = io
        self.rl.manifest["map"] = os.path.relpath(a.map, ROOT) if a.map else None
        self.rl.manifest["goals"] = os.path.relpath(a.goals, ROOT)
        self.csv_path = os.path.join(a.out_dir, f"missions_{a.seed}_{a.run_id}.csv")

    def check_sim(self):
        if self.io.sim_age() > self.a.sim_stale_s:
            raise SimStalled()

    def run_goal(self, gx, gy):
        """One attempt: (result, reason, extra)."""
        a, io = self.a, self.io
        s0, w0 = io.sim_t(), time.time()
        g, t_send = io.send(gx, gy), time.time()
        while True:
            st = g.poll()
            self.rl.belief()
            if st == "rejected":
                if time.time() - t_send > 60.0:
                    return "REJECTED", "rejected", {}
                self.log("[exec]    (goal rejected - Nav2 not active yet, resend in 5 s)")
                time.sleep(5.0)
                g = io.send(gx, gy)
                continue
            if st not in ("pending", "active"):
                p = io.pose()
                reason = {"SUCCEEDED": "claimed", "CANCELED": "canceled"}.get(st, "aborted")
                return st, (reason + (f" {g.error}" if g.error else "")), self.extra(p, g)
            el = io.sim_t() - s0
            if el >= a.timeout:
                g.cancel()
                return "TIMEOUT", "timeout", self.extra(io.pose(), g)
            if time.time() - w0 > a.timeout / a.min_rtf:
                g.cancel()
                return "TIMEOUT", "wall_cap", self.extra(io.pose(), g)
            if io.sim_age() > a.sim_stale_s:
                g.cancel()
                raise SimStalled()
            time.sleep(0.05)

    @staticmethod
    def extra(p, g):
        return dict(claim_x=p[0] if p else float("nan"), claim_y=p[1] if p else float("nan"),
                    path_m=g.path_m, recoveries=g.recoveries)

    def tour(self):
        a, io, rl = self.a, self.io, self.rl
        f = open(self.csv_path, "w", newline="")
        w = csv.writer(f)
        w.writerow(COLS)
        f.flush()
        t0 = dict(wall=time.time(), sim=io.sim_t())
        rl.event("policy_clock_start", t0_sim=round(t0["sim"], 3))
        ok = first = 0
        try:
            for name, gx, gy, room in self.goals:
                result = None
                for attempt in (1, 2):
                    self.log(f"[exec] -> {name} {room} (attempt {attempt}): ({gx:.2f}, {gy:.2f})")
                    e0, s0 = time.time(), io.sim_t()
                    result, reason, extra = self.run_goal(gx, gy)
                    e1, s1 = time.time(), io.sim_t()
                    o = io.odom()
                    self.log(f"[exec]    {result} ({reason}) in {e1 - e0:.0f}s wall / {s1 - s0:.0f}s sim")
                    row = [name, f"{gx:.2f}", f"{gy:.2f}", attempt, result, f"{e1 - e0:.1f}", f"{s1 - s0:.1f}",
                           f"{e0:.3f}", f"{e1:.3f}", f"{s0:.3f}", f"{s1:.3f}",
                           f"{extra.get('claim_x', float('nan')):.3f}", f"{extra.get('claim_y', float('nan')):.3f}",
                           f"{extra.get('path_m', float('nan')):.2f}", reason,
                           f"{o[0]:.3f}" if o else "", f"{o[1]:.3f}" if o else "", extra.get("recoveries", "")]
                    w.writerow(row)
                    f.flush()
                    rl.manifest["results"].append(dict(zip(COLS, row)))
                    if result == "SUCCEEDED":
                        first += attempt == 1
                        break
                    if attempt == 1:
                        wio.wait_sim(io, a.retry_wait, cap_wall_s=a.retry_wait / a.min_rtf, tick=rl.belief)
                ok += result == "SUCCEEDED"
                wio.wait_sim(io, a.breather, cap_wall_s=a.breather / a.min_rtf, tick=rl.belief)
                self.check_sim()
        finally:
            f.close()
            rl.manifest["summary"] = dict(succeeded=ok, first_try=first, goals=len(self.goals),
                                          t_sim=round((io.sim_t() or t0["sim"]) - t0["sim"], 2))
        self.log(f"[exec] TOUR DONE: {ok}/{len(self.goals)} SUCCEEDED ({first} first-try) -> {self.csv_path}")

    def run(self):
        status = "OK"
        try:
            bad = wio.gates(self.io, self.rl, need_map=False, need_amcl=True,
                            wait_wall_s=self.a.gate_wall_s, stale_s=self.a.sim_stale_s)
            if bad:
                status = bad
                return 2
            self.tour()
        except SimStalled:
            self.rl.event("SIM_STALLED", age_wall_s=round(self.io.sim_age(), 1))
            status = "SIM_STALLED"
        except KeyboardInterrupt:
            status = "INTERRUPTED"
        except Exception as exc:  # noqa: BLE001
            import traceback
            self.log(f"[exec] ERROR {exc}\n{traceback.format_exc()}")
            status = f"ERROR: {exc}"
        finally:
            self.rl.save(status)
            self.io.close()
        return 0 if status == "OK" else 1


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", required=True)
    ap.add_argument("--map", default=None, help="the frozen map this tour runs on (recorded in the manifest; "
                    "Nav2 loads it through wheeled_nav.sh)")
    ap.add_argument("--goals", default=None, help="goal CSV (default missions/goals/goals_<seed>.csv)")
    ap.add_argument("--tour-limit", type=int, default=0, help="only the first N goals (tests)")
    ap.add_argument("--timeout", type=float, default=240.0, help="per-attempt timeout, sim seconds")
    ap.add_argument("--retry-wait", type=float, default=3.0, help="sim seconds before the second attempt")
    ap.add_argument("--breather", type=float, default=1.0, help="sim seconds between goals")
    ap.add_argument("--min-rtf", type=float, default=0.05, help="wall caps = sim durations / this")
    ap.add_argument("--sim-stale-s", type=float, default=60.0, help="wall seconds without /aeb/sim_time = the simulator is gone")
    ap.add_argument("--gate-wall-s", type=float, default=180.0, help="wall seconds for the readiness gates")
    ap.add_argument("--out-dir", default=None, help="default runs/raw/tour_<seed>_<ts>/")
    return ap


def main():
    wio.install_stop_signals()
    a = build_parser().parse_args()
    a.run_id = time.strftime("%Y%m%d_%H%M%S")
    a.goals = a.goals or os.path.join(ROOT, "missions", "goals", f"goals_{a.seed}.csv")
    a.out_dir = a.out_dir or os.path.join(ROOT, "runs", "raw", f"tour_{a.seed}_{a.run_id}")
    os.makedirs(a.out_dir, exist_ok=True)
    log = wio.make_logger(os.path.join(a.out_dir, f"tour_{a.seed}_{a.run_id}.log"))
    log(f"[exec] wheeled tour world {a.seed} map={a.map} goals={a.goals} timeout={a.timeout:g} sim-s -> {a.out_dir}")
    io = wio.WheeledIO(log, node_name="wheeled_mission_runner", amcl_topic="/amcl_pose")
    return Tour(a, io, log).run()


if __name__ == "__main__":
    wio.exit_now(main())
