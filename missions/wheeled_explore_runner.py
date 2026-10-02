#!/usr/bin/env python3
"""FRONTIER3D exploration of the wheeled baseline: the phase that produces the
wheeled arm's frozen maps.

The mapper is slam_toolbox (online_async, policies/slam/slam_params.yaml; the same mapper,
lidar model, resolution and range as the quadrotor) on the baseline's raycast lidar
and noisy wheel odometry. The policy is the shared frontier rule (missions/frontier.py)
with the aerial explorer's parameters: nearest cluster by path length on the current
map (unknown impassable, --unknown-margin 0.2 m of unknown kept off the measured path),
clusters >= 10 cells, goal = centroid pulled into free space, every attempted goal
blacklisted within 0.5 m. The embodiment enters only through the planner radius (--radius
0.30 m, the baseline's global costmap) and the passage floor (--min-passage 0.60 m: the
baseline's planner seals doors below ~0.60 m). The mover is the baseline's Nav2
(policies/nav2/nav2_params.yaml: the predecessor's v9 Nav2 configuration + corrections) with its global costmap on the live map:
NavigateToPose per frontier, reached when the belief (TF map->base_link, slam_toolbox's
correction over the wheel odometry) is within --claim-tol 0.6 m of the goal or Nav2 reports
SUCCEEDED; --timeout 90 sim-s per frontier; a breather of --breather 1.0 sim-s between
frontiers (the aerial explorer's values).

Budget grid {1, 2.5, 5, 10} sim-minutes, nested: one run, the map frozen at each
value (sim time since the policy clock, /aeb/sim_time) from slam_toolbox's latest /map as
map_server trinary PGM/YAML; the run ends at the last budget (the goal in progress is
cancelled) or when no frontier is left (`complete`) or reachable (`exhausted`); the
remaining checkpoints then get the final map. The readiness gates (sim clock, Nav2,
the first map, the belief TF) and the dart precede the policy clock: the drone's
dart for the wheeled robot, 1.5 m along +x and back at 0.25 m/s on its odometry (/cmd_vel, Nav2
idle); slam_toolbox integrates scans only on motion, and a robot standing still keeps the first
scan's few square metres as its map (smoke test 2026-09-26).

Outputs (the aerial explorer's formats: analysis/plot_explore.py reads them), in --out-dir:
map_b1.{pgm,yaml} ... map_b10.*, frontiers_<seed>_<ts>.csv, manifest_<seed>_<ts>.json
(checkpoints, budgets_min, events, summary), belief_<seed>_<ts>.csv, explore_<seed>_<ts>.log.

  python3 missions/wheeled_explore_runner.py --seed 20260723008 [--budgets 1,2.5,5,10] [--out-dir DIR]
(Isaac through sim/launch_wheeled.ps1 and missions/wheeled_nav.sh <seed> explore running.)
"""
import argparse
import csv
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import wheeled_io as wio  # noqa: E402
import frontier  # noqa: E402
from gridmap import GridMap  # noqa: E402

ROOT = wio.ROOT
FRONTIER_COLS = ["frontier", "goal_x", "goal_y", "path_m", "size", "clusters", "result", "reason",
                 "wall_s", "sim_s", "epoch_start", "epoch_end", "sim_start", "sim_end", "coverage_m2"]


class SimStalled(Exception):
    pass


class Explorer:
    def __init__(self, a, io, log):
        self.a, self.io, self.log = a, io, log
        self.budgets = sorted(a.budgets)
        self.rl = wio.RunLog(a.seed, a.run_id, a.out_dir, log, {k: v for k, v in vars(a).items()}, "explore")
        self.rl.io = io
        self.rl.manifest["budgets_min"] = self.budgets
        self.rl.manifest["checkpoints"] = []
        self.csv_path = os.path.join(a.out_dir, f"frontiers_{a.seed}_{a.run_id}.csv")
        self.gm = None
        self.last_map_count = -1
        self.cp_next = 0
        self.cp_stop = False
        self.cp_lock = threading.Lock()
        self.t0 = None

    # ------------------------------------------------------------ clock, map
    def elapsed_min(self):
        return ((self.io.sim_t() or self.t0["sim"]) - self.t0["sim"]) / 60.0

    def refresh_map(self):
        """Rebuild self.gm from the mapper's latest /map when it changed. True if a map exists."""
        msg, n = self.io.latest_map()
        if msg is None:
            return False
        if n != self.last_map_count:
            info = msg.info
            self.gm = GridMap.from_occupancy(msg.data, info.width, info.height, info.resolution,
                                             (info.origin.position.x, info.origin.position.y),
                                             unknown_margin=self.a.unknown_margin)
            self.last_map_count = n
        return True

    def checkpoint(self, budget_min, final=False):
        """Freeze the mapper's latest map as map_b<budget>.{pgm,yaml} (map_server trinary)."""
        msg, _n = self.io.latest_map()
        if msg is None:
            self.rl.event("checkpoint_no_map", budget_min=budget_min)
            return
        info = msg.info
        gm = GridMap.from_occupancy(msg.data, info.width, info.height, info.resolution,
                                    (info.origin.position.x, info.origin.position.y))
        path = gm.save(os.path.join(self.a.out_dir, f"map_b{budget_min:g}"))
        cl = frontier.clusters(gm, self.a.radius, self.a.min_cells, self.a.min_passage)
        rec = dict(budget_min=budget_min, t_sim=round((self.io.sim_t() or 0.0) - self.t0["sim"], 2),
                   map=os.path.relpath(path, ROOT), coverage_m2=round(gm.known_free_area(), 2), frontiers=len(cl),
                   final=final, size=[gm.w, gm.h], origin=[round(gm.origin[0], 3), round(gm.origin[1], 3)])
        with self.cp_lock:
            self.rl.manifest["checkpoints"].append(rec)
        self.rl.event("checkpoint", **rec)

    def _cp_thread(self):
        """Takes the checkpoints at the exact sim time, whatever the mover is doing."""
        while not self.cp_stop:
            if self.cp_next < len(self.budgets) and self.elapsed_min() >= self.budgets[self.cp_next]:
                self.checkpoint(self.budgets[self.cp_next])
                self.cp_next += 1
            time.sleep(0.2)

    # ------------------------------------------------------------ one frontier
    def go(self, gx, gy):
        """Drive to one frontier goal with Nav2: (result, reason)."""
        a, io = self.a, self.io
        s0, w0 = io.sim_t(), time.time()
        g, t_send = io.send(gx, gy), time.time()
        while True:
            st = g.poll()
            self.rl.belief()
            if st == "rejected":
                if time.time() - t_send > 60.0:
                    return "REJECTED", "rejected"
                self.log("[exec]    (goal rejected - Nav2 not active yet, resend in 5 s)")
                time.sleep(5.0)
                g = io.send(gx, gy)
                continue
            if st not in ("pending", "active"):
                reason = {"SUCCEEDED": "nav2_succeeded", "CANCELED": "canceled"}.get(st, "aborted")
                return st, reason + (f" {g.error}" if g.error else "")
            p = io.pose()
            if p is not None and math.hypot(p[0] - gx, p[1] - gy) <= a.claim_tol:
                g.cancel()
                return "SUCCEEDED", "claimed"
            if io.sim_t() - s0 >= a.timeout or time.time() - w0 > a.timeout / a.min_rtf:
                g.cancel()
                return "TIMEOUT", "timeout"
            if self.elapsed_min() >= self.budgets[-1]:
                g.cancel()
                return "BUDGET_END", "budget_end"
            if io.sim_age() > a.sim_stale_s:
                g.cancel()
                raise SimStalled()
            time.sleep(0.05)

    # ------------------------------------------------------------ the dart
    def stop(self, sim_s=0.5):
        """Zero velocity for sim_s sim seconds (repeated: the OmniGraph holds the last command)."""
        io = self.io
        s0, w0 = io.sim_t(), time.time()
        while io.sim_t() - s0 < sim_s and time.time() - w0 < sim_s / self.a.min_rtf:
            io.cmd(0.0, 0.0)
            time.sleep(0.05)
        io.cmd(0.0, 0.0)

    def dart(self):
        """The aerial protocol's dart for the wheeled robot, before the policy clock (not counted):
        --dart m along the start heading and back at --dart-speed on the robot's own
        odometry, inside the generator's 2 m clear zone (1.5 + 0.25 < 2.0, the body's corners), with Nav2 idle.
        slam_toolbox adds a scan only after 0.2 m / 0.2 rad of motion and marks a cell only
        after two rays crossed it: standing still, its map stays the first scan's 3.7 m² star
        and the frontier rule has nothing to work with (smoke test 2026-09-26)."""
        a, io, rl = self.a, self.io, self.rl
        o0 = io.odom()
        if o0 is None:
            rl.event("dart_skipped", reason="no odometry")
            return False
        _msg, n0 = io.latest_map()
        s_start = io.sim_t()
        h0 = o0[2]
        along = (lambda o: (o[0] - o0[0]) * math.cos(h0) + (o[1] - o0[1]) * math.sin(h0))
        cap = 3.0 * a.dart / a.dart_speed                       # sim seconds per leg
        rl.event("dart_start", dart_m=a.dart, speed=a.dart_speed,
                 coverage_m2=round(self.gm.known_free_area(), 2) if self.gm else None)
        ok = True
        for name, sign, done in (("dart_out", +1.0, lambda o: along(o) >= a.dart),
                                 ("dart_back", -1.0, lambda o: along(o) <= 0.0)):
            s0, w0 = io.sim_t(), time.time()
            while True:
                o = io.odom()
                if o is not None and done(o):
                    break
                if io.sim_t() - s0 > cap or time.time() - w0 > cap / a.min_rtf:
                    ok = False
                    break
                io.cmd(sign * a.dart_speed, 0.0)
                rl.belief()
                time.sleep(0.05)
            self.stop(0.5)
            o = io.odom()
            rl.event(name, along_m=round(along(o), 3) if o else None, sim_s=round(io.sim_t() - s0, 2), ok=ok)
            if not ok:
                break
        wio.wait_sim(io, a.breather, cap_wall_s=a.breather / a.min_rtf, tick=rl.belief)
        t_w = time.time()                                        # slam_toolbox's next map, with the dart in it
        while io.latest_map()[1] <= n0 and time.time() - t_w < 10.0:
            time.sleep(0.2)
        self.refresh_map()
        rl.event("dart_done", ok=ok, sim_s=round(io.sim_t() - s_start, 2), coverage_m2=round(self.gm.known_free_area(), 2))
        return ok

    # ------------------------------------------------------------ the loop
    def explore(self):
        a, io, rl = self.a, self.io, self.rl
        f = open(self.csv_path, "w", newline="")
        w = csv.writer(f)
        w.writerow(FRONTIER_COLS)
        f.flush()
        self.refresh_map()
        if a.dart > 0:
            self.dart()                                          # before the policy clock, not counted
        self.t0 = dict(wall=time.time(), sim=io.sim_t())
        rl.event("policy_clock_start", t0_sim=round(self.t0["sim"], 3), coverage_m2=round(self.gm.known_free_area(), 2))
        cp = threading.Thread(target=self._cp_thread, daemon=True)
        cp.start()
        blacklist = []
        k = 0
        reason_done = None
        try:
            while True:
                if self.elapsed_min() >= self.budgets[-1]:
                    reason_done = "budget"
                    break
                if io.sim_age() > a.sim_stale_s:
                    raise SimStalled()
                self.refresh_map()
                pose = io.pose()
                if pose is None:
                    time.sleep(0.1)
                    continue
                cl = frontier.clusters(self.gm, a.radius, a.min_cells, a.min_passage)
                best, plen = frontier.choose(self.gm, (pose[0], pose[1]), cl, blacklist, a.radius, a.blacklist_m)
                if best is None:
                    reason_done = "complete" if not cl else "exhausted"
                    break
                k += 1
                gx, gy = best["goal"]
                rl.event("frontier_selected", k=k, x=round(gx, 2), y=round(gy, 2), path_m=round(plen, 1),
                         size=best["size"], clusters=len(cl), coverage_m2=round(self.gm.known_free_area(), 1))
                e0, s0 = time.time(), io.sim_t()
                result, reason = self.go(gx, gy)
                e1, s1 = time.time(), io.sim_t()
                self.refresh_map()
                w.writerow([f"f{k:02d}", f"{gx:.2f}", f"{gy:.2f}", f"{plen:.1f}", best["size"], len(cl), result, reason,
                            f"{e1 - e0:.1f}", f"{s1 - s0:.1f}", f"{e0:.3f}", f"{e1:.3f}", f"{s0:.3f}", f"{s1:.3f}",
                            f"{self.gm.known_free_area():.1f}"])
                f.flush()
                rl.event("frontier_done" if result == "SUCCEEDED" else "frontier_failed", k=k, result=result,
                         reason=reason, sim_s=round(s1 - s0, 1), coverage_m2=round(self.gm.known_free_area(), 1))
                blacklist.append((gx, gy))      # reached or not (the shared rule, frontier.py)
                if result == "BUDGET_END":
                    reason_done = "budget"
                    break
                wio.wait_sim(io, a.breather, cap_wall_s=a.breather / a.min_rtf, tick=rl.belief)
        finally:
            self.cp_stop = True
            cp.join(timeout=2.0)
            f.close()
        t_done = round((io.sim_t() or self.t0["sim"]) - self.t0["sim"], 2)
        while self.cp_next < len(self.budgets):          # the curve is flat beyond completion
            # (a run that reached its last budget only lost the race with the checkpoint thread: not final)
            self.checkpoint(self.budgets[self.cp_next], final=(reason_done != "budget"))
            self.cp_next += 1
        self.refresh_map()
        rl.manifest["summary"] = dict(reason=reason_done, frontiers=k, t_sim=t_done,
                                      coverage_m2=round(self.gm.known_free_area(), 2), blacklisted=len(blacklist))
        rl.event("explore_done", reason=reason_done, t_sim=t_done, frontiers=k,
                 coverage_m2=round(self.gm.known_free_area(), 1))
        self.log(f"[exec] EXPLORATION DONE ({reason_done}) after {t_done:.0f} sim-s, {k} frontiers, "
                 f"coverage {self.gm.known_free_area():.1f} m^2 -> {a.out_dir}")
        return reason_done

    def run(self):
        status = "OK"
        try:
            bad = wio.gates(self.io, self.rl, need_map=True, need_amcl=False,
                            wait_wall_s=self.a.gate_wall_s, stale_s=self.a.sim_stale_s)
            if bad:
                status = bad
                return 2
            self.explore()
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
            self.cp_stop = True                           # (an interrupted run keeps only the maps it froze)
            self.rl.save(status)
            self.io.close()
        return 0 if status == "OK" else 1


def build_parser():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", required=True)
    ap.add_argument("--budgets", default="1,2.5,5,10", help="nested budget grid, sim-minutes")
    ap.add_argument("--out-dir", default=None, help="default runs/raw/wexplore_<seed>_<ts>/")
    ap.add_argument("--min-cells", type=int, default=10, help="minimum frontier cluster, cells (0.5 m at 0.05 m/px)")
    ap.add_argument("--blacklist-m", type=float, default=0.5)
    ap.add_argument("--radius", type=float, default=0.30, help="planner radius of the embodiment (the baseline's global costmap)")
    ap.add_argument("--min-passage", type=float, default=0.60,
                    help="frontier goals in slots between known obstacles narrower than this are skipped (the "
                         "baseline's planner seals doors below ~0.60 m)")
    ap.add_argument("--unknown-margin", type=float, default=0.2,
                    help="metres of unknown kept off the measured path (the aerial explorer's value)")
    ap.add_argument("--claim-tol", type=float, default=0.6, help="a frontier counts as reached within this radius")
    ap.add_argument("--timeout", type=float, default=90.0, help="per-frontier timeout, sim seconds")
    ap.add_argument("--breather", type=float, default=1.0, help="sim seconds between frontiers")
    ap.add_argument("--dart", type=float, default=1.5,
                    help="metres along the start heading and back before the policy clock (the aerial dart); 0 = none")
    ap.add_argument("--dart-speed", type=float, default=0.25, help="m/s of the dart (sim time)")
    ap.add_argument("--min-rtf", type=float, default=0.05, help="wall caps = sim durations / this")
    ap.add_argument("--sim-stale-s", type=float, default=60.0, help="wall seconds without /aeb/sim_time = the simulator is gone")
    ap.add_argument("--gate-wall-s", type=float, default=180.0, help="wall seconds for the readiness gates")
    ap.add_argument("--map-topic", default="/map")
    return ap


def main():
    wio.install_stop_signals()
    a = build_parser().parse_args()
    a.budgets = [float(b) for b in a.budgets.split(",")]
    a.run_id = time.strftime("%Y%m%d_%H%M%S")
    a.out_dir = a.out_dir or os.path.join(ROOT, "runs", "raw", f"wexplore_{a.seed}_{a.run_id}")
    os.makedirs(a.out_dir, exist_ok=True)
    log = wio.make_logger(os.path.join(a.out_dir, f"explore_{a.seed}_{a.run_id}.log"))
    log(f"[exec] FRONTIER3D (wheeled) world {a.seed} budgets={a.budgets} min -> {a.out_dir}")
    io = wio.WheeledIO(log, node_name="wheeled_explore_runner", map_topic=a.map_topic)
    return Explorer(a, io, log).run()


if __name__ == "__main__":
    wio.exit_now(main())
