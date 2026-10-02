#!/usr/bin/env python3
"""FRONTIER3D exploration runner for the quadrotor: the phase that produces the
frozen maps the mission phase is scored on.

The mapper is slam_toolbox (online_async, policies/slam/slam_params.yaml) fed by the
drone's raycast lidar and its odometry (rf2o lidar odometry since 2026-09-24; the VIO before):
the same mapper, lidar model, resolution and range as the wheeled baseline. The policy is the shared
frontier rule (missions/frontier.py): nearest frontier cluster by path length on
the current map, clusters >= 10 cells, goal = centroid pulled into free space, failed
goals (and every visited one) blacklisted. The mover is the mission executive's follower (A* on the live map,
LOS carrot, doorway strategy, guards), with slam_toolbox's `/pose` as the localizer
(the same map->odom correction over the odometry pose as AMCL provides in the mission phase).
Vehicle-level filters (from the early test flights): goals in slots narrower than --min-passage are skipped,
an opening measured narrower than that aborts the attempt, and the planning mask keeps
--unknown-margin metres off unknown cells (the wall behind them is not known yet).

Budget grid {1, 2.5, 5, 10} sim-minutes, nested: one flight, the map frozen at
each grid value while the exploration continues, written by this runner from the
mapper's latest /map as map_server-format PGM/YAML (trinary), the format the mission
phase loads. Termination: the last budget, or no frontier >= the minimum size
left (completion time recorded; the remaining checkpoints get the final map). A crash /
odometry loss ends the run as a failure (status in the manifest; the remaining checkpoints
still receive the final map, marked final=true, for the analysis to accept or
reject); the batch continues.

The dart and the gates precede the policy clock; t0 = policy_clock_start.

Outputs: runs/raw/explore_<seed>_<ts>/  map_b1.{pgm,yaml} map_b2.5.* map_b5.* map_b10.*,
  frontiers_<seed>_<ts>.csv (one row per frontier attempt), manifest_<seed>_<ts>.json
  (events: frontier_selected / frontier_done / frontier_failed, checkpoint with coverage,
  explore_done), belief_/vio_/amcl_/imu_excitation_ logs as the mission executive.

Usage (ROS 2 sourced; PX4 SITL, the Isaac A7 app (AEB_CAMERA=0), aerial_odom.sh and
aerial_slam.sh running):
  python3 missions/aerial_explore_runner.py --seed 20260723008 [--budgets 1,2.5,5,10]
"""
import argparse
import csv
import math
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import aerial_mission_runner as amr  # noqa: E402
from gridmap import GridMap  # noqa: E402
import frontier  # noqa: E402

ROOT = amr.ROOT


class Explorer(amr.Executive):
    FATAL = ("odom_lost", "amcl_lost", "odom_diverged", "no_estimate")

    def __init__(self, a, log):
        a.map = None
        a.goals = None
        a.tour_limit = 0
        super().__init__(a, log)
        self.budgets = sorted(a.budgets)
        self.cp_next = 0
        self.cp_stop = False
        self.cp_lock = threading.Lock()
        self.csv_path = os.path.join(a.out_dir, f"frontiers_{self.seed}_{self.run_id}.csv")
        self.manifest["budgets_min"] = self.budgets
        self.manifest["checkpoints"] = []
        self.last_map_count = -1

    # ------------------------------------------------------------ the map
    def refresh_map(self):
        """Rebuild self.gm from the mapper's latest /map when it changed. True if a map exists."""
        msg, n = self.ros.latest_map()
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
        msg, _n = self.ros.latest_map()
        if msg is None:
            self.event("checkpoint_no_map", budget_min=budget_min)
            return
        info = msg.info
        gm = GridMap.from_occupancy(msg.data, info.width, info.height, info.resolution,
                                    (info.origin.position.x, info.origin.position.y))
        label = f"{budget_min:g}"
        path = gm.save(os.path.join(self.a.out_dir, f"map_b{label}"))
        cl = frontier.clusters(gm, self.a.radius, self.a.min_cells, self.a.min_passage)
        rec = dict(budget_min=budget_min, t_sim=round(self.sim_t() - self.t0["sim"], 2), map=os.path.relpath(path, ROOT),
                   coverage_m2=round(gm.known_free_area(), 2), frontiers=len(cl), final=final,
                   size=[gm.w, gm.h], origin=[round(gm.origin[0], 3), round(gm.origin[1], 3)])
        with self.cp_lock:
            self.manifest["checkpoints"].append(rec)
        self.event("checkpoint", **rec)

    def _cp_thread(self):
        """Takes the checkpoints at the exact sim time, whatever the follower is doing."""
        while not self.cp_stop:
            el = (self.sim_t() - self.t0["sim"]) / 60.0
            if self.cp_next < len(self.budgets) and el >= self.budgets[self.cp_next]:
                self.checkpoint(self.budgets[self.cp_next])
                self.cp_next += 1
            time.sleep(0.2)

    # ------------------------------------------------------------ the loop
    def explore(self):
        a = self.a
        f = open(self.csv_path, "w", newline="")
        w = csv.writer(f)
        w.writerow(["frontier", "goal_x", "goal_y", "path_m", "size", "clusters", "result", "reason",
                    "wall_s", "sim_s", "epoch_start", "epoch_end", "sim_start", "sim_end", "coverage_m2"])
        f.flush()
        # wait for the mapper's first map (needs the odometry TF = VIO initialized: done by now)
        t_wait = time.time()
        while not self.refresh_map():
            if time.time() - t_wait > a.map_wait_s:
                self.event("NO_MAP", waited_s=a.map_wait_s)
                f.close()
                return "NO_MAP"
            self.send(self.home, a.alt)
            time.sleep(self.tick_dt)
        self.t0 = dict(wall=time.time(), sim=self.clock())
        self.event("policy_clock_start", t0_sim=round(self.t0["sim"], 3), coverage_m2=round(self.gm.known_free_area(), 2))
        cp = threading.Thread(target=self._cp_thread, daemon=True)
        cp.start()
        blacklist = []
        k = 0
        reason_done = None
        try:
            while True:
                el = (self.sim_t() - self.t0["sim"]) / 60.0
                if el >= self.budgets[-1]:
                    reason_done = "budget"
                    break
                self.refresh_map()
                pose = self.belief.pose()
                if pose is None:
                    self.send(None, a.alt)
                    time.sleep(self.tick_dt)
                    continue
                cl = frontier.clusters(self.gm, a.radius, a.min_cells, a.min_passage)
                best, plen = frontier.choose(self.gm, (pose[0], pose[1]), cl, blacklist, a.radius, a.blacklist_m)
                if best is None:
                    reason_done = "complete" if not cl else "exhausted"
                    break
                k += 1
                gx, gy = best["goal"]
                cov = self.gm.known_free_area()
                self.event("frontier_selected", k=k, x=round(gx, 2), y=round(gy, 2), path_m=round(plen, 1),
                           size=best["size"], clusters=len(cl), coverage_m2=round(cov, 1))
                e0, s0 = time.time(), self.clock()
                result, reason, extra = self.run_goal(f"f{k:02d}", gx, gy, 1)
                e1, s1 = time.time(), self.clock()
                self.refresh_map()
                w.writerow([f"f{k:02d}", f"{gx:.2f}", f"{gy:.2f}", f"{plen:.1f}", best["size"], len(cl), result, reason,
                            f"{e1 - e0:.1f}", f"{s1 - s0:.1f}", f"{e0:.3f}", f"{e1:.3f}", f"{s0:.3f}", f"{s1:.3f}",
                            f"{self.gm.known_free_area():.1f}"])
                f.flush()
                self.event("frontier_done" if result == "SUCCEEDED" else "frontier_failed", k=k, result=result,
                           reason=reason, sim_s=round(s1 - s0, 1), coverage_m2=round(self.gm.known_free_area(), 1))
                # every attempted goal is blacklisted, reached or not: a reached frontier
                # normally dissolves (its unknown side becomes known); one that survives its
                # own visit is a sliver the lidar cannot resolve from there (ray shadows,
                # corners) and would be chosen forever (mock, 2026-09-24)
                blacklist.append((gx, gy))
                if reason in self.FATAL:
                    reason_done = reason
                    break
                if self.impact_limit():
                    reason_done = "impacts"
                    break
                self.settle(a.breather_s, cap_s=a.breather_s + 2.0)     # guarded: a contact here counts
                if self.impact_limit():
                    reason_done = "impacts"
                    break
        finally:
            self.cp_stop = True
            cp.join(timeout=2.0)
            f.close()
        # the remaining checkpoints get the final map (the curve is flat beyond completion)
        t_done = round(self.sim_t() - self.t0["sim"], 2)
        while self.cp_next < len(self.budgets):
            self.checkpoint(self.budgets[self.cp_next], final=True)
            self.cp_next += 1
        self.refresh_map()
        self.manifest["summary"] = dict(reason=reason_done, frontiers=k, t_sim=t_done,
                                        coverage_m2=round(self.gm.known_free_area(), 2), blacklisted=len(blacklist))
        self.event("explore_done", reason=reason_done, t_sim=t_done, frontiers=k,
                   coverage_m2=round(self.gm.known_free_area(), 1))
        self.log(f"[exec] EXPLORATION DONE ({reason_done}) after {t_done:.0f} sim-s, {k} frontiers, "
                 f"coverage {self.gm.known_free_area():.1f} m^2 -> {a.out_dir}")
        return reason_done

    def run(self):
        """The mission executive's run() with explore() in place of tour(); the return flight
        plans on the final map."""
        a = self.a
        status = "OK"
        try:
            self.connect()
            if not self.takeoff():
                status = "TAKEOFF_FAIL"                 # the finally block lands and writes the manifest
                return 3
            gated = False
            for k in (1, 2):
                if k > 1 and not self.relaunch():
                    break
                if not self.dart():
                    self.log("[exec] WARN dart did not settle")
                if self.vio_gate():
                    gated = True
                    break
                self.event("odom_retry", k=k, source=self.odom_source)
            if not gated:
                self.event("ODOM_INIT_FAIL", source=self.odom_source)
                status = "ODOM_INIT_FAIL"                 # the finally block lands and writes the manifest
                return 2
            self.stream(1.0, self.home, a.alt)
            anc = self.belief.anchor()
            self.manifest["anchor"] = anc
            if anc:
                self.event("anchor", theta_deg=round(math.degrees(anc["theta"]), 2), tx=round(anc["tx"], 3), ty=round(anc["ty"], 3))
            if not self.amcl_gate():
                self.event("MAPPER_POSE_FAIL")
                status = "MAPPER_POSE_FAIL"                 # the finally block lands and writes the manifest
                return 4
            reason = self.explore()
            if reason in self.FATAL or reason in ("impacts", "NO_MAP"):
                status = reason.upper() if reason != "impacts" else "IMPACTS"
            self.go_home()
        except KeyboardInterrupt:
            status = "INTERRUPTED"
        except Exception as exc:  # noqa: BLE001
            import traceback
            self.log(f"[exec] ERROR {exc}\n{traceback.format_exc()}")
            status = f"ERROR: {exc}"
        finally:
            self.land()
            self.save_manifest(status)
            self.belief.close()
            self.px4.close()
            self.ros.close()
        return 0 if status == "OK" else 1


def main():
    amr.install_stop_signals()
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seed", required=True)
    ap.add_argument("--budgets", default="1,2.5,5,10", help="nested budget grid, sim-minutes")
    ap.add_argument("--out-dir", default=None, help="default runs/raw/explore_<seed>_<ts>/")
    ap.add_argument("--pose-topic", default="/pose", help="the mapper's corrected pose (slam_toolbox: /pose)")
    ap.add_argument("--map-topic", default="/map")
    ap.add_argument("--map-wait-s", type=float, default=90.0, help="wall seconds to wait for the mapper's first map")
    ap.add_argument("--min-cells", type=int, default=10, help="minimum frontier cluster, cells (0.5 m at 0.05 m/px)")
    ap.add_argument("--blacklist-m", type=float, default=0.5)
    ap.add_argument("--breather-s", type=float, default=1.0, help="hold between frontiers, sim seconds")
    # the follower's parameters (the mission executive's, with exploration defaults)
    ap.add_argument("--estimator", default="amcl", help=argparse.SUPPRESS)            # the mapper's pose plays AMCL's role
    ap.add_argument("--odom-source", choices=["lidar", "vio"], default="lidar",
                    help="lidar = rf2o range-flow odometry from /scan (the benchmark's source); vio = OpenVINS")
    ap.add_argument("--odom-topic", default="/odom_rf2o", help="nav_msgs/Odometry of the lidar odometry (rf2o)")
    ap.add_argument("--vio-topic", default="/poseimu", help="OpenVINS pose topic (--odom-source vio)")
    amr.add_vehicle_args(ap)
    ap.add_argument("--alt", type=float, default=None, help="air 1.3, ground 0")
    ap.add_argument("--radius", type=float, default=None, help="planner radius: air 0.35, ground 0.30")
    ap.add_argument("--unknown", default="wall", choices=["wall"], help="exploration plans through known free space only (unknown impassable)")
    ap.add_argument("--unknown-margin", type=float, default=0.2,
                    help="metres of unknown cells kept off the planned path (the true wall hides in the unknown strip "
                         "beside the known free space: a test flight planned 0.15 m from an unmapped wall); the goal "
                         "tolerance (0.5 m) lets a path end short of a goal on the edge")
    ap.add_argument("--min-passage", type=float, default=None,
                    help="a frontier goal in a slot between known obstacles narrower than this is skipped, and an "
                         "opening the lidar measures narrower than this aborts the attempt (too_narrow)")
    ap.add_argument("--yaw-clear", type=float, default=None,
                    help="below this lateral clearance sum (m) the yaw setpoint is frozen (the 0.77 m diagonal must not "
                         "turn inside a slot)")
    ap.add_argument("--yaw", choices=["crab", "forward"], default=None, help="air crab, ground forward")
    ap.add_argument("--yaw-rate", type=float, default=60.0)
    ap.add_argument("--speed", type=float, default=None, help="air 1.0, ground 0.30 m/s")
    ap.add_argument("--lookahead", type=float, default=0.8)
    ap.add_argument("--narrow-lookahead", type=float, default=None, help="air 0.3, ground 0.5")
    ap.add_argument("--narrow-width", type=float, default=1.0)
    ap.add_argument("--narrow-ahead", type=float, default=1.0)
    ap.add_argument("--center-gain", type=float, default=1.0)
    ap.add_argument("--repel-gain", type=float, default=1.0)
    ap.add_argument("--slow-cov", type=float, default=0.5, help="slow flight above this mapper-pose covariance (m^2)")
    ap.add_argument("--claim-tol", type=float, default=0.6, help="a frontier counts as reached within this radius")
    ap.add_argument("--timeout", type=float, default=90.0, help="per-frontier timeout, sim seconds")
    ap.add_argument("--timeout-clock", choices=["sim", "wall"], default="sim")
    ap.add_argument("--retry-wait", type=float, default=3.0)
    ap.add_argument("--stop-dist", type=float, default=0.40)
    ap.add_argument("--stop-sector", type=float, default=20.0)
    ap.add_argument("--blocked-s", type=float, default=20.0)
    ap.add_argument("--contact-dist", type=float, default=0.30)
    ap.add_argument("--impact-std", type=float, default=5.0)
    ap.add_argument("--max-impacts", type=int, default=2,
                    help="impacts before the exploration ends; 0 = never (the batch's protocol)")
    ap.add_argument("--max-belief-speed", type=float, default=3.0)
    ap.add_argument("--dart", type=float, default=1.5)
    ap.add_argument("--tko-ramp", type=float, default=0.5)
    ap.add_argument("--tko-speed", type=float, default=2.0)
    ap.add_argument("--force-dart", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--gate-s", type=float, default=10.0)
    ap.add_argument("--amcl-gate-s", type=float, default=40.0)
    ap.add_argument("--amcl-cov-max", type=float, default=2.0, help="the mapper's pose counts as converged below this covariance")
    ap.add_argument("--vio-stale-s", type=float, default=1.5)
    ap.add_argument("--odom-wall-stale-s", type=float, default=90.0)
    ap.add_argument("--amcl-stale-m", type=float, default=3.0)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--mav", default="udpin:0.0.0.0:14540")
    a = ap.parse_args()
    amr.resolve_vehicle_defaults(a)
    a.budgets = [float(b) for b in a.budgets.split(",")]
    a.run_id = time.strftime("%Y%m%d_%H%M%S")
    a.out_dir = a.out_dir or os.path.join(ROOT, "runs", "raw", f"explore_{a.seed}_{a.run_id}")
    a.tour_limit = 0
    a.map = None
    a.goals = None
    log_path = os.path.join(a.out_dir, f"explore_{a.seed}_{a.run_id}.log")
    os.makedirs(a.out_dir, exist_ok=True)
    logf = open(log_path, "a", buffering=1)

    def log(msg):
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        logf.write(line + "\n")

    log(f"[exec] FRONTIER3D world {a.seed} vehicle={a.vehicle} budgets={a.budgets} min -> {a.out_dir}")
    ex = Explorer(a, log)
    return ex.run()


if __name__ == "__main__":
    sys.exit(main())
