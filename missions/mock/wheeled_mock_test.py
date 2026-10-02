"""Offline harness for the wheeled arm's runners (no Isaac, no ROS, no Nav2).

A kinematic stand-in replaces missions/wheeled_io.WheeledIO: the sim clock (/aeb/sim_time)
runs SPEEDUP x faster than wall; a unicycle robot on the TRUE geometry of world 008 (its
frozen map, worlds/maps/world_20260723008.yaml); a "Nav2" that plans with gridmap.py
(radius 0.30 m, unknown = free as the baseline's global costmap with
track_unknown_space: false) on the map it has (the live mapper's while exploring, the frozen
one in tours), drives the path at 0.30 m/s, SUCCEEDs within 0.35 m, ABORTs when it cannot
plan or makes no progress for 20 sim-s (a true obstacle on a path planned through unknown
space), and honours cancel; the belief (TF map->base_link) = truth + 3 cm; a raycast mapper
(the aerial mock's) for the exploration; ground truth written as wheeled_gt_*.csv (the
bootstrap's format) so analysis/verify_missions.py runs on the tour.

The REAL command lines run (main() of both runners with a patched argv).

  python3 wheeled_mock_test.py            -> the scenarios below, "failures: 0"
  scenarios: exploration (budgets 0.5,1 sim-min: the dart before the policy clock,
  checkpoints at the exact sim times, frontiers, explore_done), the same without the dart (the
  smoke's degenerate start: the mapper is gated on motion as slam_toolbox is), tour of 3 goals (SUCCEEDED + verify_missions REAL), a rejected first send,
  an unreachable goal (ABORTED twice), a stuck robot (TIMEOUT at the sim timeout), a stalled
  sim clock (SIM_STALLED), a stop signal (INTERRUPTED manifest)
"""
import argparse
import glob
import json
import math
import os
import subprocess
import sys
import threading
import time
import types

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "missions"))
import wheeled_io as wio  # noqa: E402
from gridmap import GridMap  # noqa: E402

TRUTH = GridMap(os.path.join(ROOT, "worlds", "maps", "world_20260723008.yaml"))
SPEEDUP = float(os.environ.get("SPEEDUP", "20"))
OUT = os.path.join(ROOT, "runs", "raw", "mock", "wheeled")
SEED = "20260723008"


class Sim:
    """Sim clock + unicycle robot + the fake Nav2 executor + the raycast mapper."""

    def __init__(self, out_dir, frozen_map=None, stuck=False, reject_first=0):
        self.t = 0.0
        self.x = self.y = self.yaw = 0.0
        self.alive = True
        self.clock_running = True
        self.stuck = stuck
        self.reject_left = reject_first
        self.goal = None                      # FakeGoal being driven
        self.frozen = GridMap(frozen_map) if frozen_map else None
        self.hits = np.zeros((TRUTH.h, TRUTH.w), dtype=np.int32)
        self.miss = np.zeros((TRUTH.h, TRUTH.w), dtype=np.int32)
        self.map_n = 0
        self.last_scan_t = -1.0
        self.manual = (0.0, 0.0)              # /cmd_vel (v, w) while no goal is active (the dart)
        self.last_int = None                  # pose of the last integrated scan (slam_toolbox's motion gate)
        self.lock = threading.Lock()
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.gt = open(os.path.join(out_dir, f"wheeled_gt_{SEED}_{ts}.csv"), "w", buffering=1)
        self.gt.write(f"# seed={SEED} fmt=gt1 (mock)\nsim_t,wall_t,x,y,z,qx,qy,qz,qw\n")
        self._scan_update()
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        dt, k = 0.02, 0
        while self.alive:
            time.sleep(dt / SPEEDUP)
            if not self.clock_running:
                continue
            with self.lock:
                self.t += dt
                k += 1
                g = self.goal
                if g is not None and g.state == "active":
                    g.step(dt)
                elif self.manual != (0.0, 0.0):
                    v, w = self.manual
                    self.x += v * dt * math.cos(self.yaw)
                    self.y += v * dt * math.sin(self.yaw)
                    self.yaw += w * dt
                if k % 5 == 0:
                    self.gt.write(f"{self.t:.3f},{time.time():.3f},{self.x:.4f},{self.y:.4f},0.175,0,0,"
                                  f"{math.sin(self.yaw / 2):.6f},{math.cos(self.yaw / 2):.6f}\n")
            if self.t - self.last_scan_t >= 0.2:
                self._scan_update()

    def _scan_update(self):
        """The aerial mock's raycast mapper from the true pose (12 m, 360 rays), gated as
        slam_toolbox is: a scan is integrated only after 0.2 m or 0.2 rad of motion since the
        last integrated one (minimum_travel_distance / _heading), and a cell is known only after
        two rays crossed it (min_pass_through 2) - a robot standing still keeps the first
        scan's star around it (smoke test 2026-09-26)."""
        self.last_scan_t = self.t
        if self.last_int is not None:
            lx, ly, lyaw = self.last_int
            if math.hypot(self.x - lx, self.y - ly) < 0.2 and abs(math.atan2(math.sin(self.yaw - lyaw), math.cos(self.yaw - lyaw))) < 0.2:
                return
        self.last_int = (self.x, self.y, self.yaw)
        ang = np.radians(np.arange(360.0)) + self.yaw
        ts = np.arange(0.05, 12.0, 0.05)
        xs = self.x + np.cos(ang)[:, None] * ts[None, :]
        ys = self.y + np.sin(ang)[:, None] * ts[None, :]
        cs = np.floor((xs - TRUTH.origin[0]) / TRUTH.resolution).astype(int)
        rs = np.floor((ys - TRUTH.origin[1]) / TRUTH.resolution).astype(int)
        inside = (rs >= 0) & (rs < TRUTH.h) & (cs >= 0) & (cs < TRUTH.w)
        occ = np.zeros_like(inside)
        occ[inside] = TRUTH.cls[rs[inside], cs[inside]] == GridMap.OCC
        first = np.where((occ | ~inside).any(axis=1), (occ | ~inside).argmax(axis=1), len(ts) - 1)
        free = np.arange(len(ts))[None, :] < first[:, None]
        okf = free & inside
        # one pass per ray and cell (a line-traced ray crosses each cell once; the 0.05 m samples
        # would count a cell twice on a diagonal): de-duplicate every ray's cells
        lin = np.where(okf, rs * TRUTH.w + cs, -1)
        lin.sort(axis=1)
        new = np.ones_like(lin, dtype=bool)
        new[:, 1:] = lin[:, 1:] != lin[:, :-1]
        cells = lin[new & (lin >= 0)]
        np.add.at(self.miss.reshape(-1), cells, 1)
        hit_r, hit_c = rs[np.arange(360), first], cs[np.arange(360), first]
        okh = (hit_r >= 0) & (hit_r < TRUTH.h) & (hit_c >= 0) & (hit_c < TRUTH.w) & occ[np.arange(360), first]
        np.add.at(self.hits, (hit_r[okh], hit_c[okh]), 1)
        self.map_n += 1

    def live_grid(self):
        g = np.full((TRUTH.h, TRUTH.w), -1, dtype=np.int8)
        occ = (self.hits >= 2) & (self.hits > 0.3 * self.miss)
        g[(self.miss >= 2) & ~occ] = 0
        g[occ] = 100
        return g

    def map_msg(self):
        info = types.SimpleNamespace(width=TRUTH.w, height=TRUTH.h, resolution=TRUTH.resolution,
                                     origin=types.SimpleNamespace(position=types.SimpleNamespace(x=TRUTH.origin[0], y=TRUTH.origin[1])))
        return types.SimpleNamespace(data=self.live_grid().ravel(), info=info)

    def nav_map(self):
        if self.frozen is not None:
            return self.frozen
        return GridMap.from_occupancy(self.live_grid().ravel(), TRUTH.w, TRUTH.h, TRUTH.resolution, TRUTH.origin)


class FakeGoal:
    def __init__(self, sim, x, y):
        self.sim, self.x, self.y = sim, x, y
        self.path_m, self.recoveries, self.error = float("nan"), 0, ""
        self.state = "pending"
        self.t_sent = time.time()
        self.path, self.idx, self.best, self.t_best = None, 0, float("inf"), 0.0
        if sim.reject_left > 0:
            sim.reject_left -= 1
            self.state = "rejected_pending"
            return
        gm = sim.nav_map()
        path, _used = gm.plan((sim.x, sim.y), (x, y), radius_m=0.30, unknown="free", tolerance=0.5)
        if not path:
            self.state = "abort_pending"
            self.error = "104:no valid path"
            return
        self.path = path
        self.path_m = GridMap.path_length(path)
        with sim.lock:
            if sim.goal is not None and sim.goal.state == "active":
                sim.goal.state = "CANCELED"                  # preempted
            sim.goal = self
            self.state = "active"
            self.t_best = sim.t

    def step(self, dt):
        s = self.sim
        d_goal = math.hypot(s.x - self.x, s.y - self.y)
        if d_goal <= 0.35:
            self.state = "SUCCEEDED"
            return
        if d_goal < self.best - 0.05:
            self.best, self.t_best = d_goal, s.t
        if s.t - self.t_best > 20.0:
            self.state = "ABORTED"
            self.error = "105:failed to make progress"
            return
        if s.stuck:
            return
        while self.idx < len(self.path) - 1 and math.hypot(self.path[self.idx][0] - s.x, self.path[self.idx][1] - s.y) < 0.15:
            self.idx += 1
        tx, ty = self.path[self.idx]
        h = math.atan2(ty - s.y, tx - s.x)
        nx, ny = s.x + 0.30 * dt * math.cos(h), s.y + 0.30 * dt * math.sin(h)
        r, c = TRUTH.to_cell(nx, ny)
        if self._leth[r, c]:
            return                                             # a true obstacle: no motion (Nav2 then aborts)
        s.x, s.y, s.yaw = nx, ny, h

    def poll(self):
        if self.state == "rejected_pending":
            return "rejected" if time.time() - self.t_sent > 0.05 else "pending"
        if self.state == "abort_pending":
            if time.time() - self.t_sent > 0.1:
                self.state = "ABORTED"
            return "pending" if self.state == "abort_pending" else self.state
        return self.state

    def cancel(self, wait_s=10.0):
        with self.sim.lock:
            if self.state == "active":
                self.state = "CANCELED"


_LETHAL = TRUTH.lethal(0.20, "free")
FakeGoal._leth = _LETHAL


class FakeIO:
    def __init__(self, log, sim, **kw):
        self.sim, self.log = sim, log

    def sim_t(self):
        return self.sim.t

    def sim_age(self):
        return 0.0 if self.sim.clock_running else getattr(self, "_stall_age", 999.0)

    def pose(self):
        s = self.sim
        return (s.x + 0.03, s.y - 0.02, s.yaw)

    def odom(self):
        return (self.sim.x * 1.02, self.sim.y * 0.98, self.sim.yaw)

    def amcl(self):
        s = self.sim
        return (s.x, s.y, s.yaw, 0.01, 0.01, time.time()), 5

    def latest_map(self):
        return (self.sim.map_msg(), self.sim.map_n) if self.sim.frozen is None else (None, 0)

    def nav_ready(self, timeout_s):
        return True

    def send(self, x, y):
        return FakeGoal(self.sim, x, y)

    def cmd(self, v, w=0.0):
        self.sim.manual = (float(v), float(w))

    def close(self):
        pass


def patch_time():
    real_sleep = time.sleep
    fake = types.SimpleNamespace(time=time.time, sleep=lambda s: real_sleep(min(s, 0.05)), strftime=time.strftime)
    return fake


def run_main(module, argv, sim):
    """Run the runner's REAL main() with argv, WheeledIO replaced by FakeIO over `sim`."""
    real_parse = argparse.ArgumentParser.parse_args
    argparse.ArgumentParser.parse_args = lambda self, args=None, namespace=None: real_parse(self, argv, namespace)
    real_io = wio.WheeledIO
    wio.WheeledIO = lambda log, **kw: FakeIO(log, sim, **kw)
    try:
        return module.main()
    finally:
        argparse.ArgumentParser.parse_args = real_parse
        wio.WheeledIO = real_io


failures = 0


def check(name, ok, detail):
    global failures
    failures += 0 if ok else 1
    print(f"  {'OK  ' if ok else 'FAIL'} {name}: {detail}", flush=True)


def newest(pattern):
    c = sorted(glob.glob(pattern), key=os.path.getmtime)
    return c[-1] if c else None


import wheeled_explore_runner as wex  # noqa: E402
import wheeled_mission_runner as wmr  # noqa: E402

# the runners install SIGINT/SIGTERM handlers; the harness keeps its own Ctrl+C
wio.install_stop_signals = lambda: None

# ---------------------------------------------------------------- exploration
d = os.path.join(OUT, "explore")
os.makedirs(d, exist_ok=True)
for f in glob.glob(os.path.join(d, "*")):
    os.remove(f)
sim = Sim(d)
t0 = time.time()
rc = run_main(wex, ["--seed", SEED, "--budgets", "0.5,1", "--out-dir", d, "--timeout", "90"], sim)
sim.alive = False
man = json.load(open(newest(os.path.join(d, "manifest_*.json"))))
cps = man.get("checkpoints", [])
ev = [e["event"] for e in man["events"]]
fr = open(newest(os.path.join(d, "frontiers_*.csv"))).read().strip().split("\n")[1:]
check("explore exit + status", rc == 0 and man["status"] == "OK", f"rc={rc} status={man['status']} ({time.time() - t0:.0f} s wall)")
check("explore checkpoints", [c["budget_min"] for c in cps] == [0.5, 1.0]
      and all(os.path.isfile(os.path.join(ROOT, c["map"])) for c in cps)
      and all(abs(c["t_sim"] - 60 * c["budget_min"]) < 0.25 * SPEEDUP for c in cps if not c["final"]),
      f"{[(c['budget_min'], c['t_sim'], c['coverage_m2'], c['final']) for c in cps]}")
check("explore frontiers", len(fr) >= 3 and "explore_done" in ev and "policy_clock_start" in ev,
      f"{len(fr)} frontier rows, summary {man.get('summary')}")
check("explore coverage grows", cps and cps[-1]["coverage_m2"] > 60, f"final coverage {cps[-1]['coverage_m2'] if cps else None} m^2")
dd = {e["event"]: e for e in man["events"]}
check("dart before the policy clock",
      all(k in dd for k in ("dart_start", "dart_out", "dart_back", "dart_done"))
      and ev.index("dart_done") < ev.index("policy_clock_start")
      and dd["dart_start"]["coverage_m2"] < 15 < 3 * dd["dart_start"]["coverage_m2"] < dd["dart_done"]["coverage_m2"]
      and 1.4 <= dd["dart_out"]["along_m"] <= 1.5 + 0.02 * SPEEDUP and abs(dd["dart_back"]["along_m"]) <= 0.02 * SPEEDUP,
      f"coverage {dd.get('dart_start', {}).get('coverage_m2')} -> {dd.get('dart_done', {}).get('coverage_m2')} m^2, "
      f"out {dd.get('dart_out', {}).get('along_m')} m, back {dd.get('dart_back', {}).get('along_m')} m")

# ---------------------------------------------------------------- exploration without the dart: the smoke's degenerate start
d0 = os.path.join(OUT, "explore_nodart")
os.makedirs(d0, exist_ok=True)
for f in glob.glob(os.path.join(d0, "*")):
    os.remove(f)
sim = Sim(d0)
rc = run_main(wex, ["--seed", SEED, "--budgets", "0.5,1", "--out-dir", d0, "--dart", "0"], sim)
sim.alive = False
m0 = json.load(open(newest(os.path.join(d0, "manifest_*.json"))))
s0 = m0.get("summary") or {}
check("no dart reproduces the smoke (degenerate start)", s0.get("reason") == "exhausted" and s0.get("frontiers") == 1
      and s0.get("t_sim", 99) < 0.5 * SPEEDUP and s0.get("coverage_m2", 99) < 15, f"summary {s0}")

# ---------------------------------------------------------------- tour: 3 goals + verify_missions
d = os.path.join(OUT, "tour")
os.makedirs(d, exist_ok=True)
for f in glob.glob(os.path.join(d, "*")):
    os.remove(f)
frozen = os.path.join(ROOT, "worlds", "maps", f"world_{SEED}.yaml")
sim = Sim(d, frozen_map=frozen, reject_first=1)
rc = run_main(wmr, ["--seed", SEED, "--map", frozen, "--tour-limit", "3", "--out-dir", d], sim)
sim.alive = False
time.sleep(0.2)
man = json.load(open(newest(os.path.join(d, "manifest_*.json"))))
rows = man["results"]
check("tour exit + 3 goals", rc == 0 and man["status"] == "OK" and len({r["mission"] for r in rows}) == 3,
      f"rc={rc} status={man['status']} results={[(r['mission'], r['attempt'], r['result'], r['reason']) for r in rows]}")
vm = subprocess.run([sys.executable, os.path.join(ROOT, "analysis", "verify_missions.py"), "--missions",
                     newest(os.path.join(d, "missions_*.csv"))], capture_output=True, text=True)
check("verify_missions on the tour", vm.returncode == 0 and "real" in vm.stdout,
      (vm.stdout.strip().split("\n")[-1] if vm.stdout else vm.stderr.strip()[-300:]))

# ---------------------------------------------------------------- unreachable goal: ABORTED twice
d2 = os.path.join(OUT, "tour_unreach")
os.makedirs(d2, exist_ok=True)
goals = os.path.join(d2, "goals.csv")
open(goals, "w").write("mission,room,x,y\ng01,wall,10.0,0.0\n")     # inside the east boundary wall
sim = Sim(d2, frozen_map=frozen)
rc = run_main(wmr, ["--seed", SEED, "--map", frozen, "--goals", goals, "--out-dir", d2], sim)
sim.alive = False
man = json.load(open(newest(os.path.join(d2, "manifest_*.json"))))
res = [(r["attempt"], r["result"]) for r in man["results"]]
check("unreachable goal", res == [(1, "ABORTED"), (2, "ABORTED")], f"{res}")

# ---------------------------------------------------------------- stuck robot: TIMEOUT at the sim timeout
d3 = os.path.join(OUT, "tour_stuck")
os.makedirs(d3, exist_ok=True)
open(os.path.join(d3, "goals.csv"), "w").write("mission,room,x,y\ng01,NE,2.0,0.0\n")
sim = Sim(d3, frozen_map=frozen, stuck=True)
FakeGoal_step = FakeGoal.step
FakeGoal.step = lambda self, dt: None if not self.sim.stuck else None       # no motion, no abort
rc = run_main(wmr, ["--seed", SEED, "--map", frozen, "--goals", os.path.join(d3, "goals.csv"), "--out-dir", d3,
                    "--timeout", "30"], sim)
FakeGoal.step = FakeGoal_step
sim.alive = False
man = json.load(open(newest(os.path.join(d3, "manifest_*.json"))))
res = [(r["result"], float(r["sim_s"])) for r in man["results"]]
check("stuck robot -> TIMEOUT", [x[0] for x in res] == ["TIMEOUT", "TIMEOUT"] and all(29.9 <= s <= 32 for _, s in res), f"{res}")

# ---------------------------------------------------------------- stalled sim clock
d4 = os.path.join(OUT, "tour_stall")
os.makedirs(d4, exist_ok=True)
sim = Sim(d4, frozen_map=frozen)
threading.Timer(1.0, lambda: setattr(sim, "clock_running", False)).start()
rc = run_main(wmr, ["--seed", SEED, "--map", frozen, "--tour-limit", "3", "--out-dir", d4, "--sim-stale-s", "5"], sim)
sim.alive = False
man = json.load(open(newest(os.path.join(d4, "manifest_*.json"))))
check("stalled sim clock -> SIM_STALLED", man["status"] == "SIM_STALLED" and rc == 1, f"status={man['status']} rc={rc}")

# ---------------------------------------------------------------- stop signal mid-tour
d5 = os.path.join(OUT, "tour_int")
os.makedirs(d5, exist_ok=True)
sim = Sim(d5, frozen_map=frozen)


def _interrupt():
    import _thread
    _thread.interrupt_main()


threading.Timer(1.5, _interrupt).start()
rc = run_main(wmr, ["--seed", SEED, "--map", frozen, "--tour-limit", "3", "--out-dir", d5], sim)
sim.alive = False
man = json.load(open(newest(os.path.join(d5, "manifest_*.json"))))
check("stop signal -> INTERRUPTED manifest", man["status"] == "INTERRUPTED", f"status={man['status']} results={len(man['results'])}")

print(f"failures: {failures}")
sys.exit(1 if failures else 0)
