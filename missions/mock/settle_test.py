"""Offline check of the post-claim behaviour (smoke test 2026-09-25: 2 s after the claim of a
frontier goal 0.16 m from a wall the map did not have yet, the vehicle flew on toward the
carrot and touched the wall; the contact went uncounted because the impact guard ran only
inside goal attempts).

Checks, on the frozen map of world 008 with a raycast lidar and a PX4 mock WITH inertia
(first-order velocity lag, so a stop overshoots as the real vehicle does):
  claim   flying at 1 m/s toward a goal 0.16 m from the east wall: with the old behaviour
          (setpoint left on the carrot) the vehicle ends inside the planner radius of the
          wall; with hold_here() at the claim it stops >= 0.5 m from it
  standoff  settle() started 0.25 m from the wall backs off to ~--stop-dist (0.40 m)
  contact   an IMU spike during a settle that began calm counts one impact (where=settle)
  after     a settle that begins right after a contact (IMU still excited) counts nothing
  limit     --max-impacts 0 never stops a run (the batch protocol), N stops it at the N-th impact

  python3 settle_test.py          ->  "failures: 0"
"""
import math
import os
import sys
import threading
import time
import types

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(ROOT, "missions"))
import numpy as np  # noqa: E402

import aerial_mission_runner as amr  # noqa: E402
from gridmap import GridMap  # noqa: E402

GM = GridMap(os.path.join(ROOT, "worlds", "maps", "world_20260723008.yaml"))
SPEEDUP = 10.0
_real_sleep = time.sleep
amr.time = types.SimpleNamespace(time=time.time, sleep=lambda s: _real_sleep(s / SPEEDUP), strftime=time.strftime)


class Px4:
    """Position-setpoint follower: P controller (0.95 1/s, 1 m/s cap) with a 0.35 s velocity lag."""
    def __init__(self, x, y, yaw=0.0):
        self.x, self.y, self.z, self.yaw = x, y, 1.3, yaw
        self.vx = self.vy = 0.0
        self.sp = (x, y, 1.3, yaw)
        self.t = 0.0
        self.alive = True
        self.track = []
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        dt, tau = 0.01, 0.35
        while self.alive:
            _real_sleep(dt / SPEEDUP)
            sx, sy = self.sp[0], self.sp[1]
            ex, ey = sx - self.x, sy - self.y
            d = math.hypot(ex, ey)
            vcmd = min(1.0, 0.95 * d)
            ux, uy = (ex / d, ey / d) if d > 1e-6 else (0.0, 0.0)
            self.vx += (vcmd * ux - self.vx) * dt / tau
            self.vy += (vcmd * uy - self.vy) * dt / tau
            self.x += self.vx * dt
            self.y += self.vy * dt
            self.t += dt
            self.track.append((self.t, self.x, self.y))

    def enu(self):
        return (self.x, self.y, self.z, self.yaw, self.t)

    def speed(self):
        return math.hypot(self.vx, self.vy)

    def send_setpoint_enu(self, x, y, z, yaw):
        self.sp = (x, y, z, yaw)


class Ros:
    """Raycast lidar on the map from the true pose; an IMU excitation schedule [(t_from, t_to, std)]."""
    def __init__(self, px4, imu=()):
        self.px4, self.imu = px4, list(imu)

    def imu_excitation(self):
        t = self.px4.t
        for a, b, s in self.imu:
            if a <= t < b:
                return (t, s, 500)
        return (t, 0.05, 500)

    def nearest_return(self):
        ang = np.radians(np.arange(360.0)) + self.px4.yaw
        ts = np.arange(0.05, 6.0, 0.02)
        xs = self.px4.x + np.cos(ang)[:, None] * ts[None, :]
        ys = self.px4.y + np.sin(ang)[:, None] * ts[None, :]
        cs = np.floor((xs - GM.origin[0]) / GM.resolution).astype(int)
        rs = np.floor((ys - GM.origin[1]) / GM.resolution).astype(int)
        inside = (rs >= 0) & (rs < GM.h) & (cs >= 0) & (cs < GM.w)
        occ = np.zeros_like(inside)
        occ[inside] = GM.cls[rs[inside], cs[inside]] == GridMap.OCC
        hit = occ | ~inside
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(ts) - 1)
        r = ts[first]
        k = int(np.argmin(r))
        return float(r[k]), float(np.radians(k))


def executive(px4, ros):
    ex = object.__new__(amr.Executive)
    ex.a = types.SimpleNamespace(stop_dist=0.40, impact_std=5.0, alt=1.3, yaw_rate=60.0, rate=20.0)
    ex.px4, ex.ros = px4, ros
    ex.events = []
    ex.manifest = dict(events=ex.events)
    ex.log = lambda m: None
    ex.sp_xy, ex.sp_z, ex.sp_yaw, ex.tick_dt, ex.impacts = None, 1.3, 0.0, 0.05, 0
    return ex


def wall_x_at(y):
    """x of the east arena wall's first occupied cell at height y (map frame)."""
    r = int((y - GM.origin[1]) / GM.resolution)
    for c in range(int((5.0 - GM.origin[0]) / GM.resolution), GM.w):
        if GM.cls[r, c] == GridMap.OCC:
            return GM.origin[0] + c * GM.resolution
    raise SystemExit("no east wall found")


failures = 0


def check(name, ok, detail):
    global failures
    failures += 0 if ok else 1
    print(f"  {'OK  ' if ok else 'FAIL'} {name}: {detail}")


Y = -1.0                                       # the east arena wall at y = -1 of world 008 (clear of furniture)
WX = wall_x_at(Y)
print(f"east wall face at x = {WX:.2f} (y = {Y})")

# --- claim: toward a goal 0.16 m from the wall, claim radius 0.6 m (the explorer's)
for mode in ("old", "new"):
    goal = (WX - 0.16, Y)
    px4 = Px4(WX - 3.0, Y)
    ex = executive(px4, Ros(px4))
    px4.sp = (goal[0], goal[1], 1.3, 0.0)                      # the carrot at the goal
    while math.hypot(px4.x - goal[0], px4.y - goal[1]) > 0.6:
        _real_sleep(0.001)
    t_claim = px4.t
    if mode == "new":
        ex.hold_here()                                         # the fix: stop at the claim
        ex.settle(1.0, cap_s=3.0)                              # the breather, guarded
    else:                                                      # before: the setpoint stayed on the
        while px4.t < t_claim + 2.0:                           # carrot through the map refresh and
            _real_sleep(0.001)                                 # the breather
    closest = WX - max(x for _, x, _ in px4.track)
    px4.alive = False
    if mode == "old":
        check("claim, old behaviour (reproduces the smoke contact)", closest < 0.35, f"closest {closest:.2f} m from the wall")
    else:
        check("claim, hold_here at the claim", closest >= 0.5, f"closest {closest:.2f} m from the wall")

# --- standoff: a settle started 0.25 m from the wall
px4 = Px4(WX - 0.25, Y)
ex = executive(px4, Ros(px4))
ex.settle(2.0, cap_s=4.0)
_real_sleep(0.2)
d_end = WX - px4.x
px4.alive = False
check("standoff", 0.36 <= d_end <= 0.48 and any(e["event"] == "settle_standoff" for e in ex.events),
      f"ended {d_end:.2f} m from the wall; events {[e['event'] for e in ex.events]}")

# --- contact: calm at the start, a spike 0.3 s into the settle
px4 = Px4(0.0, 0.0)
ex = executive(px4, Ros(px4, imu=[(0.3, 0.9, 12.0)]))
ex.settle(1.0, cap_s=4.0)
px4.alive = False
imp = [e for e in ex.events if e["event"] == "impact"]
check("contact in a settle counts", ex.impacts == 1 and len(imp) == 1 and imp[0].get("where") == "settle",
      f"impacts={ex.impacts} events={[(e['event'], e.get('where')) for e in ex.events]}")

# --- after: the settle that follows a contact (IMU excited from the start) counts nothing
px4 = Px4(0.0, 0.0)
ex = executive(px4, Ros(px4, imu=[(0.0, 0.6, 12.0)]))
ex.settle(0.2, cap_s=4.0)
waited = px4.t
px4.alive = False
check("settle after a contact", ex.impacts == 0 and waited >= 0.55, f"impacts={ex.impacts}, waited {waited:.2f} sim-s for calm")

# --- limit: --max-impacts 0 never stops a run; N stops it at the N-th impact
ex = executive(None, None)
lim = []
for m, n in ((0, 0), (0, 5), (2, 1), (2, 2)):
    ex.a.max_impacts, ex.impacts = m, n
    lim.append(ex.impact_limit())
check("impact limit", lim == [False, False, False, True], f"(max, impacts) -> stop: {lim}")

print(f"failures: {failures}")
sys.exit(1 if failures else 0)
