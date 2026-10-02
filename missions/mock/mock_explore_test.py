"""Offline harness: kinematic mock of PX4 + no ROS, drives the executive's tour logic."""
import os as _os
ROOT = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))   # the repo
MOCK_OUT = _os.path.join(ROOT, 'runs', 'raw', 'mock')
_os.makedirs(MOCK_OUT, exist_ok=True)
import math, sys, threading, time, types, os
sys.path.insert(0, _os.path.join(ROOT, 'missions'))
import aerial_mission_runner as amr
import numpy as np
from gridmap import GridMap
GM = GridMap(_os.path.join(ROOT, 'worlds', 'maps', 'world_20260723008.yaml'))

SPEEDUP = 20.0   # mock sim runs 20x faster than wall


class MockPx4:
    OFFBOARD_MAIN = 6
    AUTO_MAIN, AUTO_LAND_SUB = 4, 6
    def __init__(self, url, log, speed=1.0, lag=0.0, drift=(0.0, 0.0)):
        self.log = log; self.x = self.y = self.z = 0.0; self.yaw = 0.0
        self.sp = (0.0, 0.0, 0.0, 0.0); self.speed_cap = speed
        self.t_boot = 0.0; self.armed = False; self.main_mode = None; self.landed_state = 1
        self.acks = []; self.alive = True; self.v = 0.0
        self.thread = threading.Thread(target=self._sim, daemon=True); self.thread.start()
    def _sim(self):
        dt = 0.01
        while self.alive:
            time.sleep(dt / SPEEDUP)
            self.t_boot += dt
            if self.main_mode == 4:                      # AUTO.LAND
                self.z = max(0.0, self.z - 0.5 * dt)
                if self.z <= 0.0: self.landed_state = 1; self.armed = False
                continue
            if not self.armed or self.main_mode != 6:
                continue
            sx, sy, sz, syaw = self.sp
            dx, dy = sx - self.x, sy - self.y
            d = math.hypot(dx, dy)
            v = min(self.speed_cap, 0.95 * d)      # P controller like PX4 (MPC_XY_P ~0.95)
            self.v = v
            if d > 1e-6:
                self.x += dx / d * v * dt; self.y += dy / d * v * dt
            dz = sz - self.z; self.z += max(-1.0, min(1.0, dz * 2.0)) * dt
            dyaw = amr.wrap(syaw - self.yaw); self.yaw += max(-2.0, min(2.0, dyaw * 3.0)) * dt
            if self.main_mode == 4:
                self.z = max(0.0, self.z - 0.5 * dt)
                if self.z <= 0.0: self.landed_state = 1
            else:
                self.landed_state = 2 if self.z > 0.05 else 1
    def enu(self): return (self.x, self.y, self.z, amr.wrap(self.yaw), self.t_boot)
    def speed(self): return self.v
    def send_setpoint_enu(self, x, y, z, yaw): self.sp = (x, y, z, yaw)
    def set_mode(self, main, sub=0): self.main_mode = main
    def arm(self, arm=True, force=False): self.armed = arm
    def set_param(self, *a, **k): pass
    def close(self): self.alive = False


class MockRos:
    """estimator 'amcl' (sys.argv[2]): VIO = truth scaled 0.8 and rotated 2 deg (dead reckoning
    error), AMCL = truth + 3 cm noise every 0.5 s sim; 'vio': nothing (VIO_INIT_FAIL path)."""
    def __init__(self, log, **k):
        self.mode = sys.argv[2] if len(sys.argv) > 2 else 'px4'
        self.lock = threading.Lock(); self.anchor_T = None
        self.px4 = None; self.n = 0; self.amcl_n = 0; self.corr = None; self.t_last_amcl = -1
        self.vio_hist = []; self.vio_rx_sim = None; self.sim_clock = None
    def attach(self, px4): self.px4 = px4
    def imu_excitation(self): return (0.0, 0.05, 500)
    def _truth(self):
        return (self.px4.x, self.px4.y, self.px4.yaw, self.px4.t_boot) if self.px4 else None
    def latest_vio(self):
        if self.mode != 'amcl' or self.px4 is None or (self.px4.z < 1.0 and not getattr(self.px4, 'ground', False)): return None, 0
        x, y, yaw, t = self._truth()
        th = math.radians(2.0); c, s = math.cos(th), math.sin(th)
        vx, vy = 0.8*(c*x - s*y), 0.8*(s*x + c*y)     # scaled + rotated dead reckoning
        self.n += 1
        self.vio_hist.append((t, vx, vy, yaw + th))
        self.vio_hist = self.vio_hist[-400:]
        self.vio_rx_sim = t
        return (vx, vy, 0.0, yaw + th, t, time.time()), self.n
    def latest_amcl(self):
        if self.mode != 'amcl' or self.px4 is None or (self.px4.z < 1.0 and not getattr(self.px4, 'ground', False)) or self.n < 5: return None, 0, None
        x, y, yaw, t = self._truth()
        if os.environ.get('LATE_AMCL'):
            if not hasattr(self, 't_first'): self.t_first = t; print(f'[mock] AMCL silent until sim {t+8:.1f}', flush=True)
            if t - self.t_first < 8.0: return None, 0, None
        if t - self.t_last_amcl >= 0.5:
            self.t_last_amcl = t; self.amcl_n += 1
            ax, ay, ayaw = x + 0.03, y - 0.02 + float(os.environ.get('AMCL_BIAS', '0')), yaw + 0.01
            # correction against the odom pose at the same stamp (as the executive does)
            od = min(self.vio_hist, key=lambda h: abs(h[0]-t)) if self.vio_hist else None
            if od:
                theta = amr.wrap(ayaw - od[3]); c, s = math.cos(theta), math.sin(theta)
                self.corr = (theta, ax - (c*od[1] - s*od[2]), ay - (s*od[1] + c*od[2]))
            self.amcl = (ax, ay, ayaw, t, time.time(), 0.01, 0.01, 0.01)
            self.amcl_odom = (od[1], od[2], od[3]) if od else None
        return getattr(self, 'amcl', None), self.amcl_n, self.corr
    def odom_since_amcl(self):
        v, _ = self.latest_vio()
        ao = getattr(self, 'amcl_odom', None)
        if v is None or ao is None: return None
        return math.hypot(v[0]-ao[0], v[1]-ao[1])
    # --- raycast lidar on the frozen map from the TRUE pose (360 rays, 12 m) ---
    def _scan(self):
        if self.px4 is None: return None
        key = (round(self.px4.x, 2), round(self.px4.y, 2), round(self.px4.yaw, 2))
        if getattr(self, '_scan_key', None) == key: return self._scan_cache
        gm = GM
        ang = np.radians(np.arange(360.0)) + self.px4.yaw
        ts = np.arange(0.05, 12.0, 0.05)
        xs = self.px4.x + np.cos(ang)[:, None] * ts[None, :]
        ys = self.px4.y + np.sin(ang)[:, None] * ts[None, :]
        cs = np.floor((xs - gm.origin[0]) / gm.resolution).astype(int)
        rs = np.floor((ys - gm.origin[1]) / gm.resolution).astype(int)
        inside = (rs >= 0) & (rs < gm.h) & (cs >= 0) & (cs < gm.w)
        occ = np.zeros_like(inside)
        occ[inside] = gm.cls[rs[inside], cs[inside]] == 1
        hit = occ | ~inside
        first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(ts) - 1)
        ranges = ts[first]
        self._scan_key = key; self._scan_cache = (np.radians(np.arange(360.0)), ranges)
        return self._scan_cache
    def min_range_towards(self, rel, half_width=0.3):
        sc = self._scan()
        if sc is None: return None
        a, r = sc
        sel = np.abs(amr.wrap_arr(a - rel)) <= half_width
        return float(r[sel].min()) if sel.any() else None
    def min_range_all(self):
        sc = self._scan()
        return None if sc is None else float(sc[1].min())
    def nearest_return(self):
        sc = self._scan()
        if sc is None: return None
        k = int(np.argmin(sc[1])); return float(sc[1][k]), float(sc[0][k])
    def sector_returns(self, a_from, a_to, max_range):
        sc = self._scan()
        if sc is None: return None
        a, r = sc
        mid, half = (a_from + a_to) / 2.0, (a_to - a_from) / 2.0
        sel = (np.abs(amr.wrap_arr(a - mid)) <= half) & (r > 0.05) & (r < max_range)
        return r[sel], amr.wrap_arr(a[sel] - mid) + mid
    def close(self): pass



# ---- a perfect mapper on top of the mock lidar: integrates the true-pose scans into a grid ----
class MockMapper:
    """A crude probabilistic mapper on the mock lidar: per-cell hit / miss counters from the
    true-pose scans (occupied = hits >= 3 and hits > 0.3 * misses; free = misses >= 2)."""
    def __init__(self, ros, res=0.05, origin=(-10.0, -10.0), size=400):
        self.ros = ros; self.res = res; self.origin = origin; self.size = size
        self.hits = np.zeros((size, size), dtype=np.int32); self.miss = np.zeros((size, size), dtype=np.int32)
        self.count = 0; self.last_t = -1.0
    def update(self):
        px4 = self.ros.px4
        if px4 is None or (px4.z < 1.0 and not getattr(px4, 'ground', False)): return
        if px4.t_boot - self.last_t < 0.5: return
        self.last_t = px4.t_boot
        sc = self.ros._scan()
        if sc is None: return
        ang, rr = sc
        ts = np.arange(0.0, 12.0, self.res)
        A = ang + px4.yaw
        xs = px4.x + np.cos(A)[:, None] * ts[None, :]; ys = px4.y + np.sin(A)[:, None] * ts[None, :]
        free_mask = ts[None, :] < (np.minimum(rr, 12.0) - self.res)[:, None]
        cs = np.floor((xs - self.origin[0]) / self.res).astype(int); rs = np.floor((ys - self.origin[1]) / self.res).astype(int)
        ok = free_mask & (rs >= 0) & (rs < self.size) & (cs >= 0) & (cs < self.size)
        np.add.at(self.miss, (rs[ok], cs[ok]), 1)
        hit = rr < 11.9
        hx = px4.x + np.cos(A[hit]) * rr[hit]; hy = px4.y + np.sin(A[hit]) * rr[hit]
        hc = np.floor((hx - self.origin[0]) / self.res).astype(int); hr = np.floor((hy - self.origin[1]) / self.res).astype(int)
        ok2 = (hr >= 0) & (hr < self.size) & (hc >= 0) & (hc < self.size)
        np.add.at(self.hits, (hr[ok2], hc[ok2]), 1)
        self.count += 1
    def grid(self):
        g = np.full((self.size, self.size), -1, dtype=np.int8)
        occ = (self.hits >= 3) & (self.hits > 0.3 * self.miss)
        free = (self.miss >= 2) & ~occ
        g[free] = 0; g[occ] = 100
        return g
    def msg(self):
        info = types.SimpleNamespace(width=self.size, height=self.size, resolution=self.res,
                                     origin=types.SimpleNamespace(position=types.SimpleNamespace(x=self.origin[0], y=self.origin[1])))
        return types.SimpleNamespace(data=self.grid().ravel(), info=info)

_orig_init = MockRos.__init__
def _init(self, log, **k):
    _orig_init(self, log, **k); self.mapper = MockMapper(self)
MockRos.__init__ = _init
def latest_map(self):
    self.mapper.update()
    if self.mapper.count == 0: return None, 0
    return self.mapper.msg(), self.mapper.count
MockRos.latest_map = latest_map

amr.Px4Link = MockPx4
amr.RosIO = MockRos
GROUND = bool(os.environ.get('GROUND'))
if GROUND:                                   # the wheeled robot under the same explorer
    from ground_stub import MockGround, command_rule
    amr.GroundLink = MockGround
_real_sleep = time.sleep
amr.time = types.SimpleNamespace(time=time.time, sleep=lambda s: _real_sleep(s / SPEEDUP), strftime=time.strftime)
sys.argv = [sys.argv[0], '3', 'amcl']
import aerial_explore_runner as aer
aer.time = amr.time
out = _os.path.join(MOCK_OUT, 'explore_mock')
os.makedirs(out, exist_ok=True)
# the REAL command line of the explorer (a parameter missing from its parser killed a test flight)
import argparse
_real_parse = argparse.ArgumentParser.parse_args
argv = ['aerial_explore_runner.py', '--seed', '20260723008', '--budgets', os.environ.get('BUDGETS', '0.5,1,2'),
        '--out-dir', out, '--map-wait-s', '30', '--mav', 'mock'] + (['--vehicle', 'ground'] if GROUND else [])
def _parse(self, args=None, namespace=None):
    return _real_parse(self, argv[1:], namespace)
argparse.ArgumentParser.parse_args = _parse
_real_explorer = aer.Explorer
class _Explorer(_real_explorer):
    def __init__(self, a, log):
        super().__init__(a, log)
        if hasattr(self.ros, 'attach'): self.ros.attach(self.px4)
aer.Explorer = _Explorer
rc = aer.main()
print("exit", rc)
if GROUND:
    n, slow_turn, slow_fwd = command_rule()
    print(f"commands: {n}, turns in place below 0.5 rad/s: {slow_turn}, forward below 0.10 m/s: {slow_fwd}")
    rc = rc or (5 if (slow_turn or slow_fwd) else 0)
raise SystemExit(rc)
