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
        if self.mode != 'amcl' or self.px4 is None or self.px4.z < 1.0: return None, 0
        x, y, yaw, t = self._truth()
        th = math.radians(2.0); c, s = math.cos(th), math.sin(th)
        vx, vy = 0.8*(c*x - s*y), 0.8*(s*x + c*y)     # scaled + rotated dead reckoning
        self.n += 1
        self.vio_hist.append((t, vx, vy, yaw + th))
        self.vio_hist = self.vio_hist[-400:]
        self.vio_rx_sim = t
        return (vx, vy, 0.0, yaw + th, t, time.time()), self.n
    def latest_amcl(self):
        if self.mode != 'amcl' or self.px4 is None or self.px4.z < 1.0 or self.n < 5: return None, 0, None
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


amr.Px4Link = MockPx4
amr.RosIO = MockRos
# speed up the executive's wall-clock waits: monkeypatch time.sleep inside the module
_real_sleep = time.sleep
amr.time = types.SimpleNamespace(time=time.time, sleep=lambda s: _real_sleep(s / SPEEDUP), strftime=time.strftime)

# the REAL command line of the mission executive (a parameter missing from a parser killed an exploration test flight)
import argparse, json
_real_parse = argparse.ArgumentParser.parse_args
argv = ['aerial_mission_runner.py', '--seed', '20260723008', '--goals', os.environ.get('GOALS', f'{ROOT}/missions/goals/goals_20260723008.csv'),
        '--estimator', (sys.argv[2] if len(sys.argv) > 2 else 'px4'), '--force-dart', '--mav', 'mock',
        '--out-dir', MOCK_OUT,
        '--tour-limit', (sys.argv[1] if len(sys.argv) > 1 else '4'),
        '--narrow-lookahead', os.environ.get("NARROW", "0.3"), '--repel-gain', os.environ.get('REPEL', '0'),
        '--center-gain', os.environ.get("CENTER", "1.0")]
def _parse(self, args=None, namespace=None):
    return _real_parse(self, argv[1:], namespace)
argparse.ArgumentParser.parse_args = _parse
traj = []
_real_exec = amr.Executive
class _Executive(_real_exec):
    def __init__(self, a, log):
        super().__init__(a, log)
        if hasattr(self.ros, 'attach'): self.ros.attach(self.px4)
        ex = self
        def tracker():
            while ex.px4.alive:
                traj.append((ex.px4.t_boot, ex.px4.x, ex.px4.y, ex.px4.z, ex.px4.yaw)); _real_sleep(0.02)
        threading.Thread(target=tracker, daemon=True).start()
amr.Executive = _Executive
if os.environ.get('FAIL_TAKEOFF'):          # the setup-failure path: status must survive the finally block
    _real_exec.takeoff = lambda self: False
rc = amr.main()
print("exit", rc)
open(_os.path.join(MOCK_OUT, 'mock_traj.json'), 'w').write(json.dumps(traj))
raise SystemExit(rc)
