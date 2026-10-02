import os as _os
ROOT = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))   # the repo
MOCK_OUT = _os.path.join(ROOT, 'runs', 'raw', 'mock')
_os.makedirs(MOCK_OUT, exist_ok=True)
import sys, math, types, numpy as np
sys.path.insert(0, _os.path.join(ROOT, 'missions'))
import aerial_mission_runner as amr
from gridmap import GridMap
GM = GridMap(_os.path.join(ROOT, 'worlds', 'maps', 'world_20260723008.yaml'))
def scan(x, y, yaw):
    ang = np.radians(np.arange(360.0)) + yaw
    ts = np.arange(0.05, 12.0, 0.05)
    xs = x + np.cos(ang)[:, None] * ts[None, :]; ys = y + np.sin(ang)[:, None] * ts[None, :]
    cs = np.floor((xs - GM.origin[0]) / GM.resolution).astype(int); rs = np.floor((ys - GM.origin[1]) / GM.resolution).astype(int)
    inside = (rs >= 0) & (rs < GM.h) & (cs >= 0) & (cs < GM.w)
    occ = np.zeros_like(inside); occ[inside] = GM.cls[rs[inside], cs[inside]] == 1
    hit = occ | ~inside
    first = np.where(hit.any(axis=1), hit.argmax(axis=1), len(ts) - 1)
    return np.radians(np.arange(360.0)), ts[first]
class R:
    def __init__(self, sc): self.sc = sc
    def sector_returns(self, a_from, a_to, max_range):
        a, r = self.sc
        mid, half = (a_from + a_to) / 2.0, (a_to - a_from) / 2.0
        sel = (np.abs(amr.wrap_arr(a - mid)) <= half) & (r > 0.05) & (r < max_range)
        return r[sel], amr.wrap_arr(a[sel] - mid) + mid
# map faces (measured from the raster): N_A east face x=-3.84, S_H west face x=-4.34 (travel east from the SW room), W_A south face y=-4.26 (travel north? the tour goes NW->SW = south) -> use travel south: north face y=-4.66? use the free-width scan instead
def face_and_centre(name):
    doors = {"N_A": (-4.09, 5.23, "x"), "S_H": (-4.09, -6.44, "x"), "W_A": (-5.57, -4.46, "y")}
    return doors[name]
MINW = float(sys.argv[1]) if len(sys.argv) > 1 else 0.45
tests = [("N_A", 180), ("S_H", 0), ("W_A", 270)]
bad = 0
for name, axis_deg in tests:
    wx, wy, ax = face_and_centre(name)
    print(f"door {name}: axis {axis_deg} deg")
    for d in (1.0, 0.6, 0.3, 0.05):
        for off in (-0.3, 0.0, 0.3):
            ux, uy = math.cos(math.radians(axis_deg)), math.sin(math.radians(axis_deg))
            # drone = door centre - (d + 0.2) * axis  (0.2 = half the wall thickness) + off * left
            lx, ly = -uy, ux
            x = wx - (d + 0.2) * ux + off * lx; y = wy - (d + 0.2) * uy + off * ly
            yaw = math.radians(axis_deg + 90)
            ex = types.SimpleNamespace(ros=R(scan(x, y, yaw)))
            h_rel = amr.wrap(math.radians(axis_deg) - yaw)
            shift, gap = amr.Executive.door_shift(ex, h_rel, d, MINW)
            expected = -off
            ok = shift is not None and abs(shift - expected) < 0.08 and gap is not None and 0.65 <= gap <= 0.95
            bad += not ok
            print(f"   d={d:4.2f} off={off:+.1f} drone=({x:.2f},{y:.2f}): shift={None if shift is None else round(float(shift),2)} gap={gap} expected={expected:+.2f} {'OK' if ok else '<-- CHECK'}")
print("failures:", bad)
