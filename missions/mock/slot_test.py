import os as _os
ROOT = _os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))   # the repo
MOCK_OUT = _os.path.join(ROOT, 'runs', 'raw', 'mock')
_os.makedirs(MOCK_OUT, exist_ok=True)
import sys, math, types
sys.path.insert(0, _os.path.join(ROOT, 'missions'))
import aerial_mission_runner as amr
from door_geom_test import scan, R   # the raycast on the frozen map of world 008
def clearance(x, y, axis_deg, d_door):
    yaw = math.radians(axis_deg + 90)          # crab: body x perpendicular to travel
    ex = types.SimpleNamespace(ros=R(scan(x, y, yaw)))
    h_rel = amr.wrap(math.radians(axis_deg) - yaw)
    cl = amr.Executive.lateral_clearance(ex, h_rel)
    shift, gap = amr.Executive.door_shift(ex, h_rel, d_door, 0.70)
    return cl, shift, gap
print("slot: cylinder (8.22,-7.41) r0.92 vs east wall x=9.75 -> true gap 0.61; drone on the slot axis x=9.45, travelling north")
for y in (-8.6, -8.2, -7.8, -7.41):
    cl, shift, gap = clearance(9.45, y, 90, max(0.0, -7.41 - y))
    s = None if cl[0] is None or cl[1] is None else round(cl[0] + cl[1], 2)
    print(f"  y={y:5.2f} d_door={max(0.0, -7.41 - y):.2f}: clearance L={cl[0] and round(cl[0],2)} R={cl[1] and round(cl[1],2)} sum={s}  door_shift shift={shift} gap={gap}")
print("pocket: box rnd_box_0 north end (y 9.34) vs north wall (9.75): 0.41 m; drone east of the slab moving north-west into the corner")
for (x, y) in ((-1.2, 9.0), (-1.4, 9.3), (-1.55, 9.5)):
    cl, shift, gap = clearance(x, y, 135, 0.3)
    s = None if cl[0] is None or cl[1] is None else round(cl[0] + cl[1], 2)
    print(f"  at ({x},{y}): clearance L={cl[0] and round(cl[0],2)} R={cl[1] and round(cl[1],2)} sum={s}  door_shift shift={shift} gap={gap}")
print("doors at the mouth (d_door 0.4 and 0.1), aligned, centred: expect sum >= 0.70")
for name, (wx, wy, axis) in {"N_A 0.91": (-4.09, 5.23, 180), "S_H 0.78": (-4.09, -6.44, 0), "W_A 0.84": (-5.57, -4.46, 270)}.items():
    for d in (0.4, 0.1):
        ux, uy = math.cos(math.radians(axis)), math.sin(math.radians(axis))
        x, y = wx - (d + 0.2) * ux, wy - (d + 0.2) * uy
        cl, shift, gap = clearance(x, y, axis, d)
        s = None if cl[0] is None or cl[1] is None else round(cl[0] + cl[1], 2)
        print(f"  {name} d={d}: clearance sum={s} (L={cl[0] and round(cl[0],2)} R={cl[1] and round(cl[1],2)}) door_shift gap={gap}")
