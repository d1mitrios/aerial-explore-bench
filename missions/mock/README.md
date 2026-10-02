# missions/mock: offline harnesses for the shared executive (both vehicles), the exploration runner and the wheeled Nav2 runners

No Isaac, no PX4, no ROS: a kinematic PX4 mock (1 m/s, lag), a raycast lidar on the
frozen map of world 008 (`worlds/maps/world_20260723008.yaml`), a localizer mock (truth +
noise, optional bias) and, for the explorer, a probabilistic mapper, all at 20× speed.
They drive the REAL command lines (`aerial_mission_runner.py main()`,
`aerial_explore_runner.py main()`), so a parameter added to one parser and not the other
fails here (an exploration test flight died on exactly that). Outputs go to `runs/raw/mock/`
(git-ignored). Run from this directory:

| Script | What it checks | Expected |
|---|---|---|
| `python3 mock_exec_test.py 3 amcl` | the mission tour: dart, gates, three goals through the doorways, guarded return | `TOUR DONE: 3/3`, `home_reached`, `exit 0` |
| `AMCL_BIAS=0.25 REPEL=1 python3 mock_exec_test.py 3 amcl` (also `-0.25`) | the same with a ±0.25 m localizer bias and the live repulsion (`--repel-gain`) | 3/3 |
| `GOALS=goals_slot.csv REPEL=1 python3 mock_exec_test.py 1 amcl` with a goals file holding `g01,slot,9.40,-7.43` | a goal inside the cylinder/wall slot | `too_narrow` + `retreat_done moved_m≈0.6`, `home_reached` |
| `BUDGETS=1,2.5 python3 mock_explore_test.py` | the exploration loop, checkpoints at the exact sim times, frontier blacklist, return | `explore_done reason=budget`, `home_reached`, `exit 0` |
| `python3 settle_test.py` | the post-claim behaviour on a PX4 mock with inertia: a claim 0.6 m before a goal 0.16 m from the wall; the old setpoint-on-the-carrot reproduces the contact of the smoke test of 2026-09-25 (0.16 m), `hold_here` keeps ≥ 0.5 m; the settle standoff backs off to `--stop-dist`; a contact inside a settle counts once, a settle after a contact counts nothing | `failures: 0` |
| `python3 door_geom_test.py` | `door_shift` centring at the three doors of world 008 from 0.05–1 m, ±0.3 m offsets | `failures: 0` |
| `python3 wheeled_mock_test.py` | the wheeled runners' real command lines on a kinematic stand-in (sim clock at 20×, unicycle on world 008's true geometry, a gridmap "Nav2" at 0.30 m/s that aborts without a path or progress, the raycast mapper): exploration with checkpoints at the exact sim times, a 3-goal tour verified by `verify_missions.py`, a rejected first send, an unreachable goal, a stuck robot (TIMEOUT at the sim timeout), a stalled sim clock, a stop signal | `failures: 0` |
| `python3 ground_mock_test.py 6 amcl` | the mission executive's real command line with `--vehicle ground` on `ground_stub.py`'s unicycle (the real `GroundLink` controller; a turn in place below 0.2 rad/s moves nothing, as the simulated drive; odometry = truth scaled 0.8 and rotated 2°, AMCL = truth + 3 cm): six goals through the doorways nose-first, guarded return; the command rule (nothing commanded below 0.10 m/s or 0.5 rad/s) | `TOUR DONE: 6/6`, `home_reached`, `exit 0`, `turns in place below 0.5 rad/s: 0, forward below 0.10 m/s: 0` |
| `GROUND=1 BUDGETS=1,2.5 python3 mock_explore_test.py` | the exploration loop on the same unicycle | `explore_done reason=budget`, `home_reached`, `exit 0`, the command rule |
| `python3 slot_test.py` | the lateral-clearance verdict on the 0.61 m slot, the 0.4 m pocket and the three doors | slot/pocket < 0.70 m, doors ≥ 0.85 m |

Also run `python3 ../cli_check.py` before every batch. Environment knobs of the mission
mock: `NARROW` (narrow lookahead, 0 = doorway strategy off), `CENTER`, `REPEL`, `AMCL_BIAS`,
`LATE_AMCL`; of the exploration mock: `BUDGETS`.
