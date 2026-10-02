# worlds/

The 10 seeded four-room worlds shared by both embodiments (20 × 20 m arena, cross
partition with one door per half-wall, controlled door widths, seeded furniture; walls
and furniture 2.0 m tall, no ceiling). Fixed list (chosen 2026-09-05):

| Seed | Spawn room | Spawn exits (m) | Rooms reachable from spawn at 0.66 / 0.75 / 0.85 m |
|---|---|---|---|
| 20260723001 | NW | 0.55 / 0.67 | 4 / 1 / 1 |
| 20260723002 | SW | 0.62 / 0.89 | 4 / 3 / 3 |
| 20260723003 | SE | 0.79 / 0.95 | 3 / 3 / 2 |
| 20260723004 | NE | 0.78 / 0.83 | 4 / 3 / 1 |
| 20260723005 | NW | 0.56 / 0.91 | 3 / 3 / 2 |
| 20260723008 | NE | 0.76 / 0.91 | 4 / 4 / 2 |
| 20260723013 | NW | 0.73 / 0.74 | 4 / 1 / 1 |
| 20260723016 | SE | 0.77 / 0.93 | 4 / 4 / 3 |
| 20260723018 | SW | 0.51 / 0.67 | 4 / 1 / 1 |
| 20260723023 | SW | 0.66 / 0.77 | 4 / 4 / 1 |

Selection rule: stratified by rooms reachable from the spawn room at the aerial door
threshold (0.75 m), in proportion to the 25-world population; within strata, spawn-room
balance and availability of the predecessor's verified wheeled-robot data. The 0.66 / 0.75 / 0.85 m
thresholds encode the door-passability assumption (wheeled physical minimum /
quadrotor sideways / comfortable) made before the benchmark flights.

Contents:

- `manifests/world_<seed>.csv`: the ground-truth manifest of each world (`type,name,x,y,param1,param2,yaw`; box/cyl rows are geometry at z = 1.0, height 2.0; `door` rows carry the true door widths; `person`/`path`/`xdoor` rows are ignored in this benchmark: pedestrians are absent). The flight apps rebuild a world 1:1 from this file.
- `maps/world_<seed>.pgm/.yaml`: the predecessor's frozen 2D maps (0.05 m/px), used for developing and testing the A* executive before the exploration pipeline produces its own maps. Benchmark missions run on the maps frozen by the budgeted exploration, never on these.
- `plot_worlds_overview.py`, `v1_worlds_stats.csv`, `v1_worlds_overview.png`: decision support for the list above, over the 25 candidate worlds of v1 (the predecessor project, [sim-nav-benchmark](https://github.com/d1mitrios/sim-nav-benchmark)): a 5 × 5 montage and one row of statistics per candidate world (door widths, spawn room, reachability by footprint threshold, clutter, baseline results).
