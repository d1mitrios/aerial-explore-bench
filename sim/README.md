# sim/

Isaac-side code: the standalone flight apps and sensor components that run inside
Isaac Sim 6.0.1 (Windows) with the local Pegasus Isaac-6 port and an external PX4
v1.14.3 SITL (WSL2). Apart from the Isaac Sim and Pegasus installs, everything here reads only
from this repository (`worlds/manifests/`, `sim/wheeled/`) and writes into `runs/raw/` or the run
directory it is given.

| File | What it is |
|---|---|
| `aeb_flight.py` | Shared scaffolding: SimulationApp + ROS 2 bridge bring-up, 1:1 world rebuild from a seed manifest, Iris on external PX4, physics-rate loop, component hooks; `GroundTruthLogger` (10 Hz out-of-band CSV, both clocks) and `VioSensorKit` (IMU 250 Hz + forward camera ~20 Hz for OpenVINS; `camera=False` = `AEB_CAMERA=0` keeps the IMU only: the benchmark configuration since the switch to lidar odometry on 2026-09-24) |
| `lidar_raycast.py` | The benchmark lidar on the quadrotor: 360-ray PhysX scene-query raycast at ~20 Hz → `/scan` (`sensor_msgs/LaserScan`), stabilized (yaw-only) scan plane by default, optional scan-sample CSV for the offline geometry check |
| `a7_lidar_flight.py` | A7 flight app, the one every aerial batch run launches (`launch_a7.ps1`): seeded world + PX4 + VIO sensors + ground truth + lidar. It also verified the lidar port and measured the RTF with the full sensor suite |
| `env.ps1` | The ROS 2 environment of the standalone `python.bat` apps, dot-sourced in their PowerShell window: domain 0, FastDDS with `fastdds_win.xml`, Isaac's internal Humble libraries on `PATH`, the app defaults (`AEB_WORLD_SEED`, `AEB_VIO`, `AEB_RUNS_DIR`); used by `launch_a7.ps1` and `link_probe.ps1`, never by the full app |
| `launch_a7.ps1` | The A7 app started detached for the batch runner (from WSL through `powershell.exe`): `env.ps1` + per-run `AEB_*` settings, a minimized console running `python.bat` with stdout+stderr to `<RunDir>\isaac.log`, prints `PID=<console pid>`; the app writes `isaac_status.txt` (state, frame, sim_t, rtf_now, PX4 heartbeat; every 50 frames) and stops cleanly on `isaac_stop` |
| `reap_isaac.ps1` | Ends a run's console tree (`taskkill /T /F`) and any process running from the Isaac install (never another Python) |
| `keep_awake.ps1` | Holds `SetThreadExecutionState(ES_CONTINUOUS\|ES_SYSTEM_REQUIRED)` while a batch runs (no power setting changed; ends by itself after `-MaxHours`) |
| `link_probe.py` | One-shot TCP listener on 0.0.0.0:4560 (Pegasus's port, no `SO_REUSEADDR`: a stale listener is reported, not shared) for the PX4 → Pegasus link test; run under Isaac's own python so the firewalls judge the flight's program. Prints `LISTENING`, then `ACCEPTED <peer>` / `TIMEOUT` / `PORT_IN_USE` |
| `link_probe.ps1` | Starts `link_probe.py` detached exactly as `launch_a7.ps1` starts the flight app (Start-Process of a `.cmd`, `python.bat`, output to a file); used by `missions/batch_aerial.sh` (pre-flight, `--link-check`, after a run without a PX4 heartbeat) |
| `fastdds_win.xml` | Windows-side FastDDS profile (UDPv4 only, no shared memory) so topics cross the Windows↔WSL2 boundary |
| `wheeled/` | The predecessor's Isaac-side files copied into the repo: the robot stage `robot.usda` with its payloads, the raycast lidar publisher (unchanged) and the odometry publisher (noise 0.03 / 0.05 / 0; only its file locations come from the environment); provenance and the exact changes in `wheeled/README.md` |
| `wheeled_bootstrap.py` | The wheeled arm's Isaac side, run by the full app at launch (`isaac-sim.bat --exec`, the way the predecessor starts it): opens `wheeled/robot.usda`, rebuilds the seed's world from its manifest under `/Arena` (the aerial app's rules; people skipped), starts the predecessor's two publishers, then the run services on the physics step: ground truth (`<RunDir>/wheeled_gt_<seed>_<ts>.csv`, 10 Hz, both clocks, the aerial format), `/aeb/sim_time` (`std_msgs/Float64`, the physics-step sum since Play: the WSL runners' budget and timeout clock) and the status / stop files; and presses Play. Status states `loading → playing → closing → closed`, or `error` with the reason |
| `launch_wheeled.ps1` | The wheeled run started detached (from WSL through `powershell.exe`): the ROS 2 variables set by the script itself (domain 0, FastDDS with `fastdds_win.xml`, Isaac's ROS library folders removed from `PATH`: the full app loads its own Jazzy libraries, and `env.ps1`'s Humble folder would break it) + `AEB_*` per run, a minimized console running `isaac-sim.bat --exec sim\wheeled_bootstrap.py` (window by default, `-Window 0` = Kit's `--no-window`; `-OdomTf 0` = no TF from the odometry publisher, the shared executive's runs) with stdout+stderr to `<RunDir>\isaac.log`, prints `PID=<console pid>` |
| `photo_scene.py` | A still scene for screenshots (`docs/images/`): the full app (`isaac-sim.bat --exec sim\photo_scene.py --/app/file/ignoreUnsavedOnExit=true`, a fresh PowerShell without `env.ps1`) opens `wheeled/robot.usda`, rebuilds `AEB_WORLD_SEED`'s world with `wheeled_bootstrap.py`'s builder, places the wheeled robot (`AEB_PHOTO_WHEELED` = `x,y,yaw_deg`) and Pegasus's Iris as a USD reference (`AEB_PHOTO_QUAD` = `x,y,z,yaw_deg`; `AEB_IRIS_USD` when Pegasus is not importable), aims the viewport at them and leaves the app open; no ROS, no Play, nothing saved. F10 captures the viewport |

## Launching an app (PowerShell)

The ROS 2 environment must be set **in the same PowerShell window, before** `python.bat`
starts: the bridge loads Isaac's internal Humble libraries from `PATH`, and without them
`rclpy` silently fails to import. `sim\env.ps1` sets everything; dot-source it once per
window, override what you need, launch:

```powershell
. C:\path\to\aerial-explore-bench\sim\env.ps1
$env:AEB_WORLD_SEED="20260723008"
$env:AEB_VIO="1"
$env:AEB_CAMERA="0"
C:\isaacsim\python.bat -u C:\path\to\aerial-explore-bench\sim\a7_lidar_flight.py *>&1 | Tee-Object -FilePath C:\path\to\aerial-explore-bench\runs\raw\a7_log.txt
```

The app logs `[aeb] ROS env: …` and `[aeb] rclpy import OK` at start; a run that needs
ROS (`AEB_VIO=1`, or `AEB_REQUIRE_ROS=1`) **refuses to fly** without it.

Environment variables understood by every app: `AEB_WORLD_SEED` (default `20260723001`),
`AEB_WORLD_CSV` (explicit manifest path), `AEB_RUNS_DIR` (default `runs/raw`, git-ignored),
`AEB_HEADLESS=1`, `AEB_STATUS_FILE` (a one-line status the app overwrites every 50 frames) and `AEB_STOP_FILE` (the app stops cleanly when it appears), both set by `launch_a7.ps1`. A7 adds `AEB_CAMERA=0` (IMU without the camera, the benchmark runs
since 2026-09-24: the odometry comes from the lidar, the camera was the RTF cost: ~0.17 with it,
~0.4 without), `AEB_VIO=0` (no camera/IMU kit at all) and `AEB_LIDAR_STABILIZE=0`
(body-fixed scan plane).

PX4 SITL is started by hand in WSL2 only after the quadrotor is visible (Pegasus is the TCP
server on 0.0.0.0:4560, PX4 the client with `PX4_SIM_HOSTNAME=<host vswitch IP>`); never
probe port 4560 by hand, and restart both sides together.

## Rules that break silently if ignored (Isaac 6.0.1)

- Standalone apps drive the vehicle pipeline **manually at physics rate** (250 Hz, 4
  substeps per rendered frame): the core-API physics callbacks never fire here, and
  driving at render rate flips the quadrotor.
- Every PhysX schema (force APIs, joint drives) is applied **before Play**.
- Images to `ov_msckf` are published RELIABLE; the IMU with sensor-data QoS.
- Ground truth is written to CSV from inside the app, **never** published as a topic.
- Scene-query rays must start **outside the vehicle's colliders**: the Iris body is a
  convex decomposition (many small solid pieces), and a ray born inside it reports self
  hits at distance ~0 until the skip logic gives up. The lidar origin therefore sits
  0.10 m above the body origin (first A7 flight: ~60 % of rays lost with the origin at
  +0.03 m; `selfblk` in the telemetry counts any recurrence).
