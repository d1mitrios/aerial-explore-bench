# Installing Pegasus Simulator on Isaac Sim 6.0.1 (Windows 11 + WSL2)

This guide sets up the quadrotor side of the benchmark: Pegasus Simulator v5.1.0 with a local port to Isaac Sim 6.0.1 on Windows, PX4 v1.14.3 SITL and ROS 2 Jazzy in WSL2, and OpenVINS for the optional VIO odometry. The pinned versions and the reasons for them are in `VERSIONS.md`; v1 is the predecessor project, [sim-nav-benchmark](https://github.com/d1mitrios/sim-nav-benchmark), whose Isaac setup this one extends. Every step was verified on the benchmark machine on the date given.

The steps are labelled A1 to A6. The benchmark's own flight app continues the numbering as A7 (`sim/a7_lidar_flight.py`), and the VIO topics keep the A6 prefix (`/a6/imu`, `/a6/cam0/image_raw`).

Verified end to end: a PX4-controlled hover and landing (2026-08-23), a scripted flight inside a seeded world (2026-08-30), OpenVINS running live on the simulation (2026-08-30) and a VIO drift plot against ground truth (`analysis/vio_drift_seed20260723001.png`).

---

## Machine and network (surveyed 2026-08-21)

- Windows 11 build 26200 · WSL2 2.7.8 · Ubuntu 24.04.4 · ROS 2 Jazzy · `rmw_fastrtps_cpp` · driver 610.88 (616.92 since 2026-09-24, see `VERSIONS.md`) · GPU visible from WSL.
- Isaac Sim 6.0.1: `C:\isaacsim` (untouched), the version v1 runs on the same machine. The ROS 2 environment of the Isaac side is set by `sim/env.ps1`.
- Bridge = v1's deliberate NAT recipe: no `.wslconfig`, no portproxy; UDPv4-only FastDDS profiles on **both** sides: Windows `sim/fastdds_win.xml` (set by `sim/env.ps1`), WSL `~/.ros/fastdds.xml` with initial peer **172.19.80.1** (host address on the WSL vswitch). The recipe is reused unchanged. If topics ever vanish, first check that gateway IP hasn't changed (`ip route | grep default` in WSL).
- v1 constraints that still apply: Isaac on Windows only (no WSL2 Vulkan) · no driver change during the benchmark (`VERSIONS.md`) · all Isaac-side files local on `C:\` (never `\\wsl.localhost\...`).

---

## Setup: Isaac 6.0.1 on Windows + PX4/ROS in WSL2 (pinned 2026-08-23)
*Same topology and same Isaac version as v1. Rationale: `VERSIONS.md`.*

### A1 + A2: Pegasus v5.1.0 on Isaac 6.0.1 (2026-08-23)
- Isaac 5.1.0 (the first A1) does not run on this machine: driver-incompatible (`VERSIONS.md`, note 11); `C:\isaacsim-5.1.0` is not used
- Pegasus v5.1.0 cloned to `C:\PegasusSimulator` (commit 644da37), pip-installed into 6.0.1's Python (`ISAACSIM_PATH=C:\isaacsim`), locally ported to Isaac 6: `extension.toml` deps → `isaacsim.*` · camera import rename · unused import removed · `logic/vehicles/dc_shim.py` replaces the removed Dynamic Control (10/10 dc methods covered)
- Probe result: extension loads, vehicle spawns, **Play runs clean**

Launch (two modes):

- **Standalone `python.bat` app, the working path for flight (A4/A5, and the batch runner):** `C:\isaacsim\python.bat -u C:\PegasusSimulator\examples\<script>.py`; the benchmark's app is `sim/a7_lidar_flight.py` (see `sim/README.md`)
- GUI extension (loads + Plays, useful for scene work; PX4 flight is **not** possible here on 6.0.1: physics callbacks never fire): `C:\isaacsim\isaac-sim.bat --ext-folder C:\PegasusSimulator\extensions --enable pegasus.simulator`

### A3. PX4 v1.14.3 SITL (WSL2), 2026-08-23
1. Clone, check out `v1.14.3`, initialize the submodules; run dir `~/px4_run`
2. Dependencies: `PIP_BREAK_SYSTEM_PACKAGES=1 bash ./Tools/setup/ubuntu.sh` (PEP 668 on Noble)
3. Build: `CC=gcc-12 CXX=g++-12 make px4_sitl_default none` (gcc-13 breaks v1.14.3)

### A4. The boundary link: first PX4-controlled hover + land (2026-08-23)
Pegasus is the TCP **server**; **PX4 connects out to it**. Networking stays as v1 left it: NAT, no mirrored mode, no portproxy.

**Verified procedure** (standalone app; the GUI flow cannot fly on 6.0.1: its physics callbacks never fire):

1. PowerShell (`a4_hover.py` is the hover test app in the local Pegasus port's `examples` folder; the benchmark's `sim/a7_lidar_flight.py`, started as in `sim/README.md`, runs the same loop and prints the same markers with the `[a7]` tag):

```powershell
C:\isaacsim\python.bat -u C:\PegasusSimulator\examples\a4_hover.py *>&1 | Tee-Object -FilePath C:\PegasusSimulator\a4_log.txt
```

   Wait until the quadrotor is visible. Expected `[a4]` markers in the log: `pre-applied PhysxForceAPI on ...` (×5) · `pre-applied propeller velocity drive on ...` (×4) · `control at physics rate: dt=0.0040s x 4 substeps per rendered frame`.

2. WSL, only after the quadrotor is visible, and never probe port 4560 by hand (it consumes the single client slot):

```bash
pkill px4 2>/dev/null; sleep 2; cd ~/px4_run
PX4_SIM_HOSTNAME=172.19.80.1 PX4_SIM_MODEL=gazebo-classic_iris \
  ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 ~/PX4-Autopilot/ROMFS/px4fmu_common/ \
  -s ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS -i 0
```

   **No `-d`**: daemon mode disables the interactive `pxh>` console.

3. At `pxh>` after `Ready for takeoff`: `commander takeoff` → stable hover at ~2.5 m → `commander land`.

- Success criteria met (2026-08-23): heartbeat received, EKF converges on the shim's finite-difference sensors, lockstep runs, takeoff / hover / land clean
- Firewall, if the connection ever blocks again: the culprit was **ESET Endpoint Security**, not Windows Firewall; permanent rule: allow inbound from remote hosts **172.19.80.0/20** (ESET rules evaluate top-down; keep it above any block rule). The Windows Firewall rule "Pegasus PX4 4560" also exists.
- Restart discipline: restart both sides together. A half-dead handshake leaves a zombie client slot (`CLOSE_WAIT` in `netstat -an | findstr 4560`); "PX4 server already running" in WSL needs `pkill px4`.

### A5. Flight in a v1 world, ROS 2 from WSL2 (2026-08-30)
- Scripted flight (arm, take off, hover, land) done as part of A4 (`examples/a4_hover.py` of the local port, Default Environment)
- Flight **inside a v1 world** (`examples/a5_world_flight.py` of the local port, 2026-08-30): world `20260723001` rebuilt 1:1 from its seed manifest (boundary walls as v1's `build_arena.py` builds them + all box/cyl rows; person rows skipped; door/gate rows are metadata, not geometry). The same rebuild is in `sim/aeb_flight.py`, where `AEB_WORLD_SEED` or `AEB_WORLD_CSV` picks the world. Result: hover **1.21–1.23 m physical, EKF 1.20 m**; agreement ≤ 3 cm; propellers spinning; zero loop errors
- Flight-altitude sanity check: at the flight altitude (1.2–1.5 m) the quadrotor flies below the 2.0 m walls/furniture; geometry binds, no overflight. No generator changes needed
- **pxh prelude for takeoff tests** (both persist in `~/px4_run`): `param set MIS_TAKEOFF_ALT 1.4` and `param set NAV_MC_ALT_RAD 0.25`; `commander takeoff` completes within NAV_MC_ALT_RAD (default **0.8 m**!) of the target and Holds there; with defaults the quadrotor stalls at target−0.8 (observed 0.66 m on 2026-08-27). Offboard missions are unaffected
- ROS 2 from WSL2 (2026-08-30, probe topic verified end-to-end). The standalone recipe, which `sim/env.ps1` sets: v1's env pattern (`ROS_DOMAIN_ID=0` · `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`) **plus** `FASTRTPS_DEFAULT_PROFILES_FILE=<repo>\sim\fastdds_win.xml` (clean UDPv4-only profile; v1's on-disk `fastdds.xml` carries an unfilled `<address>` placeholder = malformed XML) **plus** `PATH += C:\isaacsim\exts\isaacsim.ros2.core\humble\lib` (else LoadLibrary error 126, "ROS2 Bridge startup failed"). Isaac 6.0.1 ships internal libs for both **humble** (default) and **jazzy**
- Lidar: **no RTX lidar** (`VERSIONS.md`, note 9): v1's custom raycast publisher is the single lidar model for both embodiments

### A6. VIO in WSL2: OpenVINS running live on the simulation (2026-08-30)
Since 2026-09-24 the benchmark's odometry is rf2o lidar odometry (`VERSIONS.md`); OpenVINS remains available as `--odom-source vio`.

- OpenVINS built at the pinned commit (`nathanshankar/open_vins` @ `0bd027a99...`, recorded in `VERSIONS.md`); `colcon build --packages-up-to ov_msckf`, 3 packages, no failures
- Sensor streaming from the standalone app (`examples/a6_vio_flight.py` of the local port; `VioSensorKit` in `sim/aeb_flight.py` carries the same sensors): camera mono8 640×480 ~20 Hz + IMU 250 Hz + GT→CSV 10 Hz (out-of-band). No `/clock`: everything is stamped with the shared sim clock and joins on timestamps. **Images must be RELIABLE**: `ov_msckf` subscribes reliable; a best-effort publisher delivers zero frames (silent DDS incompatibility)
- Run: `ros2 run ov_msckf run_subscribe_msckf <repo>/policies/openvins/estimator_config.yaml` (WSL reads the configs through /mnt/c) + `analysis/vio_to_csv.py` recording `/poseimu`. Start both BEFORE takeoff: initialization fires on the takeoff jolt
- Throughput measured, not fixed: with the camera on, the sim runs at RTF ≈ 0.21; IMU arrives at ~52 msg/s wall (bursts of 4), images ~4–5/s wall; lockstep keeps it all consistent in sim time
- Config deviations from template defaults (each forced by a measured failure, documented in the yamls): init thresholds (smooth-takeoff excitation ~0.11), FAST 7 / min_px_dist 10 (flat-shaded worlds hold few corners), gravity 9.80665
- **Drift plot:** `analysis/vio_drift_seed20260723001.png` (`analysis/plot_vio_drift.py`, data in `analysis/data/`): 20 s window, final drift 0.42 m, RMSE 0.39 m, monocular scale 0.70×, similarity-aligned ATE 0.04 m. The fallback (a VINS-Fusion fork) was not needed

---

## Alternatives not used

- Isaac inside WSL2: tested during v1's migration: **no Vulkan in WSL2: "Isaac on Windows only."**
- Dual-boot native Ubuntu 24.04: the only environment Pegasus is officially tested on; it costs a day of disk surgery plus re-setup and was not needed.

---

## v1 gotchas that carry over
Script Editor: use the **Run button, once**; never Ctrl+Enter (double-fires; a duplicate `/scan` publisher means restarting Isaac) · save the stage only while **STOPPED** (saving during Play once wrote 0/0 timecodes) · after any structural edit verify `root_joint` is still deactivated (symptom: perfect cmd_vel chain, zero errors, robot bolted in place) · caster/wheel physics materials are load-bearing: caster friction is the #1 dynamics trap · `ArticulationController` robotPath must point at the prim holding `ArticulationRootAPI` · obstacles must never sit under the robot prim (raycast self-filter) · watch VRAM: v1 saw a hard freeze at ~8.3/11 GiB; close GPU-hungry background apps before batches.
