# sim/wheeled/: the predecessor's Isaac-side files, copied into this repo

The wheeled embodiment runs the predecessor's own robot and sensors. These are copies of the
predecessor's files, so the benchmark carries no runtime reference to a folder outside the
repo. `sim/wheeled_bootstrap.py` loads them (it is launched by `sim/launch_wheeled.ps1`).

| File | Source in the predecessor | Changes |
|---|---|---|
| `robot.usda` | `robot.usda` (the stage the predecessor's bootstrap opens) | (1) the four asset paths `../my_robot_1/payloads/…` now point to `./payloads/…`; (2) every `/Arena` child except the four `boundary_*` walls and the two lights is removed: the three `person_*` prims (the benchmark runs without pedestrians; they also referenced character assets on a remote server) and the last world the predecessor's generator had saved (the run rebuilds the world from `worlds/manifests/` anyway). The robot, its physics, the `/Graph/ROS_DiffDrive` OmniGraph (`cmd_vel` → DifferentialController, wheel distance 0.35 m, wheel radius 0.05 m), the ground plane and the physics scene are unchanged. |
| `payloads/` | `C:\my_robot_1\payloads\` (`base`, `robot`, `materials`, `Physics/{physics,physx,mujoco}.usda`), the folder the predecessor's stage resolved (`../my_robot_1/` next to its folder) | none |
| `scan_raycast_publisher2.py` | same name | none; the PhysX-raycast 2D lidar (360 rays, 0.1–100 m, ~20 Hz of sim time, frame `lidar_link`), the benchmark's single lidar model (the quadrotor carries its port, `sim/lidar_raycast.py`) |
| `odom_publisher.py` | same name (the predecessor's v5) | the two file locations (`NOISE_FILE`, `RUNS_DIR`) come from the environment (`AEB_ODOM_NOISE_FILE`, unset in the benchmark so the defaults 0.03 / 0.05 / 0 apply; `AEB_RUNS_DIR`, the run directory), plus the `import os` they need; `AEB_ODOM_TF=0` (`launch_wheeled.ps1 -OdomTf 0`) keeps the `/odom` topic and the log but sends no TF: the shared executive's runs take odom→base_link and base_link→lidar_link from rf2o + `lidar_odom_bridge.py`, and two publishers of one transform would fight |

The predecessor's other Isaac-side scripts are not used: the camera publisher (no camera in
the benchmark), the people mover (no pedestrians in the benchmark), the metrics logger (replaced by the ground-truth
logger of `sim/wheeled_bootstrap.py`, same 10 Hz lidar_link pose, both clocks, the
aerial CSV format) and its batch runner (one Isaac process per run here, as the aerial arm).

Provenance of the payloads: byte copies of `C:\my_robot_1\payloads\`, the files the
predecessor's stage loaded. (The predecessor's local work tree holds an earlier URDF import of the
same robot, made 94 s before it and identical except the temporary folder names in the `doc`
strings; checked 2026-09-26.)
