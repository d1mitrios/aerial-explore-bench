#!/usr/bin/env python
"""
| File: a7_lidar_flight.py
| Description: A7, the quadrotor carrying the benchmark lidar. The A6 flight (seeded
|              world, external PX4 SITL, VIO sensors, 10 Hz out-of-band ground truth) plus
|              the raycast 2D lidar publishing /scan at ~20 Hz (sim/lidar_raycast.py).
|              The batch runs launch it (sim/launch_a7.ps1); it also verified the lidar port
|              in flight and measured the real-time factor with the full sensor suite.
|
| Launch (PowerShell: the ROS env must be set BEFORE python.bat; see sim/README.md):
|   $env:ROS_DOMAIN_ID="0"
|   $env:RMW_IMPLEMENTATION="rmw_fastrtps_cpp"
|   $env:FASTRTPS_DEFAULT_PROFILES_FILE="<repo>\\sim\\fastdds_win.xml"
|   $env:PATH="$env:PATH;C:\\isaacsim\\exts\\isaacsim.ros2.core\\humble\\lib"
|   $env:AEB_WORLD_SEED="20260723001"
|   C:\\isaacsim\\python.bat -u <repo>\\sim\\a7_lidar_flight.py
|
| Options (environment):
|   AEB_CAMERA=0            IMU without the camera, the benchmark configuration since 2026-09-24
|                           (lidar odometry): the camera is what pulls the RTF from ~0.4
|                           down to ~0.17; the IMU feeds the executive's impact metrics
|   AEB_VIO=0               skip the camera/IMU kit entirely (lidar-only run, no IMU)
|   AEB_LIDAR_STABILIZE=0   body-fixed scan plane instead of the stabilized (yaw-only) default
|   AEB_HEADLESS=1          headless Isaac (RTF measurement)
|
| Outputs (in <repo>/runs/raw, git-ignored): a7_gt_<seed>_<ts>.csv (ground truth, both
| clocks) and a7_scans_<seed>_<ts>.csv (every 20th scan with its true origin); check
| with: python3 analysis/plot_lidar_check.py --scans <that csv>
|
| PX4 side (WSL2), only after the drone is visible; never probe port 4560 by hand:
|   pkill px4 2>/dev/null; sleep 2; cd ~/px4_run && PX4_SIM_HOSTNAME=172.19.80.1 \\
|   PX4_SIM_MODEL=gazebo-classic_iris ~/PX4-Autopilot/build/px4_sitl_default/bin/px4 \\
|   ~/PX4-Autopilot/ROMFS/px4fmu_common/ -s ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/rcS -i 0
|   pxh> commander takeoff   (MIS_TAKEOFF_ALT 1.4 / NAV_MC_ALT_RAD 0.25 persist in ~/px4_run)
"""

import os

import aeb_flight

# must precede every other isaacsim import; a VIO run without ROS is refused outright
simulation_app = aeb_flight.start_simulation_app(require_ros=os.environ.get("AEB_VIO", "1") != "0"
                                                 or os.environ.get("AEB_REQUIRE_ROS", "0") == "1")

from aeb_flight import FlightApp, GroundTruthLogger, VioSensorKit  # noqa: E402
from lidar_raycast import RaycastLidar2D  # noqa: E402


def main():
    app = FlightApp(tag="a7")
    app.add(GroundTruthLogger())
    if os.environ.get("AEB_VIO", "1") != "0":
        app.add(VioSensorKit(camera=os.environ.get("AEB_CAMERA", "1") != "0"))
    app.add(RaycastLidar2D(stabilize=os.environ.get("AEB_LIDAR_STABILIZE", "1") != "0"))
    app.run()


if __name__ == "__main__":
    main()
