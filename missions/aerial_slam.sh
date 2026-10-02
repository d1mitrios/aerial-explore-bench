#!/usr/bin/env bash
# === Aerial mapping bring-up (WSL): slam_toolbox online_async for one exploration flight ===
# The baseline's slam_batch.sh pattern (fresh slam_toolbox per world, DDS hygiene on stop),
# for the FRONTIER3D exploration runner (missions/aerial_explore_runner.py). Foreground:
# shows readiness, then keeps the mapper up until Ctrl+C.
#
#   bash missions/aerial_slam.sh <seed>
#
# Params: policies/slam/slam_params.yaml (the baseline's, with the odometry-latency transform
# timeout documented in the file). Needs /scan from the Isaac app and TF odom->base_link from
# the odometry chain (missions/aerial_odom.sh = rf2o lidar odometry; or
# vio_odom_bridge.py --pose-topic /pose with OpenVINS). The map (/map, latched) and the
# corrected pose (/pose) appear once the odometry exists; the runner freezes the checkpoints
# itself from /map. Log: runs/raw/slam_<seed>_<ts>.log.
# (no `set -u`: ROS's setup.bash references unset variables and would abort under it)
if [ $# -lt 1 ]; then echo "usage: $0 <seed>"; exit 2; fi
SEED="$1"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
PARAMS="$REPO/policies/slam/slam_params.yaml"
LOGDIR="${AEB_LOG_DIR:-$REPO/runs/raw}"          # the batch runner puts each run's logs in its run directory
LOG="$LOGDIR/slam_${SEED}_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$LOGDIR"
[ -f "$PARAMS" ] || { echo "[slam] params not found: $PARAMS"; exit 2; }
source /opt/ros/jazzy/setup.bash

dds_hygiene() {
  ros2 daemon stop >/dev/null 2>&1
  rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* 2>/dev/null
  sleep 1
}

teardown() {
  echo "[slam] $(date +%H:%M:%S) stopping the mapper ..."
  kill -INT -- -"$SLAM" 2>/dev/null
  for _ in $(seq 1 10); do kill -0 "$SLAM" 2>/dev/null || break; sleep 1; done
  kill -TERM -- -"$SLAM" 2>/dev/null
  for _ in $(seq 1 5); do kill -0 "$SLAM" 2>/dev/null || break; sleep 1; done
  kill -KILL -- -"$SLAM" 2>/dev/null
  wait "$SLAM" 2>/dev/null
  pkill -9 -f "async_slam_toolbox_node|sync_slam_toolbox_node|online_async_launch" 2>/dev/null   # orphans hold /map
  dds_hygiene
  echo "[slam] down + dds clean (log: $LOG)"
}

pkill -9 -f "async_slam_toolbox_node|sync_slam_toolbox_node|online_async_launch" 2>/dev/null
pkill -9 -f "nav2_amcl|nav2_map_server|lifecycle_manager|localization_launch" 2>/dev/null   # never both localizers on /map
dds_hygiene
echo "[slam] $(date +%H:%M:%S) world $SEED  params=$PARAMS  log=$LOG"
{
  echo "# aerial_slam.sh $(date -Is) seed=$SEED"
  echo "# params=$PARAMS"
  echo "# ros-jazzy-slam-toolbox version: $(dpkg-query -W -f='${Version}' ros-jazzy-slam-toolbox 2>/dev/null || echo 'not a deb package')"
  echo "# ---"
} > "$LOG"
setsid ros2 launch slam_toolbox online_async_launch.py slam_params_file:="$PARAMS" use_sim_time:=false >> "$LOG" 2>&1 &
SLAM=$!
trap 'teardown; exit 0' INT TERM

# gate 1: the node exists (every probe bounded)
up=0
for _ in $(seq 1 20); do
  if timeout 6 ros2 node list 2>/dev/null | grep -q "slam_toolbox"; then up=1; break; fi
  kill -0 "$SLAM" 2>/dev/null || { echo "[slam] launch died; see $LOG"; tail -20 "$LOG"; exit 1; }
  sleep 3
done
if [ "$up" != 1 ]; then
  echo "[slam] $(date +%H:%M:%S) WARN: slam_toolbox node not visible after 60 s; see $LOG"
  tail -20 "$LOG"
else
  echo "[slam] $(date +%H:%M:%S) slam_toolbox up (mapping mode, 0.05 m, 12 m range)."
fi
# gate 2 (informational): scans visible from here?
if timeout 8 ros2 topic echo --once /scan --field header.frame_id >/dev/null 2>&1; then
  echo "[slam] /scan flowing from the Isaac side"
else
  echo "[slam] WARN: no /scan seen in 8 s; is the Isaac app up with ROS (sim/env.ps1)?"
fi
echo "[slam] /map and /pose appear once the odometry TF exists (aerial_odom.sh; with the VIO chain: after its initialization)."
echo "[slam] running; Ctrl+C to stop (tail -f $LOG for the node log)"
wait "$SLAM"
teardown
