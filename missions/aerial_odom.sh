#!/usr/bin/env bash
# === Aerial odometry bring-up (WSL): lidar odometry for the quadrotor ===
# Starts the bridge (missions/lidar_odom_bridge.py: /scan -> /scan_odom with misses zeroed,
# static TF base_link->lidar_link) and rf2o_laser_odometry (range-flow 2D odometry on
# /scan_odom -> /odom_rf2o + TF odom->base_link), then verifies that scans and odometry
# flow. Replaces the OpenVINS + vio_odom_bridge terminals of the VIO chain (used until 2026-09-24).
# Foreground: shows readiness, then keeps both up until Ctrl+C.
#
#   bash missions/aerial_odom.sh <seed>
#
# Needs: rf2o_laser_odometry built in ~/ros2_ws (VERSIONS.md: ros2 branch, commit b38c68e,
# C++17), /scan from the Isaac app (sim/env.ps1, AEB_CAMERA=0 for the benchmark RTF).
# Start BEFORE aerial_amcl.sh / aerial_slam.sh (they need the odom TF) and before the
# executive. Log: runs/raw/odom_<seed>_<ts>.log (rf2o's own output + the bridge's health lines).
# rf2o parameters: policies/odom/rf2o_aerial.yaml (the launch file's, with the scan topic,
# an empty init-pose topic and a 50 Hz wall loop; reasons in the file).
# AEB_LIDAR_Z: the bridge's static base_link->lidar_link height (default 0.10, the drone's
# lidar_raycast.py offset; the wheeled robot's lidar IS base_link: 0; the batch sets it).
# (no `set -u`: ROS's setup.bash references unset variables and would abort under it)
if [ $# -lt 1 ]; then echo "usage: $0 <seed>"; exit 2; fi
SEED="$1"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
PARAMS="$REPO/policies/odom/rf2o_aerial.yaml"
LOGDIR="${AEB_LOG_DIR:-$REPO/runs/raw}"          # the batch runner puts each run's logs in its run directory
LOG="$LOGDIR/odom_${SEED}_$(date +%Y%m%d_%H%M%S).log"
mkdir -p "$LOGDIR"
[ -f "$PARAMS" ] || { echo "[odom] params not found: $PARAMS"; exit 2; }
source /opt/ros/jazzy/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
if ! ros2 pkg executables rf2o_laser_odometry 2>/dev/null | grep -q rf2o_laser_odometry_node; then
  echo "[odom] rf2o_laser_odometry not found in ~/ros2_ws; build it first (VERSIONS.md)"; exit 2
fi

dds_hygiene() {
  ros2 daemon stop >/dev/null 2>&1
  rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* 2>/dev/null
  sleep 1
}

teardown() {
  echo "[odom] $(date +%H:%M:%S) stopping rf2o + bridge ..."
  for P in "$RF2O" "$BRIDGE"; do
    kill -INT -- -"$P" 2>/dev/null
  done
  for _ in $(seq 1 8); do kill -0 "$RF2O" 2>/dev/null || kill -0 "$BRIDGE" 2>/dev/null || break; sleep 1; done
  kill -KILL -- -"$RF2O" 2>/dev/null; kill -KILL -- -"$BRIDGE" 2>/dev/null
  wait "$RF2O" "$BRIDGE" 2>/dev/null
  pkill -9 -f "rf2o_laser_odometry_node|lidar_odom_bridge.py" 2>/dev/null
  dds_hygiene
  echo "[odom] down + dds clean (log: $LOG)"
}

pkill -9 -f "rf2o_laser_odometry_node|lidar_odom_bridge.py|vio_odom_bridge.py|run_subscribe_msckf" 2>/dev/null   # one odometry source
dds_hygiene
echo "[odom] $(date +%H:%M:%S) world $SEED  log=$LOG"
{
  echo "# aerial_odom.sh $(date -Is) seed=$SEED"
  echo "# rf2o: $(cd "$HOME/ros2_ws/src/rf2o_laser_odometry" 2>/dev/null && git rev-parse --short HEAD 2>/dev/null || echo 'commit unknown')  params=$PARAMS"
  echo "# ---"
} > "$LOG"
setsid python3 "$HERE/lidar_odom_bridge.py" --lidar-z "${AEB_LIDAR_Z:-0.10}" >> "$LOG" 2>&1 &
BRIDGE=$!
sleep 2
# rf2o prints "Waiting for laser_scans...." at every idle loop iteration (44 times a second
# at freq 50 between 6 Hz wall scans: 7.7 MB per flight), dropped from the log here
setsid bash -c "ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args -r __node:=rf2o_laser_odometry --params-file '$PARAMS' 2>&1 | grep --line-buffered -v 'Waiting for laser_scans'" >> "$LOG" 2>&1 &
RF2O=$!
trap 'teardown; exit 0' INT TERM

# gate 1: both processes alive and the rf2o node visible (every probe bounded)
up=0
for _ in $(seq 1 15); do
  kill -0 "$BRIDGE" 2>/dev/null || { echo "[odom] bridge died; see $LOG"; tail -20 "$LOG"; teardown; exit 1; }
  kill -0 "$RF2O" 2>/dev/null || { echo "[odom] rf2o died; see $LOG"; tail -20 "$LOG"; teardown; exit 1; }
  if timeout 6 ros2 node list 2>/dev/null | grep -q "rf2o_laser_odometry"; then up=1; break; fi
  sleep 2
done
[ "$up" = 1 ] && echo "[odom] $(date +%H:%M:%S) rf2o + bridge up" || echo "[odom] WARN: rf2o node not visible after 30 s; see $LOG"
# gate 2 (informational): scans from Isaac, then odometry
if timeout 8 ros2 topic echo --once /scan_odom --field header.frame_id >/dev/null 2>&1; then
  echo "[odom] /scan_odom flowing (scans from the Isaac side, misses zeroed)"
  if timeout 15 ros2 topic echo --once /odom_rf2o --field header.frame_id >/dev/null 2>&1; then
    echo "[odom] /odom_rf2o flowing; TF odom->base_link is up; start the localizer (aerial_amcl.sh / aerial_slam.sh)"
  else
    echo "[odom] WARN: no /odom_rf2o within 15 s of scans; see $LOG (rf2o prints 'Got first Laser Scan' on its first scan)"
  fi
else
  echo "[odom] WARN: no /scan_odom seen in 8 s; is the Isaac app up with ROS (sim/env.ps1)?"
fi
echo "[odom] running; Ctrl+C to stop (tail -f $LOG for rf2o + bridge output)"
wait "$RF2O"
teardown
