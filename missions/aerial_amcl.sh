#!/usr/bin/env bash
# === Aerial localization bring-up (WSL): map_server + AMCL on one world's frozen map ===
# Pattern of the wheeled baseline's nav bring-up (bounded readiness probes, DDS hygiene
# on every stop). Foreground: shows readiness, then keeps the stack up until Ctrl+C.
#
#   bash missions/aerial_amcl.sh <seed> [map.yaml]
#
# Defaults: map = worlds/maps/world_<seed>.yaml (the baseline's frozen map, for the
# executive tests); in the benchmark the map frozen by the budgeted exploration is passed
# explicitly. Params: policies/nav2/amcl_aerial.yaml, written per run with the absolute
# map path into runs/raw/amcl_params_<seed>_<ts>.yaml; on the installed nav2_bringup the
# `map:=` launch argument did not reach map_server ("yaml-filename parameter is empty",
# test flight 2026-09-23), so the path travels inside the params file, /map is verified
# after activation and the load_map service is the fallback. Log: runs/raw/amcl_<seed>_<ts>.log
# (its first lines record the nav2_bringup version and how its launch file passes the map).
# Needs: ros-jazzy-nav2-bringup (the baseline used it), /scan from the Isaac app,
# TF odom->base_link from the odometry chain (missions/aerial_odom.sh = rf2o lidar odometry,
# or vio_odom_bridge.py with OpenVINS). AMCL publishes /amcl_pose only once the
# odometry exists; the executive gates on that, not this script.
# (no `set -u`: ROS's setup.bash references unset variables and would abort under it)
if [ $# -lt 1 ]; then echo "usage: $0 <seed> [map.yaml]"; exit 2; fi
SEED="$1"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
MAP="${2:-$REPO/worlds/maps/world_${SEED}.yaml}"
[ -f "$MAP" ] || { echo "[amcl] map not found: $MAP"; exit 2; }
MAP="$(readlink -f "$MAP")"                 # absolute: map_server resolves the .pgm next to it
PARAMS_SRC="$REPO/policies/nav2/amcl_aerial.yaml"
[ -f "$PARAMS_SRC" ] || { echo "[amcl] params not found: $PARAMS_SRC"; exit 2; }
LOGDIR="${AEB_LOG_DIR:-$REPO/runs/raw}"          # the batch runner puts each run's logs in its run directory
TS="$(date +%Y%m%d_%H%M%S)"
LOG="$LOGDIR/amcl_${SEED}_${TS}.log"
PARAMS="$LOGDIR/amcl_params_${SEED}_${TS}.yaml"
mkdir -p "$LOGDIR"
source /opt/ros/jazzy/setup.bash

# the params file of this run = the committed one with the absolute map path filled in
MAP_SED="$(printf '%s' "$MAP" | sed 's/[&|\\]/\\&/g')"
sed -E "s|^([[:space:]]*yaml_filename:).*|\1 \"$MAP_SED\"|" "$PARAMS_SRC" > "$PARAMS"
grep -qF "yaml_filename: \"$MAP\"" "$PARAMS" || { echo "[amcl] could not write the map path into $PARAMS"; exit 2; }

dds_hygiene() {
  ros2 daemon stop >/dev/null 2>&1
  rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* 2>/dev/null
  sleep 1
}

teardown() {
  echo "[amcl] $(date +%H:%M:%S) stopping localization ..."
  kill -INT -- -"$NAV" 2>/dev/null
  for _ in $(seq 1 15); do kill -0 "$NAV" 2>/dev/null || break; sleep 1; done
  kill -TERM -- -"$NAV" 2>/dev/null
  for _ in $(seq 1 8); do kill -0 "$NAV" 2>/dev/null || break; sleep 1; done
  kill -KILL -- -"$NAV" 2>/dev/null
  wait "$NAV" 2>/dev/null
  pkill -9 -f "nav2_amcl|nav2_map_server|lifecycle_manager|localization_launch|component_container_isolated|nav2_container" 2>/dev/null
  dds_hygiene
  echo "[amcl] down + dds clean (log: $LOG)"
}

map_published() {
  # /map is latched (transient_local); the width prints as a bare integer
  timeout 10 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable \
      --field info.width /map 2>/dev/null | grep -Eq '^[0-9]+'
}

pkill -9 -f "nav2_amcl|nav2_map_server|lifecycle_manager|localization_launch|component_container_isolated|nav2_container" 2>/dev/null
dds_hygiene
echo "[amcl] $(date +%H:%M:%S) world $SEED  map=$MAP"
echo "[amcl] params=$PARAMS (from $PARAMS_SRC)  log=$LOG"
# provenance at the top of the log: which nav2_bringup, and how its launch file hands the map over
LAUNCH_FILE="$(ros2 pkg prefix nav2_bringup 2>/dev/null)/share/nav2_bringup/launch/localization_launch.py"
{
  echo "# aerial_amcl.sh $(date -Is) seed=$SEED"
  echo "# map=$MAP"
  echo "# params=$PARAMS"
  echo "# ros-jazzy-nav2-bringup version: $(dpkg-query -W -f='${Version}' ros-jazzy-nav2-bringup 2>/dev/null || echo 'not a deb package')"
  echo "# launch file: $LAUNCH_FILE"
  grep -n "yaml_filename\|param_rewrites\|'map'" "$LAUNCH_FILE" 2>/dev/null | sed 's/^/#   /'
  echo "# ---"
} > "$LOG"
setsid ros2 launch nav2_bringup localization_launch.py \
    map:="$MAP" params_file:="$PARAMS" use_sim_time:=false autostart:=true >> "$LOG" 2>&1 &
NAV=$!
trap 'teardown; exit 0' INT TERM

# gate 1: amcl ACTIVE (every probe bounded)
up=0
for _ in $(seq 1 24); do
  if timeout 6 ros2 lifecycle get /amcl 2>/dev/null | grep -q "^active"; then up=1; break; fi
  kill -0 "$NAV" 2>/dev/null || { echo "[amcl] launch died; see $LOG"; tail -20 "$LOG"; exit 1; }
  sleep 4
done
if [ "$up" != 1 ]; then
  echo "[amcl] $(date +%H:%M:%S) WARN: /amcl not active after 96 s; see $LOG"
  tail -20 "$LOG"
else
  echo "[amcl] $(date +%H:%M:%S) map_server + amcl ACTIVE (initial pose = spawn)."
fi

# gate 2: the map is really published (AMCL waits for it forever otherwise)
if map_published; then
  echo "[amcl] /map published by map_server ($MAP)"
else
  echo "[amcl] WARN: no /map after activation: map_server did not take the yaml; loading it through the load_map service ..."
  RESP="$(mktemp)"
  timeout 40 ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '$MAP'}" > "$RESP" 2>&1
  if grep -q "result=0" "$RESP" && map_published; then
    echo "[amcl] /map published through load_map"
    echo "# load_map fallback used: result=0" >> "$LOG"
    rm -f "$RESP"
  else
    echo "[amcl] FAIL: the map could not be loaded (load_map response below); AMCL will not localize"
    grep -v "data=" "$RESP" | tail -5
    cp "$RESP" "$LOGDIR/amcl_loadmap_${SEED}_${TS}.txt" 2>/dev/null; rm -f "$RESP"
  fi
fi

# gate 3 (informational, non-blocking): are scans visible from here?
if timeout 8 ros2 topic echo --once /scan --field header.frame_id >/dev/null 2>&1; then
  echo "[amcl] /scan flowing from the Isaac side"
else
  echo "[amcl] WARN: no /scan seen in 8 s; is the Isaac app up with ROS (sim/env.ps1)?"
fi
echo "[amcl] /amcl_pose appears after VIO initialization (odom TF from vio_odom_bridge.py) + the first scan."
echo "[amcl] running; Ctrl+C to stop (tail -f $LOG for the node log)"
wait "$NAV"
teardown
