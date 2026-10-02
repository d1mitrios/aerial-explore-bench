#!/usr/bin/env bash
# === Wheeled arm, navigation bring-up (WSL): the baseline's Nav2 for one run ===
# The baseline's nav_batch.sh / slam_batch.sh pattern (a fresh stack per run, bounded
# readiness probes, DDS hygiene on every stop) with this repo's copies of its configs.
# Foreground: shows readiness, prints one line "[wnav] READY ..." (or "[wnav] FAILED ...",
# exit 1) for the batch, then keeps the stack up until Ctrl+C / SIGTERM.
#
#   bash missions/wheeled_nav.sh <seed> explore            slam_toolbox (online_async,
#       policies/slam/slam_params.yaml; the mapper of both embodiments) + Nav2's
#       navigation servers (navigation_launch.py, policies/nav2/nav2_params.yaml: the predecessor's v9 + corrections) whose
#       global costmap follows the live /map: the mover of wheeled_explore_runner.py
#   bash missions/wheeled_nav.sh <seed> tour <map.yaml>    Nav2 as the baseline ran its tours
#       (bringup_launch.py: map_server + AMCL + navigation, the same params) on a frozen map
#   AEB_NAV2_PARAMS=<file>  another params file for either mode (batch_wheeled.sh --nav2-params;
#       the footprint sensitivity variant policies/nav2/nav2_params_polygon.yaml)
#
# The map travels inside a per-run copy of the params (map_server.yaml_filename): on the
# installed nav2_bringup the `map:=` launch argument did not reach map_server
# (aerial_amcl.sh, 2026-09-23); /map is verified after activation, load_map is the fallback.
# Gates: explore = the slam_toolbox node, bt_navigator active, /map; tour = bt_navigator
# active, /map, the first /amcl_pose (the baseline's gate 2). Needs the Isaac run up
# (sim/launch_wheeled.ps1: /scan, /odom + TF). Logs in $AEB_LOG_DIR (the batch: the run
# directory) or runs/raw: wnav_<mode>_<seed>_<ts>.log (+ slam_<seed>_<ts>.log).
# (no `set -u`: ROS's setup.bash references unset variables and would abort under it)
if [ $# -lt 2 ]; then echo "usage: $0 <seed> explore | <seed> tour <map.yaml>"; exit 2; fi
SEED="$1"; MODE="$2"; MAP="$3"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
NAV_SRC="${AEB_NAV2_PARAMS:-$REPO/policies/nav2/nav2_params.yaml}"
SLAM_PARAMS="$REPO/policies/slam/slam_params.yaml"
LOGDIR="${AEB_LOG_DIR:-$REPO/runs/raw}"
TS="$(date +%Y%m%d_%H%M%S)"
LOG="$LOGDIR/wnav_${MODE}_${SEED}_${TS}.log"
SLAM_LOG="$LOGDIR/slam_${SEED}_${TS}.log"
PARAMS="$LOGDIR/nav2_params_${SEED}_${TS}.yaml"
mkdir -p "$LOGDIR"
case "$MODE" in
  explore) ;;
  tour) [ -n "$MAP" ] && [ -f "$MAP" ] || { echo "[wnav] FAILED map not found: $MAP"; exit 2; }
        MAP="$(readlink -f "$MAP")" ;;
  *) echo "usage: $0 <seed> explore | <seed> tour <map.yaml>"; exit 2 ;;
esac
[ -f "$NAV_SRC" ] || { echo "[wnav] FAILED params not found: $NAV_SRC"; exit 2; }
[ -f "$SLAM_PARAMS" ] || { echo "[wnav] FAILED params not found: $SLAM_PARAMS"; exit 2; }
source /opt/ros/jazzy/setup.bash

STACK_RE="async_slam_toolbox_node|sync_slam_toolbox_node|online_async_launch|nav2_amcl|nav2_map_server|lifecycle_manager|localization_launch|navigation_launch|bringup_launch|component_container_isolated|nav2_container|controller_server|planner_server|smoother_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|collision_monitor|docking_server"

dds_hygiene() {
  ros2 daemon stop > /dev/null 2>&1
  rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* 2> /dev/null
  sleep 1
}

stop_group() {  # pid: INT, TERM, KILL on the launch's process group
  local pid="$1"; [ -n "$pid" ] || return 0
  kill -INT -- -"$pid" 2> /dev/null
  for _ in $(seq 1 15); do kill -0 "$pid" 2> /dev/null || break; sleep 1; done
  kill -TERM -- -"$pid" 2> /dev/null
  for _ in $(seq 1 8); do kill -0 "$pid" 2> /dev/null || break; sleep 1; done
  kill -KILL -- -"$pid" 2> /dev/null
  wait "$pid" 2> /dev/null
}

teardown() {
  echo "[wnav] $(date +%H:%M:%S) stopping the stack ..."
  stop_group "$NAV"
  stop_group "$SLAM"
  pkill -9 -f "$STACK_RE" 2> /dev/null            # orphans hold /map and the action server
  dds_hygiene
  echo "[wnav] down + dds clean (log: $LOG)"
}

fail() { echo "[wnav] FAILED $*"; teardown; exit 1; }

map_published() {  # /map is latched; the width prints as a bare integer
  timeout 10 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable \
      --field info.width /map 2> /dev/null | grep -Eq '^[0-9]+'
}

bt_active() {
  timeout 6 ros2 lifecycle get /bt_navigator 2> /dev/null | grep -q "^active"
}

# the run's params = the committed ones, with the map path for map_server in tour mode
if [ "$MODE" = tour ]; then
  MAP_SED="$(printf '%s' "$MAP" | sed 's/[&|\\]/\\&/g')"
  sed -E "s|^([[:space:]]*yaml_filename:).*|\1 \"$MAP_SED\"|" "$NAV_SRC" > "$PARAMS"
  grep -qF "yaml_filename: \"$MAP\"" "$PARAMS" || { echo "[wnav] FAILED could not write the map path into $PARAMS"; exit 2; }
else
  cp "$NAV_SRC" "$PARAMS"
fi

pkill -9 -f "$STACK_RE" 2> /dev/null               # never two localizers on /map, never a stale server
dds_hygiene
BRINGUP_DIR="$(ros2 pkg prefix nav2_bringup 2> /dev/null)/share/nav2_bringup/launch"
{
  echo "# wheeled_nav.sh $(date -Is) seed=$SEED mode=$MODE${MAP:+ map=$MAP}"
  echo "# params=$PARAMS (from $NAV_SRC)"
  echo "# ros-jazzy-nav2-bringup: $(dpkg-query -W -f='${Version}' ros-jazzy-nav2-bringup 2> /dev/null || echo 'not a deb package')"
  echo "# ros-jazzy-slam-toolbox: $(dpkg-query -W -f='${Version}' ros-jazzy-slam-toolbox 2> /dev/null || echo 'not a deb package')"
  echo "# ---"
} > "$LOG"
echo "[wnav] $(date +%H:%M:%S) world $SEED, $MODE${MAP:+, map $MAP}; log $LOG"
SLAM=""; NAV=""
trap 'teardown; exit 0' INT TERM

if [ "$MODE" = explore ]; then
  { echo "# slam_toolbox for wheeled_nav.sh $(date -Is) seed=$SEED params=$SLAM_PARAMS"; echo "# ---"; } > "$SLAM_LOG"
  setsid ros2 launch slam_toolbox online_async_launch.py slam_params_file:="$SLAM_PARAMS" use_sim_time:=false >> "$SLAM_LOG" 2>&1 &
  SLAM=$!
  up=0
  for _ in $(seq 1 20); do
    if timeout 6 ros2 node list 2> /dev/null | grep -q "slam_toolbox"; then up=1; break; fi
    kill -0 "$SLAM" 2> /dev/null || fail "slam_toolbox launch died - see $SLAM_LOG"
    sleep 3
  done
  [ "$up" = 1 ] || fail "slam_toolbox node not visible after 60 s - see $SLAM_LOG"
  echo "[wnav] $(date +%H:%M:%S) slam_toolbox up (mapping, 0.05 m, 12 m)"
  setsid ros2 launch nav2_bringup navigation_launch.py params_file:="$PARAMS" use_sim_time:=false autostart:=true >> "$LOG" 2>&1 &
  NAV=$!
else
  setsid ros2 launch nav2_bringup bringup_launch.py map:="$MAP" params_file:="$PARAMS" use_sim_time:=false autostart:=true >> "$LOG" 2>&1 &
  NAV=$!
fi

# gate: bt_navigator ACTIVE (every probe bounded; the baseline's gate 1)
up=0
for _ in $(seq 1 30); do
  if bt_active; then up=1; break; fi
  kill -0 "$NAV" 2> /dev/null || fail "Nav2 launch died - see $LOG"
  sleep 4
done
[ "$up" = 1 ] || fail "bt_navigator not active after 120 s - see $LOG"
echo "[wnav] $(date +%H:%M:%S) Nav2 active (bt_navigator)"

# gate: the map (explore: slam_toolbox's first map, once the odometry TF and a scan exist;
# tour: map_server's, else the load_map service)
ok=0
for _ in $(seq 1 12); do map_published && { ok=1; break; }; sleep 3; done
if [ "$ok" != 1 ] && [ "$MODE" = tour ]; then
  echo "[wnav] no /map after activation - loading it through map_server's load_map ..."
  RESP="$(timeout 40 ros2 service call /map_server/load_map nav2_msgs/srv/LoadMap "{map_url: '$MAP'}" 2>&1)"
  grep -q "result=0" <<< "$RESP" && map_published && { ok=1; echo "# load_map fallback used: result=0" >> "$LOG"; }
fi
[ "$ok" = 1 ] || fail "no /map - see $LOG${SLAM:+ and $SLAM_LOG}"
echo "[wnav] $(date +%H:%M:%S) /map published"

# gate (tour): the first /amcl_pose = AMCL active and scans + odometry flowing (the baseline's gate 2)
if [ "$MODE" = tour ]; then
  ok=0
  for _ in $(seq 1 20); do
    if timeout 8 ros2 topic echo --once --field pose.pose.position.x /amcl_pose > /dev/null 2>&1; then ok=1; break; fi
    kill -0 "$NAV" 2> /dev/null || fail "Nav2 launch died - see $LOG"
  done
  [ "$ok" = 1 ] || fail "no /amcl_pose (AMCL not localized) - see $LOG"
  echo "[wnav] $(date +%H:%M:%S) AMCL localized (/amcl_pose)"
fi

echo "[wnav] READY $MODE world $SEED"
echo "[wnav] running - Ctrl+C to stop (node logs: $LOG${SLAM:+, $SLAM_LOG})"
while kill -0 "$NAV" 2> /dev/null; do
  if [ -n "$SLAM" ] && ! kill -0 "$SLAM" 2> /dev/null; then echo "[wnav] slam_toolbox exited - see $SLAM_LOG"; break; fi
  sleep 2
done
echo "[wnav] $(date +%H:%M:%S) a launch exited by itself"
teardown
exit 1
