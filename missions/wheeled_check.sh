#!/usr/bin/env bash
# === Wheeled arm, bring-up check (WSL): one Isaac run of the wheeled robot, its ROS link, a short drive ===
#
#   bash missions/wheeled_check.sh [--window 0|1] [<seed>]        (default: world 20260723008, window 1)
#
# 1. pre-flight: ROS 2 Jazzy, the FastDDS peer (~/.ros/fastdds.xml) = the Windows host, DDS
#    hygiene (ros2 daemon stopped, stale FastDDS shared memory removed), any Isaac process
#    left over is ended (sim/reap_isaac.ps1 - this also closes an Isaac window you have open)
# 2. launches Isaac with sim/launch_wheeled.ps1 (the full app as the baseline runs it,
#    sim/wheeled_bootstrap.py: robot stage, the seed's world, the baseline's lidar and
#    odometry publishers, ground truth, /aeb/sim_time) into
#    runs/raw/wheeled_check/<seed>_<timestamp>/ and waits for state=playing
# 3. missions/wheeled_probe.py: /scan, /odom, /tf, /tf_static, /aeb/sim_time and the RTF; then
#    /cmd_vel 0.2 m/s for 5 sim-s and the ground truth's and the odometry's displacement
# 4. stops Isaac the way the batch will (stop file -> "closed" -> the app quits by itself; the
#    reaper only if it does not) and prints a summary (also in check.txt)
# --window 0 runs Kit with --no-window (to measure the RTF without the viewport).
# Ctrl+C: Isaac is stopped the same way before the script exits.
# (no `set -u`: ROS's setup.bash references unset variables)

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
PS="${AEB_PS:-/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe}"
TASKLIST="${AEB_TASKLIST:-/mnt/c/Windows/System32/tasklist.exe}"
WSLPATH="${AEB_WSLPATH:-wslpath}"
READY_S="${AEB_ISAAC_READY_S:-420}"      # the full app took 192 s to load on its first check
SEED=20260723008; WINDOW=1
while [ $# -gt 0 ]; do
  case "$1" in
    --window) WINDOW="$2"; shift ;;
    -h|--help) sed -n '2,20p' "$0"; exit 0 ;;
    -*) echo "unknown option $1 (--help)"; exit 2 ;;
    *) SEED="$1" ;;
  esac
  shift
done
case "$WINDOW" in 0|1) ;; *) echo "--window 0|1"; exit 2 ;; esac

RUN_DIR="$REPO/runs/raw/wheeled_check/${SEED}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$RUN_DIR" || exit 2
SUMMARY="$RUN_DIR/check.txt"
say() { echo "$*"; echo "$*" >> "$SUMMARY"; }
winpath() { "$WSLPATH" -w "$1"; }
ps_run() { "$PS" -NoProfile -ExecutionPolicy Bypass -File "$@" 2>&1 | tr -d '\r'; }
sfield() {  # file key -> value of key=value in the one-line status file (written on Windows: CRLF)
  [ -f "$1" ] || return 0
  tr -d '\r' < "$1" 2>/dev/null | tr ' ' '\n' | sed -n "s/^$2=//p" | head -1
}
win_alive() { [ -n "$1" ] && "$TASKLIST" /FI "PID eq $1" /NH 2>/dev/null | tr -d '\r' | grep -qw "$1"; }
isaac_reap() {  # [console pid]
  if [ -n "$1" ]; then ps_run "$(winpath "$REPO/sim/reap_isaac.ps1")" -ConsolePid "$1"
  else ps_run "$(winpath "$REPO/sim/reap_isaac.ps1")"; fi
}
dds_hygiene() {
  ros2 daemon stop > /dev/null 2>&1
  rm -f /dev/shm/fastrtps_* /dev/shm/fast_datasharing_* 2> /dev/null
}

ISAAC_PID=""; CLOSE_HOW="-"
isaac_stop() {  # the batch's order: stop file, wait for "closed", let Kit quit, then the reaper
  [ -n "$ISAAC_PID" ] || return 0
  touch "$RUN_DIR/isaac_stop"
  local t0=$SECONDS out="" i closed=0 st
  st="$(sfield "$RUN_DIR/isaac_status.txt" state)"
  if [ "$st" = playing ] || [ "$st" = closing ]; then   # the stop-file watcher runs from Play on
    while [ $((SECONDS - t0)) -lt 60 ]; do
      [ "$(sfield "$RUN_DIR/isaac_status.txt" state)" = closed ] && { closed=1; break; }
      win_alive "$ISAAC_PID" || break
      sleep 3
    done
    if [ "$closed" = 1 ]; then
      t0=$SECONDS
      while win_alive "$ISAAC_PID" && [ $((SECONDS - t0)) -lt 45 ]; do sleep 2; done
      if win_alive "$ISAAC_PID"; then CLOSE_HOW="closed, but Kit did not quit within 45 s (reaped)"
      else CLOSE_HOW="clean (closed, quit by itself in $((SECONDS - t0)) s)"; fi
    else
      CLOSE_HOW="no 'closed' state within 60 s (reaped)"
    fi
  else
    CLOSE_HOW="state ${st:-none} - no clean stop before Play (reaped)"
  fi
  for i in 1 2 3 4; do
    out="$(isaac_reap "$ISAAC_PID")"
    grep -q "ISAAC_PROCESSES=0" <<< "$out" && break
    sleep 5
  done
  grep -q "ISAAC_PROCESSES=0" <<< "$out" || CLOSE_HOW="$CLOSE_HOW; WARNING: Isaac processes left ($(grep -o 'ISAAC_PROCESSES=[0-9]*' <<< "$out"))"
  ISAAC_PID=""
}
trap 'echo; echo "interrupted - stopping Isaac"; isaac_stop; echo "Isaac: $CLOSE_HOW"; exit 130' INT TERM

isaac_lines() {  # the bootstrap's and the publishers' own lines from the Isaac log
  tr -d '\r' < "$RUN_DIR/isaac.log" 2>/dev/null | grep -E '\[wheeled\]|\[scan\]|\[odom\]|Traceback|Error|BOOT FAILED' | grep -v -i -E 'deprecat|warning:' | tail -n "${1:-25}"
}

# ------------------------------------------------------------------ 1. pre-flight
: > "$SUMMARY"
say "=== wheeled check: world $SEED, window $WINDOW, $(date '+%F %T')"
say "    run dir ${RUN_DIR#$REPO/}"
source /opt/ros/jazzy/setup.bash
bad=0
[ -f "$REPO/worlds/manifests/world_$SEED.csv" ] || { say "no world manifest for $SEED"; bad=1; }
[ -f "$REPO/sim/wheeled/robot.usda" ] || { say "sim/wheeled/robot.usda missing"; bad=1; }
python3 -c "import rclpy" 2> /dev/null || { say "rclpy not importable (ROS 2 Jazzy)"; bad=1; }
[ -x "$PS" ] || { say "Windows PowerShell not reachable at $PS"; bad=1; }
[ "$bad" = 0 ] || exit 2
HOST_IP="$(ip route show default 2> /dev/null | awk '{print $3; exit}')"
PEER="$(grep -oE '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' "$HOME/.ros/fastdds.xml" 2> /dev/null | head -1)"
say "    host ${HOST_IP:-?}, FastDDS peer ${PEER:-none} (FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-unset})"
[ -n "$PEER" ] && [ -n "$HOST_IP" ] && [ "$PEER" != "$HOST_IP" ] && say "    WARNING: the FastDDS peer differs from the host - ROS may not see Isaac"
dds_hygiene
isaac_reap | grep -E "REAPED" | sed 's/^/    ended a leftover Isaac process: /'

# ------------------------------------------------------------------ 2. launch
T_LAUNCH=$SECONDS
out="$(ps_run "$(winpath "$REPO/sim/launch_wheeled.ps1")" -Seed "$SEED" -RunDir "$(winpath "$RUN_DIR")" -Window "$WINDOW")"
ISAAC_PID="$(sed -n 's/^PID=//p' <<< "$out" | head -1)"
if [ -z "$ISAAC_PID" ]; then say "launch failed: $(tr '\n' ' ' <<< "$out" | cut -c1-400)"; exit 1; fi
say "    isaac console pid $ISAAC_PID - waiting for Play (up to $READY_S s)"
state=""; last_print=0
while [ $((SECONDS - T_LAUNCH)) -lt "$READY_S" ]; do
  state="$(sfield "$RUN_DIR/isaac_status.txt" state)"
  [ "$state" = playing ] && break
  [ "$state" = error ] && break
  if [ $((SECONDS - T_LAUNCH)) -gt 20 ] && ! win_alive "$ISAAC_PID"; then state=gone; break; fi
  if [ $((SECONDS - last_print)) -ge 20 ]; then
    last_print=$SECONDS; echo "      ... $((SECONDS - T_LAUNCH)) s, state ${state:-no status yet}"
  fi
  sleep 3
done
STARTUP_S=$((SECONDS - T_LAUNCH))
if [ "$state" != playing ]; then
  case "$state" in
    error) say "ISAAC BOOT ERROR after $STARTUP_S s: $(sfield "$RUN_DIR/isaac_status.txt" reason)" ;;
    gone) say "ISAAC EXITED before Play ($STARTUP_S s)" ;;
    *) say "ISAAC NOT PLAYING after $STARTUP_S s (state: ${state:-no status file - did --exec run the bootstrap?})" ;;
  esac
  say "--- isaac.log (bootstrap lines)"; isaac_lines 30 | tee -a "$SUMMARY"
  isaac_stop; say "Isaac: $CLOSE_HOW"; say "=== RESULT: FAIL (Isaac side)"; exit 1
fi
say "    isaac playing after $STARTUP_S s"
sleep 5                                         # the publishers' first messages
GT="$(ls -t "$RUN_DIR"/wheeled_gt_*.csv 2> /dev/null | head -1)"

# ------------------------------------------------------------------ 3. probe
say "--- probe (ROS side)"
python3 "$REPO/missions/wheeled_probe.py" ${GT:+--gt "$GT"} 2>&1 | tee "$RUN_DIR/probe.txt"
PROBE_RC=${PIPESTATUS[0]}
cat "$RUN_DIR/probe.txt" >> "$SUMMARY"
STATUS_LINE="$(tr -d '\r' < "$RUN_DIR/isaac_status.txt" 2> /dev/null)"

# ------------------------------------------------------------------ 4. stop, summary
isaac_stop
dds_hygiene
say "--- isaac.log (bootstrap and publisher lines)"; isaac_lines 25 | tee -a "$SUMMARY"
say "--- summary"
say "    world $SEED, window $WINDOW, Isaac start-up $STARTUP_S s"
say "    last status: $STATUS_LINE"
say "    probe: $(grep -c '^CHECK .* PASS' "$RUN_DIR/probe.txt") checks passed, $(grep -c '^CHECK .* FAIL' "$RUN_DIR/probe.txt") failed; $(grep -E '^RTF=' "$RUN_DIR/probe.txt")"
say "    Isaac stop: $CLOSE_HOW"
say "    files: $(cd "$RUN_DIR" && ls | tr '\n' ' ')"
if [ "$PROBE_RC" = 0 ] && [[ "$CLOSE_HOW" == clean* ]]; then say "=== RESULT: PASS"; exit 0; fi
say "=== RESULT: FAIL"; exit 1
