#!/usr/bin/env bash
# === Wheeled batch runner (WSL): the benchmark's wheeled arm, unattended ===
#
# Per world: one FRONTIER3D exploration (budgets nested, a map frozen at each), then one
# tour per frozen map (map_b1 / b2.5 / b5 / b10, the world's 10 goals) - the aerial arm's
# protocol (missions/batch_aerial.sh) on the wheeled baseline's stack. Every run gets a
# fresh Isaac process (the full app as the baseline runs it: sim/launch_wheeled.ps1 through
# the WSL->Windows interop, sim/wheeled_bootstrap.py), then the navigation bring-up
# (missions/wheeled_nav.sh: slam_toolbox + Nav2 for the exploration, Nav2 with map_server +
# AMCL on the frozen map for a tour) behind its readiness gates, then the runner
# (wheeled_explore_runner.py / wheeled_mission_runner.py). After each run: the verdict
# (verify_missions.py) or the exploration figure in the run directory, one line in the batch
# log and in status.csv.
#
#   bash missions/batch_wheeled.sh [options] <seed> [<seed> ...]
#   bash missions/batch_wheeled.sh [options] --all       (the ten worlds, 008 first)
#   bash missions/batch_wheeled.sh --smoke               (008, 1-minute budget, 2-goal tour)
#
# Options:
#   --dry-run            show which runs are done / pending, then exit
#   --window 0|1         the Isaac app's window (default 0 = Kit's --no-window: RTF 0.41 against 0.37
#                        with the window, the sensors identical - the lidar is a PhysX raycast, not rendered)
#   --only explore|tours only that phase
#   --budgets LIST       default 1,2.5,5,10 (sim minutes); the tours follow the list
#   --tour-limit N       only the first N goals of each tour (tests)
#   --batch-dir DIR      default runs/raw/wbatch2 (must be on a Windows drive, /mnt/<x>/...); the first
#                        batch, runs/raw/wbatch, ran before the Nav2 corrections and is kept as it is
#   --explore-timeout M  wall minutes per exploration run (default 60)
#   --tour-timeout M     wall minutes per tour (default 240: 20 attempts x 240 sim-s at RTF 0.33)
#   --nav2-params FILE   the Nav2 params of every run (default policies/nav2/nav2_params.yaml, the
#                        benchmark's: the predecessor's v9 + corrections); the footprint sensitivity variant, the true body outline,
#                        is policies/nav2/nav2_params_polygon.yaml, into its own --batch-dir. A batch dir
#                        whose runs used other settings is refused (comments and the map path aside)
#   --stack nav2|shared  nav2 (default): the baseline's stack above. shared: the same executive
#                        as the aerial arm drives the wheeled robot - lidar odometry (rf2o, aerial_odom.sh
#                        with the lidar at base_link), slam_toolbox (aerial_slam.sh) or AMCL on the frozen
#                        map (aerial_amcl.sh), aerial_explore_runner.py / aerial_mission_runner.py
#                        --vehicle ground over /cmd_vel; Isaac's odometry publisher sends no TF (-OdomTf 0);
#                        default --batch-dir runs/raw/wbatch3; --nav2-params does not apply. A batch dir
#                        of the other stack is refused
#
# Resume and finality as the aerial batch: every attempt in its own directory,
# <seed>/<phase>/try<k>/ (never moved); a try whose policy clock started and that did not end
# in an ERROR or lose the simulator is final (FINAL written into it) and its phase is skipped
# from then on; a setup failure, an ERROR, a dead simulator or a user stop is followed by a
# new try (MAX_TRIES per batch run, then FAILED and the batch goes on; a failed exploration
# skips that world's tours). A run cut by the wall timeout is final (INTERRUPTED).
#
# Outputs: <batch-dir>/<seed>/explore/try<k>/ and tour_b<budget>/try<k>/ - isaac.log, the
# ground truth (wheeled_gt_*.csv), the odometry log, wnav.txt + the Nav2 / slam logs, the
# run's params, runner.txt, the runner's CSVs and manifest, clock_offset.txt, verdict.txt /
# a figure, FINAL on the try that counts; <batch-dir>/batch.log, status.csv.
#
# Needs: ROS 2 Jazzy with nav2_bringup and slam_toolbox (--stack shared: also rf2o built in
# ~/ros2_ws, VERSIONS.md), Windows PowerShell reachable from WSL.
# Keeps Windows awake while it runs (sim/keep_awake.ps1). Ctrl+C: the current run stops
# cleanly (goal cancelled, manifest written, Isaac closed), the batch ends; running the same
# command again resumes.
# (no `set -u`: ROS's setup.bash references unset variables and would abort under it)

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
PS="${AEB_PS:-/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe}"
TASKLIST="${AEB_TASKLIST:-/mnt/c/Windows/System32/tasklist.exe}"
TASKKILL="${AEB_TASKKILL:-/mnt/c/Windows/System32/taskkill.exe}"
WSLPATH="${AEB_WSLPATH:-wslpath}"
ALL_WORLDS="20260723008 20260723001 20260723002 20260723003 20260723004 20260723005 20260723013 20260723016 20260723018 20260723023"

DRY=0; WINDOW=0; ONLY=""; BUDGETS="1,2.5,5,10"; TOUR_LIMIT=0; NAV2_PARAMS="policies/nav2/nav2_params.yaml"
STACK="nav2"; BATCH_DIR=""; SMOKE=0; EXPLORE_TIMEOUT_MIN=60; TOUR_TIMEOUT_MIN=240
MAX_IMPACTS="${AEB_MAX_IMPACTS:-0}"   # shared stack: impacts never stop a run, as the aerial batch (no IMU on this robot anyway)
ISAAC_READY_S="${AEB_ISAAC_READY_S:-420}"; NAV_READY_S="${AEB_NAV_READY_S:-300}"
SIM_STALE_S="${AEB_SIM_STALE_S:-150}"; MAX_TRIES="${AEB_MAX_TRIES:-2}"
SEEDS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --window) WINDOW="$2"; shift ;;
    --only) ONLY="$2"; shift ;;
    --budgets) BUDGETS="$2"; shift ;;
    --tour-limit) TOUR_LIMIT="$2"; shift ;;
    --batch-dir) BATCH_DIR="$2"; shift ;;
    --explore-timeout) EXPLORE_TIMEOUT_MIN="$2"; shift ;;
    --tour-timeout) TOUR_TIMEOUT_MIN="$2"; shift ;;
    --nav2-params) NAV2_PARAMS="$2"; shift ;;
    --stack) STACK="$2"; shift ;;
    --all) read -r -a W <<< "$ALL_WORLDS"; SEEDS+=("${W[@]}") ;;
    --smoke) BUDGETS="1"; TOUR_LIMIT=2; SMOKE=1; SEEDS+=(20260723008) ;;
    -h|--help) sed -n '2,58p' "$0"; exit 0 ;;
    -*) echo "unknown option $1 (--help)"; exit 2 ;;
    *) SEEDS+=("$1") ;;
  esac
  shift
done
[ ${#SEEDS[@]} -gt 0 ] || { echo "usage: $0 [options] <seed>... | --all | --smoke   (--help)"; exit 2; }
case "$ONLY" in ""|explore|tours) ;; *) echo "--only explore|tours"; exit 2 ;; esac
case "$WINDOW" in 0|1) ;; *) echo "--window 0|1"; exit 2 ;; esac
case "$STACK" in nav2|shared) ;; *) echo "--stack nav2|shared"; exit 2 ;; esac
if [ -z "$BATCH_DIR" ]; then
  if [ "$SMOKE" = 1 ]; then BATCH_DIR="$REPO/runs/raw/wbatch_smoke$([ "$STACK" = shared ] && echo _shared)"
  else BATCH_DIR="$REPO/runs/raw/$([ "$STACK" = shared ] && echo wbatch3 || echo wbatch2)"; fi
fi
UNIQ=(); for s in "${SEEDS[@]}"; do case " ${UNIQ[*]} " in *" $s "*) ;; *) UNIQ+=("$s") ;; esac; done; SEEDS=("${UNIQ[@]}")
BLIST="$(python3 -c 'import sys; print(" ".join(f"{float(b):g}" for b in sys.argv[1].split(",")))' "$BUDGETS")" || exit 2
BUDGETS_CSV="$(tr ' ' ',' <<< "$BLIST")"
mkdir -p "$BATCH_DIR" && BATCH_DIR="$(cd "$BATCH_DIR" && pwd)"
# the Nav2 params of every run (wheeled_nav.sh reads AEB_NAV2_PARAMS) - one set of settings
# per batch dir, checked against the copy the newest run kept
case "$NAV2_PARAMS" in /*) ;; *) if [ -f "$NAV2_PARAMS" ]; then NAV2_PARAMS="$(pwd)/$NAV2_PARAMS"; else NAV2_PARAMS="$REPO/$NAV2_PARAMS"; fi ;; esac
[ -f "$NAV2_PARAMS" ] || { echo "Nav2 params not found: $NAV2_PARAMS"; exit 2; }
NAV2_PARAMS="$(readlink -f "$NAV2_PARAMS")"; NAV2_REL="${NAV2_PARAMS#$REPO/}"
params_sig() {  # the settings only: comments, blank lines, CR line ends and the per-run map path left out
  tr -d '\r' < "$1" | sed -E -e 's/[[:space:]]+#.*$//' -e '/^[[:space:]]*(#|$)/d' -e 's|^([[:space:]]*yaml_filename:).*|\1|' | md5sum | cut -c1-12
}
NAV2_SIG="$(params_sig "$NAV2_PARAMS")"
if [ "$STACK" = nav2 ]; then
  LAST_PARAMS="$(ls -t "$BATCH_DIR"/2026*/*/try*/nav2_params_*.yaml 2>/dev/null | head -1)"
  if [ -n "$LAST_PARAMS" ] && [ "$(params_sig "$LAST_PARAMS")" != "$NAV2_SIG" ]; then
    echo "the runs in ${BATCH_DIR#$REPO/} used other Nav2 settings than $NAV2_REL (${LAST_PARAMS#$REPO/}):"
    echo "pass the same --nav2-params, or give these params a new --batch-dir"; exit 2
  fi
fi
# one stack per batch dir: the marker of the first run decides
if [ -f "$BATCH_DIR/stack.txt" ] && [ "$(tr -d '\r\n ' < "$BATCH_DIR/stack.txt")" != "$STACK" ]; then
  echo "the runs in ${BATCH_DIR#$REPO/} used the $(cat "$BATCH_DIR/stack.txt") stack: give --stack $STACK its own --batch-dir"; exit 2
fi
export AEB_NAV2_PARAMS="$NAV2_PARAMS"
BATCH_LOG="$BATCH_DIR/batch.log"
STATUS_CSV="$BATCH_DIR/status.csv"
PROGRESS_RE='-> g[0-9]+|SUCCEEDED|ABORTED|TIMEOUT|REJECTED|TOUR DONE|frontier_done|frontier_failed|checkpoint budget|EXPLORATION DONE|SIM_STALLED|ERROR|Traceback|INTERRUPTED'

log() { local l; l="$(date '+%F %T') $*"; echo "$l"; echo "$l" >> "$BATCH_LOG"; }
winpath() { "$WSLPATH" -w "$1"; }
sfield() {  # file key -> value of key=value in a one-line status file (Isaac writes it on Windows: CRLF)
  [ -f "$1" ] || return 0
  tr -d '\r' < "$1" 2>/dev/null | tr ' ' '\n' | sed -n "s/^$2=//p" | head -1
}

manifest_state() {  # run_dir -> "<none|setup|started|error> <status>"
  python3 - "$1" <<'PY'
import glob, json, os, sys
d = sys.argv[1]
ms = sorted(glob.glob(os.path.join(d, "manifest_*.json")), key=os.path.getmtime)
if not ms:
    print("none -"); sys.exit()
try:
    m = json.load(open(ms[-1]))
except Exception:
    print("none unreadable"); sys.exit()
st = str(m.get("status", "?")).replace(" ", "_")[:60]
started = any(e.get("event") == "policy_clock_start" for e in m.get("events", []))
print(("error" if st.startswith("ERROR") else "started") if started else "setup", st)
PY
}

run_done() { [ -f "$1/FINAL" ] && [ ! -e "$1/interrupted_by_user" ]; }
tries() { ls -dv "$1"/try[0-9]* 2>/dev/null; }
final_try() { local d f=""; for d in $(tries "$1"); do run_done "$d" && f="$d"; done; echo "$f"; }
next_try() { local n=1; while [ -e "$1/try$n" ]; do n=$((n + 1)); done; echo "$1/try$n"; }

# ------------------------------------------------------------------ dry run
if [ "$DRY" = 1 ]; then
  echo "batch dir: $BATCH_DIR   budgets: $BLIST   window: $WINDOW   stack: $STACK$([ "$STACK" = nav2 ] && echo "   nav2: $NAV2_REL ($NAV2_SIG)")"
  for SEED in "${SEEDS[@]}"; do
    for ph in explore $(for b in $BLIST; do echo tour_b$b; done); do
      pd="$BATCH_DIR/$SEED/$ph"; f="$(final_try "$pd")"
      if [ -n "$f" ]; then echo "  $SEED $ph: DONE in $(basename "$f") ($(manifest_state "$f" | cut -d' ' -f2))"
      elif [ -n "$(tries "$pd")" ]; then
        echo "  $SEED $ph: pending ($(tries "$pd" | wc -l) tries so far; last: $(manifest_state "$(tries "$pd" | tail -1)"))"
      else echo "  $SEED $ph: pending"; fi
    done
  done
  exit 0
fi

# ------------------------------------------------------------------ Windows side
ps_run() { "$PS" -NoProfile -ExecutionPolicy Bypass -File "$@" 2>&1 | tr -d '\r'; }
isaac_reap() {  # [console pid]
  if [ -n "$1" ]; then ps_run "$(winpath "$REPO/sim/reap_isaac.ps1")" -ConsolePid "$1"
  else ps_run "$(winpath "$REPO/sim/reap_isaac.ps1")"; fi
}
win_alive() { [ -n "$1" ] && "$TASKLIST" /FI "PID eq $1" /NH 2>/dev/null | tr -d '\r' | grep -qw "$1"; }
isaac_alive() { win_alive "$ISAAC_PID"; }

isaac_start() {  # run_dir
  local out
  out="$(ps_run "$(winpath "$REPO/sim/launch_wheeled.ps1")" -Seed "$SEED" -RunDir "$(winpath "$1")" -Window "$WINDOW" \
        -OdomTf "$([ "$STACK" = shared ] && echo 0 || echo 1)" 9>&-)"
  ISAAC_PID="$(sed -n 's/^PID=//p' <<< "$out" | head -1)"
  if [ -z "$ISAAC_PID" ]; then log "    isaac launch failed: $(tr '\n' ' ' <<< "$out" | cut -c1-300)"; return 1; fi
  log "    isaac console pid $ISAAC_PID (window=$WINDOW)"
}

isaac_wait_ready() {  # run_dir timeout_s
  local t0=$SECONDS st
  while [ $((SECONDS - t0)) -lt "$2" ]; do
    st="$(sfield "$1/isaac_status.txt" state)"
    [ "$st" = playing ] && { log "    isaac playing after $((SECONDS - t0)) s"; return 0; }
    [ "$st" = error ] && { log "    isaac boot error: $(sfield "$1/isaac_status.txt" reason | cut -c1-200)"; return 1; }
    if [ $((SECONDS - t0)) -gt 20 ] && ! isaac_alive; then log "    isaac process gone before it was ready - see isaac.log"; return 1; fi
    sleep 3
  done
  log "    isaac not ready after $2 s"; return 1
}

isaac_stop() {  # the stop file (honoured from Play on), "closed", Kit quits by itself; then the reaper
  [ -n "$RUN_DIR" ] || return 0
  touch "$RUN_DIR/isaac_stop" 2>/dev/null
  local t0=$SECONDS out="" i closed=0 st
  st="$(sfield "$RUN_DIR/isaac_status.txt" state)"
  if [ "$st" = playing ] || [ "$st" = closing ]; then
    while [ $((SECONDS - t0)) -lt 60 ]; do
      [ "$(sfield "$RUN_DIR/isaac_status.txt" state)" = closed ] && { closed=1; break; }
      isaac_alive || break
      sleep 3
    done
    if [ "$closed" = 1 ]; then t0=$SECONDS; while isaac_alive && [ $((SECONDS - t0)) -lt 45 ]; do sleep 2; done; fi
  fi
  for i in 1 2 3 4; do                            # a killed GPU process can take seconds to go
    out="$(isaac_reap "$ISAAC_PID")"
    grep "REAPED" <<< "$out" | sed 's/^/    reap: /'
    grep -q "ISAAC_PROCESSES=0" <<< "$out" && break
    sleep 5
  done
  if ! grep -q "ISAAC_PROCESSES=0" <<< "$out"; then
    if grep -q "ISAAC_PROCESSES=" <<< "$out"; then log "    WARNING: Isaac processes left after the reaper ($(grep -o 'ISAAC_PROCESSES=[0-9]*' <<< "$out"))"
    else log "    WARNING: the reaper gave no answer (PowerShell interrupted?) - check for a leftover Isaac before the next run"; fi
  fi
  ISAAC_PID=""
}

clock_check() {  # Windows vs WSL wall clock (the verifier joins the two sides by wall time)
  local t1 t2 w
  t1=$(date +%s.%N)
  w=$("$PS" -NoProfile -Command '[DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()' 2>/dev/null | tr -d '\r' | tail -1)
  t2=$(date +%s.%N)
  python3 -c 'import sys; t1, t2, w = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3] or 0) / 1000.0
print(f"wsl_mid={(t1 + t2) / 2:.3f} windows={w:.3f} offset_windows_minus_wsl={w - (t1 + t2) / 2:+.3f} s uncertainty=+-{(t2 - t1) / 2:.3f} s")' "$t1" "$t2" "$w"
}

KA_BG=""
keep_awake_start() {
  "$PS" -NoProfile -ExecutionPolicy Bypass -File "$(winpath "$REPO/sim/keep_awake.ps1")" \
      -PidFile "$(winpath "$BATCH_DIR")\\keep_awake.pid" -MaxHours 36 > /dev/null 2>&1 9>&- &
  KA_BG=$!
}
keep_awake_stop() {
  local p=""; [ -f "$BATCH_DIR/keep_awake.pid" ] && p="$(tr -d '\r\n ' < "$BATCH_DIR/keep_awake.pid")"
  [ -n "$p" ] && "$TASKKILL" /F /PID "$p" > /dev/null 2>&1
  [ -n "$KA_BG" ] && kill "$KA_BG" 2>/dev/null
  rm -f "$BATCH_DIR/keep_awake.pid"
}

# ------------------------------------------------------------------ WSL side
bg_script() {  # var logfile script args... ; the script runs in its own session, $var = its pid
  local var="$1" logf="$2"; shift 2
  setsid env AEB_LOG_DIR="$RUN_DIR" bash "$@" > "$logf" 2>&1 < /dev/null 9>&- &
  printf -v "$var" '%s' "$!"
}

stop_script() {  # pid: TERM runs the script's own teardown (nodes stopped, DDS cleaned)
  local pid="$1" i; [ -n "$pid" ] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -TERM "$pid" 2>/dev/null
  for i in $(seq 1 60); do kill -0 "$pid" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -- -"$pid" 2>/dev/null; kill -KILL "$pid" 2>/dev/null
}

topic_alive() {  # topic timeout_s (the shared stack's odometry gate, as the aerial batch)
  local t0=$SECONDS
  while [ $((SECONDS - t0)) -lt "$2" ]; do
    timeout 15 ros2 topic echo --once "$1" --field header.frame_id > /dev/null 2>&1 && return 0
    sleep 2
  done
  return 1
}

wait_line() {  # file regex timeout_s pid [fail_regex]
  local t0=$SECONDS
  while [ $((SECONDS - t0)) -lt "$3" ]; do
    grep -qE "$2" "$1" 2>/dev/null && return 0
    [ -n "$5" ] && grep -qE "$5" "$1" 2>/dev/null && return 1
    kill -0 "$4" 2>/dev/null || return 1
    sleep 2
  done
  return 1
}

start_runner() {  # logfile cmd... ; own session: a Ctrl+C in this terminal reaches only the batch
  local logf="$1"; shift
  ( cd "$REPO" && exec setsid "$@" ) > "$logf" 2>&1 < /dev/null 9>&- &
  RUNNER_PID=$!
}

stop_runner() {  # SIGTERM = KeyboardInterrupt in the runner: the goal is cancelled, the manifest written
  local i; [ -n "$RUNNER_PID" ] || return 0
  if kill -0 "$RUNNER_PID" 2>/dev/null; then
    kill -TERM "$RUNNER_PID" 2>/dev/null
    for i in $(seq 1 60); do kill -0 "$RUNNER_PID" 2>/dev/null || break; sleep 1; done
    kill -KILL "$RUNNER_PID" 2>/dev/null
  fi
  wait "$RUNNER_PID" 2>/dev/null
  RUNNER_PID=""
}

show_progress() {  # new interesting lines of the runner's output
  local f="$1" n
  n=$(wc -l < "$f" 2>/dev/null || echo 0)
  if [ "$n" -gt "$SHOWN" ]; then
    sed -n "$((SHOWN + 1)),${n}p" "$f" | grep -E -- "$PROGRESS_RE" | sed 's/^/      /'
    SHOWN=$n
  fi
}

supervise() {  # run_dir timeout_s -> SUP_RESULT: done | timeout | sim_dead
  local rd="$1" tmo="$2" t0=$SECONDS lastf="" lastc=$SECONDS lastcheck=$SECONDS f
  SUP_RESULT="done"
  while kill -0 "$RUNNER_PID" 2>/dev/null; do
    sleep 5
    show_progress "$rd/runner.txt"
    f="$(sfield "$rd/isaac_status.txt" frame)"
    if [ "$f" != "$lastf" ]; then lastf="$f"; lastc=$SECONDS; fi
    if [ $((SECONDS - lastc)) -gt "$SIM_STALE_S" ]; then SUP_RESULT=sim_dead; break; fi
    if [ $((SECONDS - lastcheck)) -gt 30 ]; then
      lastcheck=$SECONDS
      isaac_alive || { SUP_RESULT=sim_dead; break; }
    fi
    if [ $((SECONDS - t0)) -gt "$tmo" ]; then SUP_RESULT=timeout; break; fi
  done
  [ "$SUP_RESULT" = "done" ] || log "    $SUP_RESULT - stopping the runner (it cancels the goal and writes its manifest)"
  stop_runner
  show_progress "$rd/runner.txt"
}

teardown_run() {  # runner, then the navigation stack (shared: the localizer, then the odometry), then Isaac
  stop_runner
  stop_script "$NAV_PID"; NAV_PID=""
  stop_script "$ODOM_PID"; ODOM_PID=""
  isaac_stop
}

# ------------------------------------------------------------------ one run
run_once() {  # phase run_dir map -> RUN_RESULT (done|setup|sim_dead|error), RUN_STATUS
  RUN_ACTIVE=1                                    # a Ctrl+C now marks this run interrupted_by_user
  run_steps "$@"
  RUN_ACTIVE=0
}

run_steps() {
  local phase="$1" map="$3" tmo args mode
  RUN_DIR="$2"; SHOWN=0; SUP_RESULT=""; RUNNER_PID=""; NAV_PID=""; ODOM_PID=""; ISAAC_PID=""
  mkdir -p "$RUN_DIR"
  isaac_reap | grep -E "REAPED" | sed 's/^/    stale: /'
  clock_check > "$RUN_DIR/clock_offset.txt" 2>&1
  RUN_RESULT=setup
  if ! isaac_start "$RUN_DIR"; then RUN_STATUS=ISAAC_LAUNCH; teardown_run; return; fi
  if ! isaac_wait_ready "$RUN_DIR" "$ISAAC_READY_S"; then RUN_STATUS=ISAAC_NOT_READY; teardown_run; return; fi
  if [ "$STACK" = shared ]; then run_steps_shared "$phase" "$map"; return; fi
  if [ "$phase" = explore ]; then mode=explore; else mode=tour; fi
  bg_script NAV_PID "$RUN_DIR/wnav.txt" "$REPO/missions/wheeled_nav.sh" "$SEED" "$mode" ${map:+"$map"}
  if ! wait_line "$RUN_DIR/wnav.txt" "\[wnav\] READY" "$NAV_READY_S" "$NAV_PID" "\[wnav\] FAILED"; then
    log "    navigation stack not ready: $(grep -E '\[wnav\] FAILED' "$RUN_DIR/wnav.txt" | tail -1 | cut -c1-200) - see wnav.txt"
    RUN_STATUS=NAV_NOT_READY; teardown_run; return
  fi
  if [ "$phase" = explore ]; then
    log "    slam_toolbox + Nav2 up; exploring (budgets $BUDGETS_CSV sim-min)"
    start_runner "$RUN_DIR/runner.txt" python3 -u "$REPO/missions/wheeled_explore_runner.py" \
        --seed "$SEED" --budgets "$BUDGETS_CSV" --out-dir "$RUN_DIR"
    tmo=$(awk -v m="$EXPLORE_TIMEOUT_MIN" 'BEGIN { print int(m * 60) }')
  else
    log "    Nav2 + AMCL up on $(basename "$map"); touring"
    args=(--seed "$SEED" --map "$map" --out-dir "$RUN_DIR")
    [ "$TOUR_LIMIT" -gt 0 ] && args+=(--tour-limit "$TOUR_LIMIT")
    start_runner "$RUN_DIR/runner.txt" python3 -u "$REPO/missions/wheeled_mission_runner.py" "${args[@]}"
    tmo=$(awk -v m="$TOUR_TIMEOUT_MIN" 'BEGIN { print int(m * 60) }')
  fi
  supervise "$RUN_DIR" "$tmo"
  teardown_run
  local st; read -r st RUN_STATUS <<< "$(manifest_state "$RUN_DIR")"
  if [ "$SUP_RESULT" = sim_dead ]; then RUN_RESULT=sim_dead
  elif [ "$RUN_STATUS" = SIM_STALLED ]; then RUN_RESULT=sim_dead
  elif [ "$st" = started ]; then RUN_RESULT="done"
  elif [ "$st" = error ]; then RUN_RESULT=error
  else RUN_RESULT=setup; fi
}

run_steps_shared() {  # phase map (RUN_DIR set): the aerial batch's chain on the wheeled robot
  local phase="$1" map="$2" tmo args
  AEB_LIDAR_Z=0 bg_script ODOM_PID "$RUN_DIR/odom_script.txt" "$REPO/missions/aerial_odom.sh" "$SEED"
  if ! wait_line "$RUN_DIR/odom_script.txt" "odom\] running" 150 "$ODOM_PID" "died|not found" \
      || ! topic_alive /odom_rf2o 60; then
    log "    odometry chain not ready - see odom_script.txt"; RUN_STATUS=ODOM_NOT_READY; teardown_run; return
  fi
  log "    lidar odometry flowing"
  if [ "$phase" = explore ]; then
    bg_script NAV_PID "$RUN_DIR/slam_script.txt" "$REPO/missions/aerial_slam.sh" "$SEED"
    if ! wait_line "$RUN_DIR/slam_script.txt" "slam\] running" 150 "$NAV_PID" "launch died"; then
      log "    mapper not ready - see slam_script.txt"; RUN_STATUS=SLAM_NOT_READY; teardown_run; return
    fi
    log "    mapper up; exploring with the shared executive (budgets $BUDGETS_CSV sim-min)"
    start_runner "$RUN_DIR/runner.txt" python3 -u "$REPO/missions/aerial_explore_runner.py" --vehicle ground \
        --seed "$SEED" --budgets "$BUDGETS_CSV" --max-impacts "$MAX_IMPACTS" --out-dir "$RUN_DIR"
    tmo=$(awk -v m="$EXPLORE_TIMEOUT_MIN" 'BEGIN { print int(m * 60) }')
  else
    bg_script NAV_PID "$RUN_DIR/amcl_script.txt" "$REPO/missions/aerial_amcl.sh" "$SEED" "$map"
    if ! wait_line "$RUN_DIR/amcl_script.txt" "amcl\] running" 240 "$NAV_PID" "launch died" \
        || ! grep -q "/map published" "$RUN_DIR/amcl_script.txt"; then
      log "    localizer not ready - see amcl_script.txt"; RUN_STATUS=AMCL_NOT_READY; teardown_run; return
    fi
    log "    AMCL up on $(basename "$map"); touring with the shared executive"
    args=(--vehicle ground --seed "$SEED" --map "$map" --repel-gain 1.0 --max-impacts "$MAX_IMPACTS" --out-dir "$RUN_DIR")
    [ "$TOUR_LIMIT" -gt 0 ] && args+=(--tour-limit "$TOUR_LIMIT")
    start_runner "$RUN_DIR/runner.txt" python3 -u "$REPO/missions/aerial_mission_runner.py" "${args[@]}"
    tmo=$(awk -v m="$TOUR_TIMEOUT_MIN" 'BEGIN { print int(m * 60) }')
  fi
  supervise "$RUN_DIR" "$tmo"
  teardown_run
  local st; read -r st RUN_STATUS <<< "$(manifest_state "$RUN_DIR")"
  if [ "$SUP_RESULT" = sim_dead ]; then RUN_RESULT=sim_dead
  elif [ "$st" = started ]; then RUN_RESULT="done"
  elif [ "$st" = error ]; then RUN_RESULT=error
  else RUN_RESULT=setup; fi
}

post_run() {  # phase run_dir map -> RUN_SUMMARY
  local phase="$1" rd="$2" map="$3" m
  RUN_SUMMARY=""
  if [ "$phase" = explore ]; then
    RUN_SUMMARY="$(python3 - "$rd" <<'PY' 2>/dev/null
import glob, json, os, sys
ms = sorted(glob.glob(os.path.join(sys.argv[1], "manifest_*.json")), key=os.path.getmtime)
m = json.load(open(ms[-1]))
s = m.get("summary") or {}
cps = " ".join(f"b{c['budget_min']:g}={c['coverage_m2']:.0f}m2{'(final)' if c.get('final') else ''}" for c in m.get("checkpoints", []))
print(f"{s.get('reason')} t={s.get('t_sim')}s frontiers={s.get('frontiers')} | {cps}")
PY
)"
    ( cd "$REPO" && python3 analysis/plot_explore.py --run "$rd" --out "$rd/explore.png" > /dev/null 2>&1 )
  else
    m="$(ls -t "$rd"/missions_*.csv 2>/dev/null | head -1)"
    if [ -n "$m" ]; then
      ( cd "$REPO" && python3 analysis/verify_missions.py --missions "$m" > "$rd/verdict.txt" 2>&1 )
      RUN_SUMMARY="$(grep -E "goals, .* attempts" "$rd/verdict.txt" | tail -1)"
      ( cd "$REPO" && python3 analysis/plot_tour.py --missions "$m" --map "$map" --out "$rd/tour.png" > /dev/null 2>&1 )
    fi
  fi
}

record() {  # phase try result status wall_min summary
  [ -f "$STATUS_CSV" ] || echo "time,seed,phase,try,result,status,wall_min,summary" > "$STATUS_CSV"
  local summ="${6//\"/}"
  echo "$(date '+%F %T'),$SEED,$1,$2,$3,$4,$5,\"$summ\"" >> "$STATUS_CSV"
}

do_phase() {  # phase phase_dir map -> PHASE_FINAL (the final try directory)
  local phase="$1" pd="$2" map="$3" k rd t0 wall
  PHASE_FINAL="$(final_try "$pd")"
  if [ -n "$PHASE_FINAL" ]; then
    log "  $phase: done already in $(basename "$PHASE_FINAL") ($(manifest_state "$PHASE_FINAL" | cut -d' ' -f2)) - skipped"; return 0
  fi
  for k in $(seq 1 "$MAX_TRIES"); do
    rd="$(next_try "$pd")"                        # a new directory per try: nothing is ever moved
    log "  $phase: attempt $k/$MAX_TRIES -> ${rd#$REPO/}"
    t0=$SECONDS
    run_once "$phase" "$rd" "$map"
    wall=$(( (SECONDS - t0 + 30) / 60 ))
    [ "$RUN_RESULT" = "done" ] && echo "$(date '+%F %T') result=$RUN_RESULT status=$RUN_STATUS" > "$rd/FINAL"
    post_run "$phase" "$rd" "$map"
    record "$phase" "$(basename "$rd")" "$RUN_RESULT" "$RUN_STATUS" "$wall" "$RUN_SUMMARY"
    log "  $phase: $RUN_RESULT ($RUN_STATUS) in $wall min${RUN_SUMMARY:+ - $RUN_SUMMARY}"
    if [ "$RUN_RESULT" = "done" ]; then PHASE_FINAL="$rd"; return 0; fi
    [ "$STOPPING" = 1 ] && return 1
  done
  log "  $phase: FAILED after $MAX_TRIES tries"
  return 1
}

# ------------------------------------------------------------------ batch
STOPPING=0; RUN_ACTIVE=0; RUN_DIR=""
on_signal() {
  [ "$STOPPING" = 1 ] && return
  STOPPING=1
  if [ "$RUN_ACTIVE" = 1 ]; then
    log "interrupted - stopping the current run cleanly (goal cancelled, Isaac closes) ..."
    [ -n "$RUN_DIR" ] && [ -d "$RUN_DIR" ] && touch "$RUN_DIR/interrupted_by_user"
    teardown_run
  fi
  log "batch stopped by the user; the same command resumes"
  exit 130
}
trap on_signal INT TERM
trap keep_awake_stop EXIT

exec 9> /tmp/aeb_batch.lock                       # the aerial batch's lock: one batch owns Isaac
flock -n 9 || { echo "another batch is already running on this machine (it owns Isaac)"; exit 1; }
case "$BATCH_DIR" in /mnt/[a-z]/*) ;; *) echo "the batch dir must be on a Windows drive (/mnt/<x>/...): $BATCH_DIR"; exit 2 ;; esac
[ -x "$PS" ] || { echo "PowerShell not found: $PS"; exit 2; }

# pre-flight: every missing piece stops the batch before the first launch, not at 2 a.m.
source /opt/ros/jazzy/setup.bash
bad=0
for SEED in "${SEEDS[@]}"; do
  [ -f "$REPO/worlds/manifests/world_$SEED.csv" ] || { echo "no world manifest for $SEED"; bad=1; }
  [ -f "$REPO/missions/goals/goals_$SEED.csv" ] || { echo "no goal file for $SEED"; bad=1; }
done
[ -f "$REPO/sim/wheeled/robot.usda" ] || { echo "sim/wheeled/robot.usda missing"; bad=1; }
python3 -c "import numpy, yaml, matplotlib, rclpy, tf2_ros, nav2_msgs.action" 2>/dev/null || { echo "python deps missing (numpy yaml matplotlib rclpy tf2_ros nav2_msgs)"; bad=1; }
ros2 pkg prefix nav2_bringup > /dev/null 2>&1 || { echo "nav2_bringup not installed"; bad=1; }
ros2 pkg prefix slam_toolbox > /dev/null 2>&1 || { echo "slam_toolbox not installed"; bad=1; }
if [ "$STACK" = shared ]; then
  [ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
  ros2 pkg executables rf2o_laser_odometry 2>/dev/null | grep -q rf2o_laser_odometry_node || { echo "rf2o_laser_odometry not built in ~/ros2_ws (the shared stack's odometry)"; bad=1; }
  for f in missions/aerial_odom.sh missions/aerial_slam.sh missions/aerial_amcl.sh missions/aerial_mission_runner.py missions/aerial_explore_runner.py policies/nav2/amcl_aerial.yaml; do
    [ -f "$REPO/$f" ] || { echo "missing $f"; bad=1; }
  done
fi
[ "$bad" = 0 ] || exit 2
HOST_IP="$(ip route show default 2>/dev/null | awk '{print $3; exit}')"
PEER="$(grep -oE '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' "$HOME/.ros/fastdds.xml" 2>/dev/null | head -1)"
FREE_C="$(df -BG --output=avail "$BATCH_DIR" 2>/dev/null | tail -1 | tr -d ' G')"

echo "$STACK" > "$BATCH_DIR/stack.txt"
if [ "$STACK" = nav2 ]; then
  echo "$NAV2_REL $NAV2_SIG" > "$BATCH_DIR/nav2_params.txt"
  log "=== wheeled batch: ${SEEDS[*]} | budgets $BLIST | window $WINDOW | tour-limit $TOUR_LIMIT | dir ${BATCH_DIR#$REPO/} | nav2 $NAV2_REL ($NAV2_SIG)"
else
  log "=== wheeled batch (shared executive): ${SEEDS[*]} | budgets $BLIST | window $WINDOW | tour-limit $TOUR_LIMIT | max-impacts $MAX_IMPACTS | dir ${BATCH_DIR#$REPO/}"
fi
log "    host ${HOST_IP:-?}, FastDDS peer ${PEER:-?}; free: ${FREE_C:-?} GB on the batch drive"
[ -n "$PEER" ] && [ -n "$HOST_IP" ] && [ "$PEER" != "$HOST_IP" ] && log "    WARNING: ~/.ros/fastdds.xml peer $PEER differs from the host $HOST_IP - ROS may not see Isaac"
keep_awake_start

for SEED in "${SEEDS[@]}"; do
  [ "$STOPPING" = 1 ] && break
  WDIR="$BATCH_DIR/$SEED"
  log "world $SEED"
  if [ "$ONLY" != tours ]; then
    if ! do_phase explore "$WDIR/explore" ""; then
      log "  no exploration: tours of $SEED skipped"; continue
    fi
    EXP="$PHASE_FINAL"
  else
    EXP="$(final_try "$WDIR/explore")"
    [ -n "$EXP" ] || { log "  no final exploration: tours of $SEED skipped"; continue; }
  fi
  if [ "$ONLY" != explore ]; then
    for b in $BLIST; do
      [ "$STOPPING" = 1 ] && break
      map="$EXP/map_b$b.yaml"
      if [ ! -f "$map" ]; then log "  tour_b$b: no map ${map#$REPO/} - skipped"; record "tour_b$b" - no_map - 0 ""; continue; fi
      do_phase "tour_b$b" "$WDIR/tour_b$b" "$map"
    done
  fi
  log "world $SEED finished"
done
log "=== batch end"
