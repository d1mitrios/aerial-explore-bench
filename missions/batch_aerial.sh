#!/usr/bin/env bash
# === Aerial batch runner (WSL): the benchmark's aerial arm, unattended ===
#
# Per world: one FRONTIER3D exploration (budgets nested, a map frozen at each), then one
# mission tour per frozen map (map_b1 / b2.5 / b5 / b10, the world's 10 goals). Every run
# gets a fresh Isaac process (headless by default: sim/launch_a7.ps1 through the
# WSL->Windows interop) and a fresh PX4 SITL, then the odometry chain (aerial_odom.sh), the
# localizer (aerial_slam.sh for the exploration, aerial_amcl.sh <map> for a tour) and the
# runner, each behind the readiness gate the manual recipes use. After each run: the
# verdict (verify_missions.py) or the exploration figure in the run directory, one line in
# the batch log and in status.csv.
#
#   bash missions/batch_aerial.sh [options] <seed> [<seed> ...]
#   bash missions/batch_aerial.sh [options] --all       (the ten worlds, 008 first)
#   bash missions/batch_aerial.sh --smoke               (008, 1-minute budget, 2-goal tour:
#                                                         the whole chain in ~15 minutes)
#   bash missions/batch_aerial.sh --link-check          (only the PX4 -> Pegasus link test)
#
# Options:
#   --dry-run            show which runs are done / pending, then exit
#   --headless 0|1       Isaac without a window (default 1)
#   --only explore|tours only that phase
#   --budgets LIST       default 1,2.5,5,10 (sim minutes); the tours follow the list
#   --tour-limit N       only the first N goals of each tour (tests)
#   --batch-dir DIR      default runs/raw/batch (must be on a Windows drive, /mnt/<x>/...)
#   --explore-timeout M  wall minutes per exploration run (default 75)
#   --tour-timeout M     wall minutes per tour (default 90)
#   --link-check         test the PX4 -> Pegasus link (TCP 4560, WSL -> Windows) and exit
#
# The link: PX4 (WSL) connects out to Pegasus on the Windows host, TCP 4560; a
# firewall on that path (ESET Endpoint Security, Windows Firewall) silently starves every
# run. The batch tests it before the first launch with sim/link_probe.ps1: a one-shot
# listener in Isaac's own python, started the way the flight app is, and a connect from WSL
# (ok | dropped | refused | port_in_use). A blocked link stops the batch before it starts;
# a run without a PX4 heartbeat gets link_diag.txt (both sides' sockets on 4560) and a new
# link test, and a link found down then stops the batch (exit 3) instead of failing every
# run of the night. AEB_SKIP_LINK_CHECK=1 skips the pre-flight test; AEB_SIM_HOST overrides
# the host PX4 connects to (default: the WSL default gateway, NAT).
#
# Resume: every attempt runs in its own directory, <seed>/<phase>/try<k>/ (never moved:
# Windows refuses to rename a folder while anything holds a file in it). An attempt is
# final once the policy clock started (the measurement happened) and it did not end in an
# ERROR or lose the simulator; the batch then writes FINAL into it, and a phase with a
# FINAL try is skipped. A try that never started measuring (setup failure), ended in an
# ERROR, or lost the simulator is followed by a new try; after MAX_TRIES in one batch run the
# phase is recorded FAILED and the batch goes on (a failed exploration skips that world's
# tours: no maps). A run cut by the wall timeout is final (INTERRUPTED; its unflown goals
# are the analysis' to count). A run stopped by Ctrl+C is not final (interrupted_by_user).
# Impacts: the runners get --max-impacts 0; an impact aborts the attempt it happens
# in and is counted, the tour or exploration goes on (AEB_MAX_IMPACTS=N restores a stop).
#
# Outputs: <batch-dir>/<seed>/explore/try<k>/ and tour_b<budget>/try<k>/ - isaac.log,
# Isaac's ground truth and scan samples, px4.log, the bring-up logs, runner.txt, the
# runner's CSVs and manifest, clock_offset.txt, verdict.txt / a figure, FINAL on the try
# that counts; <batch-dir>/batch.log, status.csv, link_probe.txt.
#
# Needs: PX4 built (~/PX4-Autopilot, run dir ~/px4_run), rf2o in ~/ros2_ws, nav2 +
# slam_toolbox, Windows PowerShell reachable from WSL. Keeps Windows awake while it runs
# (sim/keep_awake.ps1). Ctrl+C: the current run stops cleanly (the vehicle lands, Isaac
# closes), the batch ends; running the same command again resumes.
# (no `set -u`: ROS's setup.bash references unset variables and would abort under it)

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
PS="${AEB_PS:-/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe}"
TASKLIST="${AEB_TASKLIST:-/mnt/c/Windows/System32/tasklist.exe}"
TASKKILL="${AEB_TASKKILL:-/mnt/c/Windows/System32/taskkill.exe}"
NETSTAT="${AEB_NETSTAT:-/mnt/c/Windows/System32/netstat.exe}"
WSLPATH="${AEB_WSLPATH:-wslpath}"
PX4_DIR="${AEB_PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_RUN="${AEB_PX4_RUN:-$HOME/px4_run}"
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
ALL_WORLDS="20260723008 20260723001 20260723002 20260723003 20260723004 20260723005 20260723013 20260723016 20260723018 20260723023"

DRY=0; HEADLESS=1; ONLY=""; BUDGETS="1,2.5,5,10"; TOUR_LIMIT=0; LINK_ONLY=0
BATCH_DIR="$REPO/runs/raw/batch"; EXPLORE_TIMEOUT_MIN=75; TOUR_TIMEOUT_MIN=90
ISAAC_READY_S="${AEB_ISAAC_READY_S:-300}"; HEARTBEAT_S="${AEB_HEARTBEAT_S:-150}"
SIM_STALE_S="${AEB_SIM_STALE_S:-150}"; MAX_TRIES="${AEB_MAX_TRIES:-2}"
MAX_IMPACTS="${AEB_MAX_IMPACTS:-0}"   # an impact aborts only the attempt (counted); 0 = never stop a run on impacts
SEEDS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1 ;;
    --headless) HEADLESS="$2"; shift ;;
    --only) ONLY="$2"; shift ;;
    --budgets) BUDGETS="$2"; shift ;;
    --tour-limit) TOUR_LIMIT="$2"; shift ;;
    --batch-dir) BATCH_DIR="$2"; shift ;;
    --explore-timeout) EXPLORE_TIMEOUT_MIN="$2"; shift ;;
    --tour-timeout) TOUR_TIMEOUT_MIN="$2"; shift ;;
    --all) read -r -a W <<< "$ALL_WORLDS"; SEEDS+=("${W[@]}") ;;
    --smoke) BUDGETS="1"; TOUR_LIMIT=2; BATCH_DIR="$REPO/runs/raw/batch_smoke"; SEEDS+=(20260723008) ;;
    --link-check) LINK_ONLY=1 ;;
    -h|--help) sed -n '2,60p' "$0"; exit 0 ;;
    -*) echo "unknown option $1 (--help)"; exit 2 ;;
    *) SEEDS+=("$1") ;;
  esac
  shift
done
[ ${#SEEDS[@]} -gt 0 ] || [ "$LINK_ONLY" = 1 ] || { echo "usage: $0 [options] <seed>... | --all | --smoke | --link-check   (--help)"; exit 2; }
case "$ONLY" in ""|explore|tours) ;; *) echo "--only explore|tours"; exit 2 ;; esac
# unique, order kept
UNIQ=(); for s in "${SEEDS[@]}"; do case " ${UNIQ[*]} " in *" $s "*) ;; *) UNIQ+=("$s") ;; esac; done; SEEDS=("${UNIQ[@]}")
# budget labels exactly as the explorer names its maps (f"{b:g}": 1, 2.5, 5, 10)
BLIST="$(python3 -c 'import sys; print(" ".join(f"{float(b):g}" for b in sys.argv[1].split(",")))' "$BUDGETS")" || exit 2
BUDGETS_CSV="$(tr ' ' ',' <<< "$BLIST")"
mkdir -p "$BATCH_DIR" && BATCH_DIR="$(cd "$BATCH_DIR" && pwd)"
BATCH_LOG="$BATCH_DIR/batch.log"
STATUS_CSV="$BATCH_DIR/status.csv"
PROGRESS_RE='-> g[0-9]+|SUCCEEDED|ABORTED|TIMEOUT|TOUR DONE|frontier_done|frontier_failed|too_narrow|impact acc|checkpoint budget|explore_done|home_reached|home_abort|_FAIL|ERROR|Traceback|INTERRUPTED'

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

run_done() {  # try_dir -> 0 when the try is final: the batch marked it FINAL after the run
  [ -f "$1/FINAL" ] && [ ! -e "$1/interrupted_by_user" ]
}

tries() {  # phase_dir -> its try directories, oldest first
  ls -dv "$1"/try[0-9]* 2>/dev/null
}

final_try() {  # phase_dir -> the final try directory (empty: none)
  local d f=""
  for d in $(tries "$1"); do run_done "$d" && f="$d"; done
  echo "$f"
}

next_try() {  # phase_dir -> the next free try directory
  local n=1
  while [ -e "$1/try$n" ]; do n=$((n + 1)); done
  echo "$1/try$n"
}

# ------------------------------------------------------------------ dry run
if [ "$DRY" = 1 ]; then
  echo "batch dir: $BATCH_DIR   budgets: $BLIST   headless: $HEADLESS"
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

win_alive() {  # pid: a Windows process still running
  [ -n "$1" ] && "$TASKLIST" /FI "PID eq $1" /NH 2>/dev/null | tr -d '\r' | grep -qw "$1"
}

isaac_alive() {
  win_alive "$ISAAC_PID"
}

isaac_start() {  # run_dir
  local out
  out="$(ps_run "$(winpath "$REPO/sim/launch_a7.ps1")" -Seed "$SEED" -RunDir "$(winpath "$1")" -Headless "$HEADLESS" -Camera 0 9>&-)"
  ISAAC_PID="$(sed -n 's/^PID=//p' <<< "$out" | head -1)"
  if [ -z "$ISAAC_PID" ]; then log "    isaac launch failed: $(tr '\n' ' ' <<< "$out" | cut -c1-300)"; return 1; fi
  log "    isaac console pid $ISAAC_PID (headless=$HEADLESS)"
}

isaac_wait_ready() {  # run_dir timeout_s
  local t0=$SECONDS
  while [ $((SECONDS - t0)) -lt "$2" ]; do
    [ "$(sfield "$1/isaac_status.txt" state)" = playing ] && { log "    isaac playing after $((SECONDS - t0)) s"; return 0; }
    if grep -q "REFUSING TO FLY" "$1/isaac.log" 2>/dev/null; then log "    isaac refused to fly (no ROS) - see isaac.log"; return 1; fi
    if [ $((SECONDS - t0)) -gt 20 ] && ! isaac_alive; then log "    isaac process gone before it was ready - see isaac.log"; return 1; fi
    sleep 3
  done
  log "    isaac not ready after $2 s"; return 1
}

isaac_stop() {  # clean stop request, then the reaper until no Isaac process is left
  [ -n "$RUN_DIR" ] || return 0
  touch "$RUN_DIR/isaac_stop" 2>/dev/null
  local t0=$SECONDS out="" i closed=0
  while [ -f "$RUN_DIR/isaac_status.txt" ] && [ $((SECONDS - t0)) -lt 60 ]; do
    [ "$(sfield "$RUN_DIR/isaac_status.txt" state)" = closed ] && { closed=1; break; }
    isaac_alive || break
    sleep 3
  done
  if [ "$closed" = 1 ]; then                      # "closed" comes before Kit's own shutdown: let it finish
    t0=$SECONDS
    while isaac_alive && [ $((SECONDS - t0)) -lt 30 ]; do sleep 2; done
  fi
  for i in 1 2 3 4; do                            # a killed GPU process can take seconds to go
    out="$(isaac_reap "$ISAAC_PID")"
    grep "REAPED" <<< "$out" | sed 's/^/    reap: /'
    grep -q "ISAAC_PROCESSES=0" <<< "$out" && break
    sleep 5
  done
  grep -q "ISAAC_PROCESSES=0" <<< "$out" || log "    WARNING: Isaac processes left after the reaper ($(grep -o 'ISAAC_PROCESSES=[0-9]*' <<< "$out"))"
  ISAAC_PID=""
}

sim_host() {  # the address PX4 connects to: the Windows host on WSL's NAT vswitch
  local h="${AEB_SIM_HOST:-}"
  [ -n "$h" ] || h="$(ip route show default 2>/dev/null | awk '{print $3; exit}')"
  echo "${h:-172.19.80.1}"
}

win_4560() {  # Windows' sockets on TCP 4560 (listeners, connections), one per line
  "$NETSTAT" -ano 2>/dev/null | tr -d '\r' | grep -E ':4560[[:space:]]' | tr -s ' ' | sed 's/^ //'
}

link_check() {  # out_file -> LINK_RESULT: ok | dropped | refused | port_in_use | no_probe, LINK_INFO
  local out="$1" host pid t0 rc
  host="$(sim_host)"; LINK_RESULT=no_probe; LINK_INFO=""
  rm -f "$out" 2>/dev/null
  pid="$(ps_run "$(winpath "$REPO/sim/link_probe.ps1")" -OutFile "$(winpath "$out")" -Port 4560 -TimeoutS 40 9>&- \
         | sed -n 's/^PID=//p' | head -1)"
  if [ -z "$pid" ]; then LINK_INFO="sim/link_probe.ps1 did not start the probe"; return 1; fi
  t0=$SECONDS                                     # python.bat takes a few seconds to come up
  while [ $((SECONDS - t0)) -lt 60 ]; do
    grep -qE "^(LISTENING|PORT_IN_USE)" "$out" 2>/dev/null && break
    [ $((SECONDS - t0)) -gt 5 ] && ! win_alive "$pid" && break
    sleep 1
  done
  if grep -q "^PORT_IN_USE" "$out" 2>/dev/null; then
    LINK_RESULT=port_in_use
    LINK_INFO="another process holds 4560 on Windows: $(win_4560 | grep -i listen | head -2 | tr '\n' ';')"
  elif ! grep -q "^LISTENING" "$out" 2>/dev/null; then
    LINK_INFO="the probe never listened: $(tr -d '\r' < "$out" 2>/dev/null | tail -2 | tr '\n' ' ')"
  else
    timeout 10 bash -c "exec 3<>/dev/tcp/$host/4560" 2>/dev/null; rc=$?
    if [ "$rc" = 0 ]; then
      t0=$SECONDS
      while [ $((SECONDS - t0)) -lt 10 ]; do grep -q "^ACCEPTED" "$out" 2>/dev/null && break; sleep 1; done
      LINK_RESULT=ok
      LINK_INFO="$host:4560 reached; $(tr -d '\r' < "$out" | sed -n 's/^ACCEPTED /the probe accepted /p' | head -1)"
    elif [ "$rc" = 124 ]; then
      LINK_RESULT=dropped; LINK_INFO="no answer from $host:4560 in 10 s - the SYNs are dropped on the way (a firewall)"
    else
      LINK_RESULT=refused; LINK_INFO="$host:4560 refused the connection although the probe listened (a firewall rejecting)"
    fi
  fi
  "$TASKKILL" /T /F /PID "$pid" > /dev/null 2>&1
  [ "$LINK_RESULT" = ok ]
}

link_down() {  # the last link_check proved the link unusable for every run
  case "$LINK_RESULT" in dropped|refused|port_in_use) return 0 ;; esac
  return 1
}

LINK_FIX="check ESET Endpoint Security (the inbound allow for 172.19.80.0/20, above any block rule; the ESET firewall log) and the Windows Firewall rule \"Pegasus PX4 4560\" (docs/INSTALL_PEGASUS.md, A4), then: bash missions/batch_aerial.sh --link-check"

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
      -PidFile "$(winpath "$BATCH_DIR")\\keep_awake.pid" -MaxHours 24 > /dev/null 2>&1 9>&- &
  KA_BG=$!
}
keep_awake_stop() {
  local p=""; [ -f "$BATCH_DIR/keep_awake.pid" ] && p="$(tr -d '\r\n ' < "$BATCH_DIR/keep_awake.pid")"
  [ -n "$p" ] && "$TASKKILL" /F /PID "$p" > /dev/null 2>&1
  [ -n "$KA_BG" ] && kill "$KA_BG" 2>/dev/null
  rm -f "$BATCH_DIR/keep_awake.pid"
}

# ------------------------------------------------------------------ WSL side
px4_stop() {  # the SITL server, then its startup script (rcS runs on after a killed server)
  pkill -INT -x px4 2>/dev/null                   # ignored while rcS still runs (system())
  local i; for i in 1 2 3 4 5 6; do pgrep -x px4 > /dev/null || break; sleep 1; done
  if pgrep -x px4 > /dev/null; then pkill -KILL -x px4 2>/dev/null; sleep 1; fi
  pkill -f "init.d-posix/rcS" 2>/dev/null
  return 0
}

px4_start() {  # log
  px4_stop
  local host; host="$(sim_host)"
  ( cd "$PX4_RUN" && exec env PX4_SIM_HOSTNAME="$host" PX4_SIM_MODEL=gazebo-classic_iris setsid "$PX4_BIN" -d \
      "$PX4_DIR/ROMFS/px4fmu_common/" -s "$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/rcS" -i 0 ) > "$1" 2>&1 < /dev/null 9>&- &
  log "    px4 started (daemon, sim host $host)"
}

link_diag() {  # run_dir: both sides' sockets on 4560 while PX4 still tries (no heartbeat)
  local f="$1/link_diag.txt"
  {
    echo "# $(date -Is) no PX4 heartbeat - TCP 4560 on both sides while PX4 still connects to $(sim_host)"
    echo "## WSL: ss -tanp   (SYN-SENT that stays = the SYNs get no answer: dropped by a firewall)"
    ss -tanp 2>/dev/null | grep -E ':4560\b' || echo "(no socket on 4560 in WSL: refused at once, or PX4 is not connecting)"
    echo "## Windows: netstat -ano   (LISTENING = Pegasus is up; SYN_RECEIVED / ESTABLISHED = packets arrive)"
    win_4560 || echo "(no socket on 4560 on Windows: Pegasus is not listening)"
    echo "## WSL: addresses, routes"
    ip -4 -o addr show 2>/dev/null; ip route 2>/dev/null
    echo "## px4.log (tail)"
    tail -n 4 "$1/px4.log" 2>/dev/null
  } > "$f" 2>&1
  log "    link diag: WSL SYN-SENT on 4560: $(grep -v '^#' "$f" | grep -c SYN-SENT), Windows listeners: $(grep -v '^#' "$f" | grep -c LISTENING) (link_diag.txt)"
}

wait_heartbeat() {  # run_dir timeout_s
  local t0=$SECONDS
  while [ $((SECONDS - t0)) -lt "$2" ]; do
    [ "$(sfield "$1/isaac_status.txt" heartbeat)" = True ] && { log "    PX4 <-> Isaac linked after $((SECONDS - t0)) s"; return 0; }
    if [ $((SECONDS - t0)) -gt 10 ] && ! pgrep -x px4 > /dev/null; then log "    px4 exited - see px4.log"; return 1; fi
    sleep 3
  done
  log "    no PX4 heartbeat on the Isaac side after $2 s"; return 1
}

bg_script() {  # var logfile script args... ; the script runs in its own session, $var = its pid
  local var="$1" logf="$2"; shift 2
  setsid env AEB_LOG_DIR="$RUN_DIR" bash "$@" > "$logf" 2>&1 < /dev/null 9>&- &
  printf -v "$var" '%s' "$!"
}

stop_script() {  # pid: TERM runs the script's own teardown (nodes stopped, DDS cleaned)
  local pid="$1" i; [ -n "$pid" ] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  kill -TERM "$pid" 2>/dev/null
  for i in $(seq 1 45); do kill -0 "$pid" 2>/dev/null || return 0; sleep 1; done
  kill -KILL -- -"$pid" 2>/dev/null; kill -KILL "$pid" 2>/dev/null
}

topic_alive() {  # topic timeout_s
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

start_runner() {  # logfile cmd... ; own session: a Ctrl+C in this terminal reaches only the batch,
  local logf="$1"; shift                          # which then stops the runner exactly once
  ( cd "$REPO" && exec setsid "$@" ) > "$logf" 2>&1 < /dev/null 9>&- &
  RUNNER_PID=$!
}

stop_runner() {  # SIGTERM = KeyboardInterrupt in the runner: it lands and writes its manifest
  local i; [ -n "$RUNNER_PID" ] || return 0
  if kill -0 "$RUNNER_PID" 2>/dev/null; then
    kill -TERM "$RUNNER_PID" 2>/dev/null
    for i in $(seq 1 150); do kill -0 "$RUNNER_PID" 2>/dev/null || break; sleep 1; done
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
  [ "$SUP_RESULT" = "done" ] || log "    $SUP_RESULT - stopping the runner (it lands and writes its manifest)"
  stop_runner
  show_progress "$rd/runner.txt"
}

teardown_run() {  # Isaac before PX4: Pegasus runs in lockstep and blocks on PX4's next message, so a
  stop_runner     # PX4 stopped first freezes Isaac before it reads its stop file (smoke test 2026-09-25:
  stop_script "$LOC_PID"; LOC_PID=""              # 60 s lost per run, Isaac killed instead of closed)
  stop_script "$ODOM_PID"; ODOM_PID=""
  isaac_stop
  px4_stop
}

# ------------------------------------------------------------------ one run
run_once() {  # phase run_dir map -> RUN_RESULT (done|setup|sim_dead|error), RUN_STATUS
  RUN_ACTIVE=1                                    # a Ctrl+C now marks this run interrupted_by_user
  run_steps "$@"
  RUN_ACTIVE=0
}

run_steps() {
  local phase="$1" map="$3" tmo args
  RUN_DIR="$2"; SHOWN=0; SUP_RESULT=""; RUNNER_PID=""; LOC_PID=""; ODOM_PID=""; ISAAC_PID=""; LINK_RESULT=""
  mkdir -p "$RUN_DIR"
  isaac_reap | grep -E "REAPED" | sed 's/^/    stale: /'
  px4_stop
  clock_check > "$RUN_DIR/clock_offset.txt" 2>&1
  RUN_RESULT=setup
  if ! isaac_start "$RUN_DIR"; then RUN_STATUS=ISAAC_LAUNCH; teardown_run; return; fi
  if ! isaac_wait_ready "$RUN_DIR" "$ISAAC_READY_S"; then RUN_STATUS=ISAAC_NOT_READY; teardown_run; return; fi
  px4_start "$RUN_DIR/px4.log"
  if ! wait_heartbeat "$RUN_DIR" "$HEARTBEAT_S"; then
    RUN_STATUS=NO_HEARTBEAT
    link_diag "$RUN_DIR"
    teardown_run                                  # Isaac gone: port 4560 is free for the probe
    link_check "$RUN_DIR/link_probe.txt"
    log "    link test after the failure: $LINK_RESULT${LINK_INFO:+ - $LINK_INFO}"
    return
  fi
  bg_script ODOM_PID "$RUN_DIR/odom_script.txt" "$REPO/missions/aerial_odom.sh" "$SEED"
  # the script's gates are one-shot; its last line comes after them, then the topic itself decides
  if ! wait_line "$RUN_DIR/odom_script.txt" "odom\] running" 150 "$ODOM_PID" "died|not found" \
      || ! topic_alive /odom_rf2o 60; then
    log "    odometry chain not ready - see odom_script.txt"; RUN_STATUS=ODOM_NOT_READY; teardown_run; return
  fi
  log "    odometry flowing"
  if [ "$phase" = explore ]; then
    bg_script LOC_PID "$RUN_DIR/slam_script.txt" "$REPO/missions/aerial_slam.sh" "$SEED"
    if ! wait_line "$RUN_DIR/slam_script.txt" "slam\] running" 150 "$LOC_PID" "launch died"; then
      log "    mapper not ready - see slam_script.txt"; RUN_STATUS=SLAM_NOT_READY; teardown_run; return
    fi
    log "    mapper up; exploring (budgets $BUDGETS_CSV sim-min)"
    start_runner "$RUN_DIR/runner.txt" python3 -u "$REPO/missions/aerial_explore_runner.py" \
        --seed "$SEED" --budgets "$BUDGETS_CSV" --max-impacts "$MAX_IMPACTS" --out-dir "$RUN_DIR"
    tmo=$(awk -v m="$EXPLORE_TIMEOUT_MIN" 'BEGIN { print int(m * 60) }')
  else
    bg_script LOC_PID "$RUN_DIR/amcl_script.txt" "$REPO/missions/aerial_amcl.sh" "$SEED" "$map"
    if ! wait_line "$RUN_DIR/amcl_script.txt" "amcl\] running" 240 "$LOC_PID" "launch died" \
        || ! grep -q "/map published" "$RUN_DIR/amcl_script.txt"; then
      log "    localizer not ready - see amcl_script.txt"; RUN_STATUS=AMCL_NOT_READY; teardown_run; return
    fi
    log "    AMCL up on $(basename "$map"); touring"
    args=(--seed "$SEED" --map "$map" --repel-gain 1.0 --max-impacts "$MAX_IMPACTS" --out-dir "$RUN_DIR")
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
    if [ "$RUN_STATUS" = NO_HEARTBEAT ] && link_down; then
      log "  network: PX4 cannot reach Pegasus (WSL -> Windows TCP 4560: $LINK_RESULT) - every run would fail the same way"
      log "  $LINK_FIX"
      LINK_IS_DOWN=1; return 1
    fi
  done
  log "  $phase: FAILED after $MAX_TRIES tries"
  return 1
}

# ------------------------------------------------------------------ batch
STOPPING=0; RUN_ACTIVE=0; RUN_DIR=""; LINK_IS_DOWN=0
on_signal() {
  [ "$STOPPING" = 1 ] && return
  STOPPING=1
  if [ "$RUN_ACTIVE" = 1 ]; then
    log "interrupted - stopping the current run cleanly (the vehicle lands, Isaac closes) ..."
    [ -n "$RUN_DIR" ] && [ -d "$RUN_DIR" ] && touch "$RUN_DIR/interrupted_by_user"
    teardown_run
  fi
  log "batch stopped by the user; the same command resumes"
  exit 130
}
trap on_signal INT TERM
trap keep_awake_stop EXIT

exec 9> /tmp/aeb_batch.lock
flock -n 9 || { echo "another batch is already running on this machine (it owns Isaac and PX4)"; exit 1; }
case "$BATCH_DIR" in /mnt/[a-z]/*) ;; *) echo "the batch dir must be on a Windows drive (/mnt/<x>/...): $BATCH_DIR"; exit 2 ;; esac
[ -x "$PS" ] || { echo "PowerShell not found: $PS"; exit 2; }

# ------------------------------------------------------------------ link check only
if [ "$LINK_ONLY" = 1 ]; then                     # no reaping here: a manual Isaac session may be open
  echo "PX4 -> Pegasus link test: a listener in Isaac's python on Windows 0.0.0.0:4560, a connect from WSL to $(sim_host) ..."
  link_check "$BATCH_DIR/link_probe.txt"
  echo "LINK_$(tr '[:lower:]' '[:upper:]' <<< "$LINK_RESULT")${LINK_INFO:+ - $LINK_INFO}"
  if [ "$LINK_RESULT" = port_in_use ]; then echo "close the app that holds 4560 (an Isaac window still open?) and run the test again"
  elif link_down; then echo "$LINK_FIX"; fi
  [ "$LINK_RESULT" = ok ]; exit $?
fi

# pre-flight: every missing piece stops the batch before the first launch, not at 2 a.m.
source /opt/ros/jazzy/setup.bash
bad=0
[ -x "$PX4_BIN" ] || { echo "PX4 SITL binary not found: $PX4_BIN"; bad=1; }
[ -d "$PX4_RUN" ] || { echo "PX4 run dir not found: $PX4_RUN"; bad=1; }
for SEED in "${SEEDS[@]}"; do
  [ -f "$REPO/worlds/manifests/world_$SEED.csv" ] || { echo "no world manifest for $SEED"; bad=1; }
  [ -f "$REPO/missions/goals/goals_$SEED.csv" ] || { echo "no goal file for $SEED"; bad=1; }
done
python3 -c "import numpy, yaml, matplotlib, pymavlink, rclpy" 2>/dev/null || { echo "python deps missing (numpy yaml matplotlib pymavlink rclpy)"; bad=1; }
python3 "$REPO/missions/cli_check.py" > /dev/null || { echo "missions/cli_check.py failed - a parameter is missing from a parser"; bad=1; }
[ "$bad" = 0 ] || exit 2
HOST_IP="$(sim_host)"
PEER="$(grep -oE '[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+' "$HOME/.ros/fastdds.xml" 2>/dev/null | head -1)"
FREE_C="$(df -BG --output=avail "$BATCH_DIR" 2>/dev/null | tail -1 | tr -d ' G')"
FREE_H="$(df -BG --output=avail "$HOME" 2>/dev/null | tail -1 | tr -d ' G')"

log "=== aerial batch: ${SEEDS[*]} | budgets $BLIST | headless $HEADLESS | tour-limit $TOUR_LIMIT | max-impacts $MAX_IMPACTS | dir ${BATCH_DIR#$REPO/}"
log "    host $HOST_IP, FastDDS peer ${PEER:-?}; free: ${FREE_C:-?} GB on the batch drive, ${FREE_H:-?} GB in WSL"
[ -n "$PEER" ] && [ "$PEER" != "$HOST_IP" ] && log "    WARNING: ~/.ros/fastdds.xml peer $PEER differs from the host $HOST_IP - ROS may not see Isaac"
[ -n "$FREE_H" ] && [ "$FREE_H" -lt 10 ] && log "    WARNING: less than 10 GB free in WSL (PX4 logs grow per flight: ~/px4_run/log)"
if [ "${AEB_SKIP_LINK_CHECK:-0}" = 1 ]; then
  log "    link test skipped (AEB_SKIP_LINK_CHECK=1)"
else
  isaac_reap | grep -E "REAPED" | sed 's/^/    stale: /'      # a leftover Isaac would hold 4560
  px4_stop
  link_check "$BATCH_DIR/link_probe.txt"
  log "    link test: $LINK_RESULT${LINK_INFO:+ - $LINK_INFO}"
  if link_down; then
    log "    PX4 could not reach Pegasus: no run can fly. $LINK_FIX"
    log "=== batch not started"; exit 3
  fi
fi
keep_awake_start

for SEED in "${SEEDS[@]}"; do
  [ "$STOPPING" = 1 ] && break
  WDIR="$BATCH_DIR/$SEED"
  log "world $SEED"
  if [ "$ONLY" != tours ]; then
    if ! do_phase explore "$WDIR/explore" ""; then
      [ "$LINK_IS_DOWN" = 1 ] && break
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
      do_phase "tour_b$b" "$WDIR/tour_b$b" "$map" || { [ "$LINK_IS_DOWN" = 1 ] && break; }
    done
  fi
  [ "$LINK_IS_DOWN" = 1 ] && break
  log "world $SEED finished"
done
if [ "$LINK_IS_DOWN" = 1 ]; then log "=== batch stopped: the PX4 -> Pegasus link is down (fix it, then the same command resumes)"; exit 3; fi
log "=== batch end"
