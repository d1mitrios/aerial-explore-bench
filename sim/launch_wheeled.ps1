# === Launch one wheeled-arm run of Isaac detached - for the WSL scripts (missions/wheeled_check.sh, the batch) ===
# Starts the full Isaac Sim app the way the wheeled baseline does (isaac-sim.bat --exec, GUI
# by default) with sim\wheeled_bootstrap.py: the robot stage sim\wheeled\robot.usda, the world
# of <Seed> from worlds\manifests, the baseline's lidar and odometry publishers, the ground
# truth, /aeb/sim_time, then Play. Environment: the baseline's ROS 2 variables (domain 0,
# FastDDS; this repo's UDPv4 profile sim\fastdds_win.xml - the one every aerial run used) plus
# the per-run settings. stdout + stderr go to <RunDir>\isaac.log. Prints "PID=<n>" (the
# console process: `taskkill /T /F /PID <n>` ends the whole tree). The app keeps
# <RunDir>\isaac_status.txt overwritten with one status line (state, frame, sim_t, rtf_now,
# x, y) and stops cleanly when <RunDir>\isaac_stop appears. Ground truth, the odometry log
# and current_seed.txt land in <RunDir>.
#
#   -Window 1   the app's window, as the baseline ran (default)
#   -Window 0   Kit's --no-window
#   -OdomTf 0   the odometry publisher sends no TF (the shared executive's runs take
#               odom->base_link and base_link->lidar_link from rf2o + lidar_odom_bridge.py);
#               default 1 = the baseline's TF, for the Nav2 stack
#
# From WSL:
#   powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w sim/launch_wheeled.ps1)" \
#       -Seed 20260723008 -RunDir "$(wslpath -w runs/raw/wheeled_check/<ts>)" [-Window 1]
param(
    [Parameter(Mandatory = $true)][string]$Seed,
    [Parameter(Mandatory = $true)][string]$RunDir,
    [int]$Window = 1,
    [int]$OdomTf = 1
)
$ErrorActionPreference = "Stop"

New-Item -ItemType Directory -Force -Path $RunDir | Out-Null
$repo = Split-Path -Parent $PSScriptRoot
$isaac = if ($env:ISAACSIM_PATH) { $env:ISAACSIM_PATH } else { "C:\isaacsim" }
# The baseline's ROS environment (its launch_isaac.bat): domain 0, FastDDS, a UDPv4-only profile.
# NO ROS library folder on PATH: the full app's ROS 2 extension loads its own internal
# libraries (jazzy), and a second copy on PATH breaks it - sim\env.ps1 puts humble's there for
# the standalone python.bat apps, and the first check died on it ("DLL load failed while
# importing _rclpy_pybind11: The specified procedure could not be found", 2026-09-26).
$env:ROS_DOMAIN_ID = "0"
$env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"
$env:FASTRTPS_DEFAULT_PROFILES_FILE = Join-Path $repo "sim\fastdds_win.xml"
$env:PATH = (($env:PATH -split ";") | Where-Object { $_ -and ($_ -notmatch "isaacsim\.ros2") }) -join ";"
$env:AEB_REPO = $repo
$env:AEB_RUNS_DIR = $RunDir
$env:AEB_WORLD_SEED = $Seed
$env:AEB_STATUS_FILE = Join-Path $RunDir "isaac_status.txt"
$env:AEB_STOP_FILE = Join-Path $RunDir "isaac_stop"
Remove-Item -ErrorAction SilentlyContinue -Force $env:AEB_STATUS_FILE, $env:AEB_STOP_FILE
Remove-Item -ErrorAction SilentlyContinue Env:\AEB_ODOM_NOISE_FILE    # the publisher's defaults (noise 0.03 / 0.05 / 0)
$env:AEB_ODOM_TF = "$OdomTf"

$log = Join-Path $RunDir "isaac.log"
$app = Join-Path $repo "sim\wheeled_bootstrap.py"
$kit = Join-Path $isaac "isaac-sim.bat"
if (-not (Test-Path $kit)) { Write-Output "ERROR=isaac-sim.bat not found at $kit"; exit 2 }
if (-not (Test-Path $app)) { Write-Output "ERROR=bootstrap not found at $app"; exit 2 }
if (-not (Test-Path (Join-Path $repo "worlds\manifests\world_$Seed.csv"))) { Write-Output "ERROR=no manifest for world $Seed"; exit 2 }

# ignoreUnsavedOnExit: the run edits the stage (the world) and never saves it - no prompt at exit
$kitArgs = "--exec `"$app`" --/app/file/ignoreUnsavedOnExit=true"
if ($Window -eq 0) { $kitArgs += " --no-window" }

# a .cmd per run: cmd's own redirection merges both streams into one log (Start-Process
# cannot), and the console title names the run in the taskbar
$cmdFile = Join-Path $RunDir "isaac_run.cmd"
$lines = @(
    "@echo off",
    "title AEB wheeled world $Seed",
    "cd /d `"$isaac`"",
    "call `"$kit`" $kitArgs > `"$log`" 2>&1"
)
Set-Content -Path $cmdFile -Value $lines -Encoding ASCII

$p = Start-Process -FilePath $cmdFile -WorkingDirectory $isaac -WindowStyle Minimized -PassThru
Write-Output "PID=$($p.Id)"
Write-Output "LOG=$log"
