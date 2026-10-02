# === Launch the A7 flight app detached - for the batch runner (missions/batch_aerial.sh) ===
# Starts <isaac>\python.bat -u sim\a7_lidar_flight.py in its own minimized console with the
# environment of sim\env.ps1 plus the per-run settings; stdout + stderr go to
# <RunDir>\isaac.log. Prints "PID=<n>" (the console process: `taskkill /T /F /PID <n>`
# ends the whole tree). The app keeps <RunDir>\isaac_status.txt overwritten with one status
# line (state, frame, sim_t, rtf_now, PX4 heartbeat) and stops cleanly when
# <RunDir>\isaac_stop appears (sim/aeb_flight.py, AEB_STATUS_FILE / AEB_STOP_FILE).
# Ground truth and scan samples land in <RunDir> (AEB_RUNS_DIR), next to the runner's outputs.
#
# From WSL:
#   powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w sim/launch_a7.ps1)" \
#       -Seed 20260723008 -RunDir "$(wslpath -w runs/raw/batch/20260723008/explore)" [-Headless 1] [-Camera 0]
param(
    [Parameter(Mandatory = $true)][string]$Seed,
    [Parameter(Mandatory = $true)][string]$RunDir,
    [int]$Headless = 1,
    [int]$Camera = 0
)
$ErrorActionPreference = "Stop"

New-Item -ItemType Directory -Force -Path $RunDir | Out-Null
$env:AEB_RUNS_DIR = $RunDir          # before env.ps1, which keeps an existing value
$env:AEB_WORLD_SEED = $Seed
. (Join-Path $PSScriptRoot "env.ps1") 6>$null    # $repo, $isaac and the ROS variables; its banner suppressed
$env:AEB_VIO = "1"                   # the IMU kit (AEB_CAMERA decides the camera)
$env:AEB_CAMERA = "$Camera"
$env:AEB_HEADLESS = "$Headless"
$env:AEB_STATUS_FILE = Join-Path $RunDir "isaac_status.txt"
$env:AEB_STOP_FILE = Join-Path $RunDir "isaac_stop"
Remove-Item -ErrorAction SilentlyContinue -Force $env:AEB_STATUS_FILE, $env:AEB_STOP_FILE

$log = Join-Path $RunDir "isaac.log"
$app = Join-Path $repo "sim\a7_lidar_flight.py"
$python = Join-Path $isaac "python.bat"
if (-not (Test-Path $python)) { Write-Output "ERROR=python.bat not found at $python"; exit 2 }
if (-not (Test-Path $app)) { Write-Output "ERROR=app not found at $app"; exit 2 }

# a .cmd per run: cmd's own redirection merges both streams into one log (Start-Process
# cannot), and the console title names the run in the taskbar
$cmdFile = Join-Path $RunDir "isaac_run.cmd"
$lines = @(
    "@echo off",
    "title AEB A7 world $Seed",
    "call `"$python`" -u `"$app`" > `"$log`" 2>&1"
)
Set-Content -Path $cmdFile -Value $lines -Encoding ASCII

$p = Start-Process -FilePath $cmdFile -WorkingDirectory $repo -WindowStyle Minimized -PassThru
Write-Output "PID=$($p.Id)"
Write-Output "LOG=$log"
