# Environment for the Isaac-side apps (dot-source it in the PowerShell window that will
# run python.bat; the values must exist BEFORE the process starts):
#
#     . C:\path\to\aerial-explore-bench\sim\env.ps1
#
# Sets the ROS 2 bridge variables (Isaac's internal Humble libraries on PATH, FastDDS
# UDPv4-only profile from this repo, domain 0, FastDDS RMW) and the app defaults.
# Override any AEB_* variable after sourcing, e.g.  $env:AEB_WORLD_SEED="20260723008".

$repo = Split-Path -Parent $PSScriptRoot
$isaac = if ($env:ISAACSIM_PATH) { $env:ISAACSIM_PATH } else { "C:\isaacsim" }

$env:ROS_DOMAIN_ID = "0"
$env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"
$env:FASTRTPS_DEFAULT_PROFILES_FILE = Join-Path $repo "sim\fastdds_win.xml"
$rosLib = Join-Path $isaac "exts\isaacsim.ros2.core\humble\lib"
if (-not (($env:PATH -split ";") -contains $rosLib)) { $env:PATH = "$env:PATH;$rosLib" }

if (-not $env:AEB_WORLD_SEED) { $env:AEB_WORLD_SEED = "20260723001" }
if (-not $env:AEB_VIO) { $env:AEB_VIO = "1" }
if (-not $env:AEB_RUNS_DIR) { $env:AEB_RUNS_DIR = Join-Path $repo "runs\raw" }

Write-Host "[aeb] repo      = $repo"
Write-Host "[aeb] isaac     = $isaac  (ROS lib on PATH: $(Test-Path $rosLib))"
Write-Host "[aeb] profile   = $env:FASTRTPS_DEFAULT_PROFILES_FILE  (exists: $(Test-Path $env:FASTRTPS_DEFAULT_PROFILES_FILE))"
Write-Host "[aeb] world     = $env:AEB_WORLD_SEED   vio = $env:AEB_VIO   runs = $env:AEB_RUNS_DIR"
Write-Host "[aeb] launch:   & '$isaac\python.bat' -u '$repo\sim\a7_lidar_flight.py' *>&1 | Tee-Object -FilePath '$env:AEB_RUNS_DIR\a7_log.txt'"
