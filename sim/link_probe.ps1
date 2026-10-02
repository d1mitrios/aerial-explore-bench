# === Start sim\link_probe.py detached, exactly the way launch_a7.ps1 starts the flight app ===
# The PX4 -> Pegasus link check of missions/batch_aerial.sh: <isaac>\python.bat runs the probe
# (a one-shot TCP listener on 0.0.0.0:<Port>) in its own minimized console, started through
# Start-Process of a .cmd - the same program, launch path and parent chain as the Isaac app
# in a batch run, so ESET / Windows Firewall judge it the same way. The probe's lines
# (LISTENING, then ACCEPTED / TIMEOUT / PORT_IN_USE) go to <OutFile>; the batch connects
# from WSL once LISTENING appears. Prints "PID=<n>" (the console; taskkill /T /F ends it).
#
# From WSL:
#   powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w sim/link_probe.ps1)" \
#       -OutFile "$(wslpath -w runs/raw/batch/link_probe.txt)" [-Port 4560] [-TimeoutS 40]
param(
    [Parameter(Mandatory = $true)][string]$OutFile,
    [int]$Port = 4560,
    [int]$TimeoutS = 40
)
$ErrorActionPreference = "Stop"

. (Join-Path $PSScriptRoot "env.ps1") 6>$null    # $repo, $isaac (the ROS variables are harmless here)
$python = Join-Path $isaac "python.bat"
$probe = Join-Path $repo "sim\link_probe.py"
if (-not (Test-Path $python)) { Write-Output "ERROR=python.bat not found at $python"; exit 2 }
if (-not (Test-Path $probe)) { Write-Output "ERROR=probe not found at $probe"; exit 2 }
Remove-Item -ErrorAction SilentlyContinue -Force $OutFile

$cmdFile = [System.IO.Path]::ChangeExtension($OutFile, ".cmd")
$lines = @(
    "@echo off",
    "title AEB link probe (TCP $Port)",
    "call `"$python`" -u `"$probe`" --port $Port --timeout $TimeoutS > `"$OutFile`" 2>&1"
)
Set-Content -Path $cmdFile -Value $lines -Encoding ASCII

$p = Start-Process -FilePath $cmdFile -WorkingDirectory $repo -WindowStyle Minimized -PassThru
Write-Output "PID=$($p.Id)"
