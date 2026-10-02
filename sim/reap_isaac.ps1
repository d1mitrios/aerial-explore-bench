# === End a run's Isaac process tree, and any Isaac process left over from earlier runs ===
# Used by missions/batch_aerial.sh, missions/batch_wheeled.sh and missions/wheeled_check.sh after the
# clean stop request (isaac_stop) has had its chance, and before every launch (a crashed run
# can leave Kit holding the GPU and TCP 4560).
# Only processes whose executable lives under the Isaac install are touched - never another
# Python on the machine.
#
#   powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w sim/reap_isaac.ps1)" [-ConsolePid <n>]
param(
    [int]$ConsolePid = 0
)
$isaac = if ($env:ISAACSIM_PATH) { $env:ISAACSIM_PATH } else { "C:\isaacsim" }
if ($ConsolePid -gt 0) {
    & taskkill.exe /T /F /PID $ConsolePid 2>$null | Out-Null
}
$left = @(Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.Path -and $_.Path.StartsWith($isaac, [System.StringComparison]::OrdinalIgnoreCase) })
foreach ($p in $left) {
    try { Stop-Process -Id $p.Id -Force -ErrorAction Stop; Write-Output "REAPED=$($p.Id) $($p.ProcessName)" } catch { }
}
Start-Sleep -Milliseconds 500
$still = @(Get-Process -ErrorAction SilentlyContinue | Where-Object { $_.Path -and $_.Path.StartsWith($isaac, [System.StringComparison]::OrdinalIgnoreCase) })
Write-Output "ISAAC_PROCESSES=$($still.Count)"
