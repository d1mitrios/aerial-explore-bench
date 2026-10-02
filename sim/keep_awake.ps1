# === Keep Windows awake while the batch runs (no settings are changed) ===
# SetThreadExecutionState(ES_CONTINUOUS | ES_SYSTEM_REQUIRED) held by this process: the
# machine does not go to sleep while it lives (the screen may still turn off). Started by
# missions/batch_aerial.sh, which ends it at the end of the batch (taskkill on the PID it
# writes); it also ends by itself after -MaxHours as a safety net.
#
#   powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$(wslpath -w sim/keep_awake.ps1)" -PidFile <path> [-MaxHours 20]
param(
    [Parameter(Mandatory = $true)][string]$PidFile,
    [int]$MaxHours = 20
)
Set-Content -Path $PidFile -Value $PID -Encoding ASCII
Add-Type -Namespace Aeb -Name Power -MemberDefinition '[DllImport("kernel32.dll")] public static extern uint SetThreadExecutionState(uint esFlags);'
[Aeb.Power]::SetThreadExecutionState([uint32]2147483649) | Out-Null
$until = (Get-Date).AddHours($MaxHours)
while ((Get-Date) -lt $until) { Start-Sleep -Seconds 30 }
[Aeb.Power]::SetThreadExecutionState([uint32]2147483648) | Out-Null
