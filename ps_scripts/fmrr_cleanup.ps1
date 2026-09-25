# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

param(
    [ValidateSet("service", "legacy")]
    [string]$Mode = "service",

    # all: local GUI + remote bring-up; gui: only the local GUI; remote: only the remote side
    [ValidateSet("all", "gui", "remote")]
    [string]$Target = "all",

    [int]$UdpPort = 5005
)

# Also called by fmrr_gui.ps1, which runs with "Stop": keep native tools
# (taskkill writing to stderr) from turning into terminating errors here.
$ErrorActionPreference = "Continue"

$remoteUser = "fit4med"
$remoteHost = "192.168.1.1"
$remoteScript = "/home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/kill_fmrr_apps.sh"

# ---------------------------------------------------------------------------
# Local GUI cleanup
# ---------------------------------------------------------------------------
function Stop-ProcessTree([int]$ProcessId, [string]$Reason) {
    $proc = Get-Process -Id $ProcessId -ErrorAction SilentlyContinue
    if (-not $proc) { return }
    Write-Host "  killing PID $ProcessId ($($proc.ProcessName)) - $Reason"
    # /T also takes down the child spawned by the venv python.exe launcher.
    & taskkill.exe /PID $ProcessId /T /F 2>&1 | Out-Null
}

function Stop-FmrrGui {
    Write-Host "Cleaning up local GUI processes..." -ForegroundColor Cyan
    $procs = Get-CimInstance Win32_Process -ErrorAction SilentlyContinue

    # Never kill ourselves or our ancestors: when called from fmrr_gui.ps1 the
    # launching console may itself have "fmrr_gui.ps1" in its command line.
    $protected = @{}
    $cursor = $PID
    while ($cursor -and -not $protected.ContainsKey([int]$cursor)) {
        $protected[[int]$cursor] = $true
        $cursor = ($procs | Where-Object { $_.ProcessId -eq $cursor }).ParentProcessId
    }

    # 1. The GUI itself (python running FMRRMainProgram.py, venv launcher included).
    $gui = @($procs | Where-Object { $_.CommandLine -match 'FMRRMainProgram\.py' -and -not $protected.ContainsKey([int]$_.ProcessId) })
    foreach ($p in $gui) { Stop-ProcessTree $p.ProcessId "FMRRMainProgram.py" }

    # 2. Windows Error Reporting instances holding a crashed GUI alive: while
    #    WerFault.exe is dumping/showing its dialog the crashed process (and its
    #    UDP socket) does not go away.
    $guiPids = $gui | ForEach-Object { $_.ProcessId }
    foreach ($w in @($procs | Where-Object { $_.Name -match '^WerFault(Secure)?\.exe$' })) {
        foreach ($gp in $guiPids) {
            if ($w.CommandLine -match "-p\s+$gp(\s|$)") { Stop-ProcessTree $w.ProcessId "WER for GUI PID $gp" }
        }
    }

    # 3. PowerShell hosts of fmrr_gui.ps1 (e.g. stuck on "Press Enter to close").
    foreach ($p in @($procs | Where-Object { $_.CommandLine -match 'fmrr_gui\.ps1' -and -not $protected.ContainsKey([int]$_.ProcessId) })) {
        Stop-ProcessTree $p.ProcessId "fmrr_gui.ps1 host"
    }

    # 4. Anything still bound to the UDP port, whatever its command line.
    Start-Sleep -Milliseconds 500
    $owners = @(Get-NetUDPEndpoint -LocalPort $UdpPort -ErrorAction SilentlyContinue |
                Select-Object -ExpandProperty OwningProcess -Unique)
    foreach ($o in $owners) {
        if ($o -eq 0 -or $o -eq 4 -or $protected.ContainsKey([int]$o)) { continue }  # Idle / System / us
        Stop-ProcessTree $o "bound to UDP $UdpPort"
    }

    Start-Sleep -Milliseconds 500
    $left = @(Get-NetUDPEndpoint -LocalPort $UdpPort -ErrorAction SilentlyContinue)
    if ($left.Count -gt 0) {
        Write-Host "UDP port $UdpPort still bound by PID(s): $(($left.OwningProcess | Select-Object -Unique) -join ', ')" -ForegroundColor Red
        Write-Host "Run this script from an elevated PowerShell if the owner belongs to another user." -ForegroundColor Yellow
    }
    else {
        Write-Host "UDP port $UdpPort is free." -ForegroundColor Green
    }
}

# ---------------------------------------------------------------------------
# Remote bring-up cleanup
# ---------------------------------------------------------------------------
function Stop-FmrrRemote {
    Write-Host "Cleaning up remote processes on ${remoteHost}..." -ForegroundColor Cyan
    if($Mode -eq "service") {
      ssh -T "${remoteUser}@${remoteHost}" "systemctl --user stop 'fit4med-bringup@*.service'; bash -lc 'source /opt/ros/jazzy/setup.bash; $remoteScript; ros2 daemon stop'"
    }
    else {
      ssh -t "${remoteUser}@${remoteHost}" "bash -l -c '$remoteScript'"
    }
}

# GUI first, so it does not keep talking to (or reconnecting to) the robot side.
if ($Target -in @("all", "gui"))    { Stop-FmrrGui }
if ($Target -in @("all", "remote")) { Stop-FmrrRemote }
