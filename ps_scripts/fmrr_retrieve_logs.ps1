# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

# ####################################### #######################################
# About ethercat.service being root: retrieving its logs doesn't need root. 
# It only needs fit4med to be allowed to read the system journal. 
# Run this once on the Linux PC:
# sudo usermod -aG systemd-journal fit4med 
# ####################################### #######################################

param(
    [string]$GuiIp = "192.168.1.2",
    # Where FMRRMainProgram.py writes its session logs and their backups (see rehab_gui/session_log.py)
    [string]$GuiLogDir = "C:\temp"
)

# Define variables
$remoteUser = "fit4med"
$remoteHost = "192.168.1.1"
$scpPathFiles = "/home/fit4med/.ros/fit4med_log/*"
$localDestination = Join-Path $env:USERPROFILE "Desktop\fit4med_logs\"
$remoteTarget = "${remoteUser}@${remoteHost}:$scpPathFiles"

# Ensure local destination exists
if (!(Test-Path $localDestination)) {
    New-Item -ItemType Directory -Path $localDestination | Out-Null
}

$combinedLog = Join-Path $localDestination "fit4med_combined.log"
$unit = "fit4med-bringup@$GuiIp.service"

ssh -T "${remoteUser}@${remoteHost}" `
    "journalctl --user -u $unit -b -o short-precise --no-pager" `
    | Out-File -FilePath $combinedLog -Encoding utf8

# EtherCAT master: ethercat.service is a system (root) unit, but reading its
# journal needs no root, only membership of the 'systemd-journal' (or 'adm')
# group. Without it journalctl prints a "Hint: ... not seeing messages" line.
# The IgH master itself logs to the kernel log, hence the journalctl -k part.
$ethercatLog = Join-Path $localDestination "ethercat_service.log"
$ethercatCmd = @(
    "echo '===== systemctl status ethercat.service ====='",
    "systemctl status ethercat.service --no-pager -l",
    "echo; echo '===== journalctl -u ethercat.service (this boot) ====='",
    "journalctl -u ethercat.service -b -o short-precise --no-pager",
    "echo; echo '===== kernel log, EtherCAT lines (this boot) ====='",
    "journalctl -k -b -o short-precise --no-pager | grep -i ethercat",
    "echo; echo '===== ethercat master ====='",
    "ethercat master",
    "echo; echo '===== ethercat slaves -v ====='",
    "ethercat slaves -v"
) -join "; "
Write-Output "Retrieve ethercat.service status/journal to $ethercatLog"
ssh -T "${remoteUser}@${remoteHost}" "$ethercatCmd" 2>&1 | Out-File -FilePath $ethercatLog -Encoding utf8

# Single timeline of the bring-up service, ethercat.service and the EtherCAT
# kernel messages, sorted by time: shows whether the EtherCAT network or the
# controller reports a problem first. Built on the robot by
# fit4med_merged_journal (bash_scripts/fit4med_backup_common.sh).
. (Join-Path $PSScriptRoot "fmrr_robot_common.ps1")
$ErrorActionPreference = "Continue"  # the common file sets Stop; keep this script best-effort
$timelineLog = Join-Path $localDestination "fit4med_ethercat_timeline.log"
Write-Output "Build the merged fit4med + EtherCAT timeline in $timelineLog"
try {
    Send-FmrrLibrary $remoteHost
    ssh -T "${remoteUser}@${remoteHost}" "bash -c '. $FmrrRemoteLibrary && fit4med_merged_journal $unit'" `
        | Out-File -FilePath $timelineLog -Encoding utf8
}
catch {
    Write-Output "Merged timeline not available: $_"
}

# The robot logs use the robot clock (offline, may drift), the GUI logs this
# PC's clock: record both now, to line the two sets of logs up.
$clocksFile = Join-Path $localDestination "clocks.txt"
$robotNow = ssh -T "${remoteUser}@${remoteHost}" "date --iso-8601=ns"
@(
    "Clocks at log retrieval (compare to align robot and GUI logs):",
    "  robot   : $robotNow",
    "  this PC : $(Get-Date -Format o)"
) | Out-File -FilePath $clocksFile -Encoding utf8

Write-Output "Copy remote ROS log archives from $remoteTarget to $localDestination"
scp -r $remoteTarget $localDestination

# Local GUI logs (current session + rotated backups).
$guiLogDestination = Join-Path $localDestination "gui"
$guiLogs = @(foreach ($pattern in "pyqt_hang*.log", "gui_errors*.log", "gui_console*.log") {
    Get-ChildItem -Path $GuiLogDir -Filter $pattern -ErrorAction SilentlyContinue
})
if ($guiLogs.Count -gt 0) {
    if (!(Test-Path $guiLogDestination)) {
        New-Item -ItemType Directory -Path $guiLogDestination | Out-Null
    }
    Write-Output "Copy $($guiLogs.Count) GUI log(s) from $GuiLogDir to $guiLogDestination"
    $guiLogs | Copy-Item -Destination $guiLogDestination -Force
}
else {
    Write-Output "No GUI log found in $GuiLogDir"
}
