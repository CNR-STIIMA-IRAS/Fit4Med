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
