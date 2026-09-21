# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

param([string]$GuiIp = "192.168.1.2")

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

Write-Output "Copy remote ROS log archives from $remoteTarget to $localDestination"
scp -r $remoteTarget $localDestination
