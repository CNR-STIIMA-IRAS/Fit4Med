# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

param(
    [ValidateSet("service", "legacy")]
    [string]$Mode = "service"
)

$remoteUser = "fit4med"
$remoteHost = "192.168.1.1"
$remoteScript = "/home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/kill_fmrr_apps.sh"

if($Mode -eq "service") {
  ssh -T "${remoteUser}@${remoteHost}" "systemctl --user stop 'fit4med-bringup@*.service'; bash -lc 'source /opt/ros/jazzy/setup.bash; $remoteScript; ros2 daemon stop'"
}
else {
  ssh -t "${remoteUser}@${remoteHost}" "bash -l -c '$remoteScript'"
}
