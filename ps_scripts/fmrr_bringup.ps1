# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

# IP of the PC running the GUI: the platform PC by default, override with
#   .\fmrr_bringup.ps1 -GuiIp <this_pc_static_ip>
param(
    [string]$GuiIp = "192.168.1.2",
    [ValidateSet("service", "legacy")]
    [string]$Mode = "service"
)

# Define variables
$remoteUser = "fit4med"     
$remoteHost = "192.168.1.1"

if ($Mode -eq "service") {
    $unit = "fit4med-bringup@$GuiIp.service"
    ssh -T "${remoteUser}@${remoteHost}" "systemctl --user stop 'fit4med-bringup@*.service'; systemctl --user start $unit; systemctl --user --no-pager status $unit"
}
else {
    $remoteScript = "source /home/fit4med/fit4med_ws/install/setup.bash; /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./log.sh; ros2 launch tecnobody_workbench run_sickPLC.launch.py gui_ip:=$GuiIp"
    ssh -t "${remoteUser}@${remoteHost}" "bash -l -i -c '$remoteScript'"
}

