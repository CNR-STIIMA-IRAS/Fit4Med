# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

# IP of the PC running the GUI: the platform PC by default, override with
#   .\fmrr_bringup.ps1 -GuiIp <this_pc_static_ip>
param([string]$GuiIp = "192.168.1.2")

# Define variables
$remoteUser = "fit4med"     
$remoteHost = "192.168.1.1"
$remoteScript = "source /home/fit4med/fit4med_ws/install/setup.bash; /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./log.sh; ros2 launch tecnobody_workbench run_sickPLC.launch.py gui_ip:=$GuiIp"

# Execute the Python script remotely
ssh -t "${remoteUser}@${remoteHost}" "bash -l -i -c '$remoteScript'"

