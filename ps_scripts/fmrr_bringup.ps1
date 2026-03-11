# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

# Define variables
$remoteUser = "fit4med"     
$remoteHost = "192.168.1.1"
$remoteScript = "source /home/fit4med/fit4med_ws/install/setup.bash; /home/fit4med/fit4med_ws/src/Fit4Med/bash_scrips/log.sh; clear; ros2 launch tecnobody_workbench run_sickPLC.launch.py gui_ip:=192.168.1.2"

# Execute the Python script remotely
ssh -t "${remoteUser}@${remoteHost}" "bash -l -c '$remoteScript'"

