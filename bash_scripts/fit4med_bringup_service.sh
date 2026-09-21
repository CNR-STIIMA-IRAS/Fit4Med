#!/usr/bin/env bash
set -Eeuo pipefail

GUI_IP="${1:-192.168.1.2}"

source /opt/ros/jazzy/setup.bash
source /home/fit4med/fit4med_ws/install/setup.bash

/home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/log.sh || true

exec ros2 launch tecnobody_workbench run_sickPLC.launch.py gui_ip:="${GUI_IP}"