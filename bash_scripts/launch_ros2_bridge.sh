#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0


source /home/fit4med/fit4med_ws/install/setup.bash

# Inside a bring-up session: logs of this start in their own numbered folder
# (see fit4med_session_log.sh).
source "$(dirname "$0")/fit4med_session_log.sh"
fit4med_log_begin_run rosbridge

echo "************************************************** Launching ROSBRIDGE **************************************************"
ros2 launch tecnobody_workbench run_rosbridge.launch.py
