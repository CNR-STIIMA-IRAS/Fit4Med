#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0


source /home/fit4med/fit4med_ws/install/setup.bash

echo "************************************************** Launching ROSBRIDGE **************************************************"
ros2 launch tecnobody_workbench run_rosbridge.launch.py
