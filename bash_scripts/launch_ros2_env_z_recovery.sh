#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Minimal launch for Z-axis limit-switch recovery.
# Starts only the infrastructure + forward_velocity_controller + filter_commands_node
# so the operator can jog Z+ out of the limit without loading the full platform stack.

source /home/fit4med/fit4med_ws/install/setup.bash

RESET_BINARY="/home/fit4med/fit4med_ws/build/reset_coe_faults/reset_coe_faults"
RESET_TIMEOUT=30

# Release EtherCAT master 1 if held by a previous process
if command -v fuser >/dev/null 2>&1 && [ -e /dev/EtherCAT1 ]; then
    ETHERCAT1_PIDS=$(fuser /dev/EtherCAT1 2>/dev/null)
    if [ -n "$ETHERCAT1_PIDS" ]; then
        echo "*** EtherCAT master 1 held by PID(s) $ETHERCAT1_PIDS — sending SIGTERM ***" >&2
        kill -TERM $ETHERCAT1_PIDS 2>/dev/null || true
        sleep 2
        ETHERCAT1_PIDS=$(fuser /dev/EtherCAT1 2>/dev/null)
        [ -n "$ETHERCAT1_PIDS" ] && kill -KILL $ETHERCAT1_PIDS 2>/dev/null || true
    fi
fi

WAIT_READY_SCRIPT="$(dirname "$0")/ec_wait_ready.sh"
if [ -x "$WAIT_READY_SCRIPT" ]; then
    "$WAIT_READY_SCRIPT" || echo "WARNING: ec_wait_ready gate did not pass cleanly." >&2
else
    echo "WARNING: ec_wait_ready.sh not found/executable at $WAIT_READY_SCRIPT, skipping gate." >&2
fi

echo "*** Launching Z-axis recovery environment (minimal stack, forward_velocity_controller only) ***"
ros2 launch tecnobody_workbench run_z_recovery_control.launch.py \
    auto_recover:=${AUTO_RECOVER:-false}
