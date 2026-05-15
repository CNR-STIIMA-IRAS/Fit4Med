#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0


source /home/fit4med/fit4med_ws/install/setup.bash

perform_homing=false
while :; do
    case $1 in
        -h|--perform-homing)
            perform_homing=true
            ;;
        -h|--help)
            echo "Usage: $0 [--perform-homing]"
            exit 0
            ;;
        --) # End of all options
            shift
            break
            ;;
        -?*)
            echo "Unknown option: $1" >&2
            exit 1
            ;;
        *)  # No more options
            break
    esac
    shift
done

RESET_BINARY="/home/fit4med/fit4med_ws/build/reset_coe_faults/reset_coe_faults"
RESET_TIMEOUT=30

# Release EtherCAT master 1 if held by a previous process (e.g. leftover ros2 stack).
if command -v fuser >/dev/null 2>&1 && [ -e /dev/EtherCAT1 ]; then
    ETHERCAT1_PIDS=$(fuser /dev/EtherCAT1 2>/dev/null)
    if [ -n "$ETHERCAT1_PIDS" ]; then
        echo "*** EtherCAT master 1 held by PID(s) $ETHERCAT1_PIDS — sending SIGTERM ***" >&2
        kill -TERM $ETHERCAT1_PIDS 2>/dev/null || true
        sleep 2
        # Force-kill if still alive.
        ETHERCAT1_PIDS=$(fuser /dev/EtherCAT1 2>/dev/null)
        [ -n "$ETHERCAT1_PIDS" ] && kill -KILL $ETHERCAT1_PIDS 2>/dev/null || true
    fi
fi

echo "*** Resetting EtherCAT CoE drive faults (master 1) before launching ROS2 ***"
if [ ! -x "$RESET_BINARY" ]; then
    echo "WARNING: reset_coe_faults binary not found at $RESET_BINARY, skipping." >&2
else
    timeout "$RESET_TIMEOUT" "$RESET_BINARY"
    reset_exit=$?
    if [ $reset_exit -eq 0 ]; then
        echo "*** CoE fault reset completed: all drives healthy ***"
    elif [ $reset_exit -eq 124 ]; then
        echo "WARNING: reset_coe_faults timed out after ${RESET_TIMEOUT}s, proceeding anyway." >&2
    else
        echo "WARNING: reset_coe_faults exited with code $reset_exit, proceeding anyway." >&2
    fi
fi

echo "************************************************** Launching ROS2 environment with perform_homing set to: $perform_homing **************************************************"
ros2 launch tecnobody_workbench run_platform_control.launch.py perform_homing:=$perform_homing
