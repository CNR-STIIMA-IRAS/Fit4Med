#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0


source /home/fit4med/fit4med_ws/install/setup.bash

perform_homing=false
eeg_delay_ms="${EEG_DELAY_MS:-4000}"
while :; do
    case $1 in
        -z|--perform-homing)
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

if ! [[ "$eeg_delay_ms" =~ ^[0-9]+$ ]]; then
    echo "ERROR: EEG_DELAY_MS must be an integer number of milliseconds." >&2
    exit 1
fi

# Inside a bring-up session: logs of this start in their own numbered folder
# (see fit4med_session_log.sh).
source "$(dirname "$0")/fit4med_session_log.sh"
fit4med_log_begin_run platform

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

# Wait until every drive on master 1 has a responsive CoE mailbox before doing
# anything that talks to the slaves. Prevents the PREOP->SAFEOP startup race.
WAIT_READY_SCRIPT="$(dirname "$0")/ec_wait_ready.sh"
if [ -x "$WAIT_READY_SCRIPT" ]; then
    "$WAIT_READY_SCRIPT" || echo "WARNING: ec_wait_ready gate did not pass cleanly." >&2
else
    echo "WARNING: ec_wait_ready.sh not found/executable at $WAIT_READY_SCRIPT, skipping gate." >&2
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

# Optional diagnostic monitors, enabled in the environment of the bring-up
# (systemd unit or the shell that runs ros2 launch):
#   FIT4MED_EC_MONITOR=1   ec_link_monitor.sh: EtherCAT slaves, counters, drive alarms
#   FIT4MED_CPU_MONITOR=1  rt_load_monitor.sh: CPU load per CPU/process/thread
# Their logs go to the folder of this start (so into the session zip); they
# stop by themselves when this script exits.
start_monitor() {  # <script> <description>
    local script
    script="$(dirname "$0")/$1"
    if [ -x "$script" ]; then
        "$script" -P $$ -o "${FIT4MED_RUN_DIR:-$HOME/.ros/fit4med_diagnostics}" \
            > /dev/null 2>&1 < /dev/null &
        echo "*** $2 started (PID $!) ***"
    else
        echo "WARNING: $2 enabled but $script is not executable." >&2
    fi
}
[ "${FIT4MED_EC_MONITOR:-0}" = "1" ] && start_monitor ec_link_monitor.sh "EtherCAT link monitor"
[ "${FIT4MED_CPU_MONITOR:-0}" = "1" ] && start_monitor rt_load_monitor.sh "CPU load monitor"

echo "************************************************** Launching ROS2 environment with perform_homing set to: $perform_homing, eeg_delay_ms set to: $eeg_delay_ms **************************************************"
ros2 launch tecnobody_workbench run_platform_control.launch.py perform_homing:=$perform_homing eeg_delay_ms:=$eeg_delay_ms
