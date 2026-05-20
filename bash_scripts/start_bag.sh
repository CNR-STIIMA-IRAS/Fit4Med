#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# start_bag.sh — Start a ros2 bag recording for rehab data.
#
# Saves bags under <repo>/rehab_gui/Data/bag_<timestamp>/.
# Writes the recording PID to /tmp/ros2bag_gui_pid so stop_bag.sh can
# send a clean SIGINT to finalize the bag.
# This script blocks until the bag process exits (either stopped by
# stop_bag.sh or killed externally).

# ---------------------------------------------------------------------------
# 1. Source ROS 2 environment only if ros2 is not already in PATH
#    (when launched from the ROS2 node, the environment is already set up)
# ---------------------------------------------------------------------------
if ! command -v ros2 &>/dev/null; then
    ROS_SOURCED=0
    for distro in jazzy humble iron rolling; do
        setup_file="/opt/ros/${distro}/setup.bash"
        if [ -f "$setup_file" ]; then
            # shellcheck disable=SC1090
            source "$setup_file" || true
            ROS_SOURCED=1
            break
        fi
    done

    if [ "$ROS_SOURCED" -eq 0 ]; then
        echo "[start_bag.sh] ERROR: ros2 not found and no ROS 2 setup.bash under /opt/ros/" >&2
        exit 1
    fi

    # Source workspace overlay
    WS_SETUP=~/fit4med_ws/install/setup.bash
    if [ -f "$WS_SETUP" ]; then
        # shellcheck disable=SC1090
        source "$WS_SETUP" || true
    fi
fi

# ---------------------------------------------------------------------------
# 2. Prepare output directory
# ---------------------------------------------------------------------------
DATA_DIR="${HOME}/Data"
mkdir -p "$DATA_DIR"

TIMESTAMP="$(date +%Y-%m-%d_%H-%M-%S)"
OUTPUT_DIR="${DATA_DIR}/bag_${TIMESTAMP}"

# ---------------------------------------------------------------------------
# 3. Start recording
# ---------------------------------------------------------------------------
# Remove any stale PID file from a previous crashed session
rm -f /tmp/ros2bag_gui_pid

TOPICS="/joint_states /ft_sensor_command_broadcaster/wrench"

# Launch in background so we can save its PID, then wait for it.
# shellcheck disable=SC2086
ros2 bag record -o "$OUTPUT_DIR" $TOPICS &
BAG_PID=$!
echo "$BAG_PID" > /tmp/ros2bag_gui_pid

echo "[start_bag.sh] Recording started (PID=${BAG_PID}) -> ${OUTPUT_DIR}"

# Block until the bag process exits (stop_bag.sh sends SIGINT to it)
wait "$BAG_PID" || true

rm -f /tmp/ros2bag_gui_pid
echo "[start_bag.sh] Recording stopped."
