#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# stop_bag.sh — Stop the running ros2 bag recording cleanly.
#
# Strategy:
#   1. Try SIGINT by PID (from /tmp/ros2bag_gui_pid) — fast and targeted.
#   2. Fall back to pkill searching by command name — works even if PID
#      tracking was lost or the signal didn't reach child processes.
#
# SIGINT lets ros2 bag record finalize (flush) the bag before exiting.

PID_FILE=/tmp/ros2bag_gui_pid

sent=0

# --- 1. PID-based SIGINT ---
if [ -f "$PID_FILE" ]; then
    BAG_PID="$(cat "$PID_FILE")"
    rm -f "$PID_FILE"
    if kill -0 "$BAG_PID" 2>/dev/null; then
        kill -INT "$BAG_PID" 2>/dev/null
        echo "[stop_bag.sh] Sent SIGINT to ros2 bag record (PID=${BAG_PID})."
        sent=1
    else
        echo "[stop_bag.sh] PID=${BAG_PID} from PID file is no longer running."
    fi
fi

# --- 2. pkill fallback (catches any surviving ros2 bag record processes) ---
if pkill -INT -f "ros2 bag record" 2>/dev/null; then
    echo "[stop_bag.sh] pkill SIGINT sent to ros2 bag record processes."
    sent=1
fi

if [ "$sent" -eq 0 ]; then
    echo "[stop_bag.sh] No ros2 bag record process found — nothing to stop."
fi
