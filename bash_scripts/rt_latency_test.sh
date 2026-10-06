#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Measure the scheduling latency of the robot PC with cyclictest, while the
# real system runs (bring-up, GUI, recordings...): how late a real-time thread
# that wants to run every INTERVAL us actually wakes up, on every CPU. This is
# what the EtherCAT control loop (ros2_control_node, 250/500 Hz) suffers.
#
# Usage:
#   rt_latency_test.sh [-d DURATION] [-p PRIO] [-i INTERVAL_US] [-o OUT_DIR]
#     -d  test duration, cyclictest syntax (default 10m; e.g. 30s, 1h)
#     -p  SCHED_FIFO priority of the measuring threads (default 50, the same
#         as the controller_manager thread_priority default)
#     -i  wake-up interval in us (default 1000)
#     -o  output folder (default ~/.ros/fit4med_diagnostics)
#
# Needs: cyclictest (package rt-tests) and sudo.
# Results: <OUT_DIR>/latency_YYYYMMDD-HHMMSS/{summary.txt,cyclictest.txt,system.txt}
#
# Reading the summary: with a 4 ms (250 Hz) or 2 ms (500 Hz) control period,
# max latencies of a few hundred us are a warning, above 1000 us the loop is
# losing cycles. Run it once with the robot idle and once in the heaviest
# condition (therapy running, GUI connected, bag recording) and compare.

set -uo pipefail

DURATION="10m"
PRIO=50
INTERVAL=1000
OUT_ROOT="${FIT4MED_DIAG_ROOT:-$HOME/.ros/fit4med_diagnostics}"
# Thresholds (us) for the "samples late by more than" counters (<= HIST_MAX).
THRESHOLDS=(250 500 1000 2000 4000)
HIST_MAX=4000  # histogram buckets 0..HIST_MAX-1 us; above: overflows

while getopts "d:p:i:o:h" opt; do
  case "$opt" in
    d) DURATION="$OPTARG" ;;
    p) PRIO="$OPTARG" ;;
    i) INTERVAL="$OPTARG" ;;
    o) OUT_ROOT="$OPTARG" ;;
    *) sed -n 's/^#   rt_latency_test.sh/  rt_latency_test.sh/p; s/^#     /    /p' "$0"; exit 1 ;;
  esac
done

if ! command -v cyclictest > /dev/null 2>&1; then
  echo "[ERROR] cyclictest not found: sudo apt install rt-tests" >&2
  echo "        (robot offline: download rt-tests .deb on a PC with internet and copy it)" >&2
  exit 1
fi

OUT="$OUT_ROOT/latency_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT" || exit 1

{
  echo "DATE=$(date --iso-8601=seconds)"
  echo "HOST=$(hostname)"
  echo "KERNEL=$(uname -a)"
  echo "PREEMPT_RT=$(cat /sys/kernel/realtime 2> /dev/null || echo 0)"
  echo "CMDLINE=$(cat /proc/cmdline)"
  echo "CPUS=$(nproc)"
  echo "GOVERNORS=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2> /dev/null | sort | uniq -c | xargs)"
  echo "TEST: duration=$DURATION prio=FIFO:$PRIO interval=${INTERVAL}us"
  echo
  echo "===== processes running during the test (top CPU) ====="
  ps -eo pid,cls,rtprio,ni,pcpu,comm --sort=-pcpu | head -n 25
} > "$OUT/system.txt"

echo "[fit4med] cyclictest for $DURATION on all CPUs (FIFO:$PRIO, ${INTERVAL}us). Leave the system working as usual."
echo "[fit4med] Results in $OUT"
# -m lock memory, -S one thread per CPU pinned to it, -q print only the
# final histogram/summary.
sudo cyclictest -m -S -p "$PRIO" -i "$INTERVAL" -D "$DURATION" -h "$HIST_MAX" -q \
  > "$OUT/cyclictest.txt" 2>&1
rc=$?
if (( rc != 0 )) || ! grep -q '^# Max Latencies' "$OUT/cyclictest.txt"; then
  echo "[ERROR] cyclictest failed (exit $rc), see $OUT/cyclictest.txt" >&2
  exit 1
fi

# Summary per CPU: min/avg/max and how many wake-ups were late by more than
# each threshold (histogram buckets >= threshold + overflows).
awk -v thr="${THRESHOLDS[*]}" -v ival="$INTERVAL" '
  BEGIN { nt = split(thr, T, " ") }
  /^[0-9]+[ \t]/ {
    for (c = 2; c <= NF; c++) {
      ncpu = (c - 1 > ncpu ? c - 1 : ncpu)
      for (k = 1; k <= nt; k++) if ($1 + 0 >= T[k]) late[c - 1, k] += $c
    }
    next
  }
  /^# Total:/               { for (c = 3; c <= NF; c++) tot[c - 2] = $c + 0 }
  /^# Min Latencies:/       { for (c = 4; c <= NF; c++) mn[c - 3]  = $c + 0 }
  /^# Avg Latencies:/       { for (c = 4; c <= NF; c++) av[c - 3]  = $c + 0 }
  /^# Max Latencies:/       { for (c = 4; c <= NF; c++) { mx[c - 3] = $c + 0; if ($c + 0 > gmax) gmax = $c + 0 } }
  /^# Histogram Overflows:/ { for (c = 4; c <= NF; c++) ov[c - 3]  = $c + 0 }
  END {
    printf "%-4s %10s %6s %6s %7s", "CPU", "samples", "min", "avg", "max"
    for (k = 1; k <= nt; k++) printf " %9s", ">" T[k] "us"
    printf "\n"
    for (i = 1; i <= ncpu; i++) {
      printf "%-4d %10d %6d %6d %7d", i - 1, tot[i], mn[i], av[i], mx[i]
      for (k = 1; k <= nt; k++) printf " %9d", late[i, k] + ov[i]  # every T <= hmax
      printf "\n"
    }
    printf "\nWorst latency: %d us (interval %d us). ", gmax, ival
    if (gmax < 250)       print "OK."
    else if (gmax < 1000) print "WARNING: noticeable jitter for a 2-4 ms EtherCAT cycle."
    else                  print "PROBLEM: the control loop can miss cycles. Run rt_cpu_hogs.sh to find the cause."
  }
' "$OUT/cyclictest.txt" | tee "$OUT/summary.txt"

grep -q '^PREEMPT_RT=1' "$OUT/system.txt" \
  || echo "NOTE: the kernel is not PREEMPT_RT: large latencies are expected." | tee -a "$OUT/summary.txt"
