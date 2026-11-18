#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Long-running, unattended variant of rt_cpu_hogs.sh: record the CPU load of
# the robot PC for the whole life of a platform start, into the log folder of
# that start (it ends up in the session zip that fmrr_retrieve_logs.ps1
# copies). launch_ros2_env.sh starts it when FIT4MED_CPU_MONITOR=1.
#
# Unlike rt_cpu_hogs.sh: no sudo, no rtla, no ros2 command (a ros2 CLI creates
# a DDS participant, which can delay the Python nodes), slow sampling to keep
# the files small (~2 MB per hour):
#   config.txt          at start: kernel, isolated CPUs, limits, RT threads
#   mpstat.txt          load per CPU, every MPSTAT_S s (default 10)
#   pidstat_proc.txt    CPU per process (active ones), every PROC_S s (30)
#   pidstat_threads.txt CPU and %wait per thread (active ones), every THREAD_S s (60)
#   rt_threads.txt      SCHED_FIFO/RR threads and the CPU they run on, a line
#                       only when that changes (checked every THREAD_S s)
#   sensors.txt         temperatures of the PC (lm-sensors), every THREAD_S s
#   clock_jumps.txt     steps of the wall clock against the monotonic clock
#                       (manual time setting, RTC restore), checked every 5 s
#
# Usage: rt_load_monitor.sh [-o OUT_DIR] [-P PID] [-c MPSTAT_S] [-p PROC_S] [-t THREAD_S]
#   -o  output folder (default ~/.ros/fit4med_diagnostics/rt_load_<date>)
#   -P  stop when this process exits (launch_ros2_env.sh passes its own PID)
# Stop it with Ctrl+C otherwise. Needs sysstat (pidstat, mpstat).

set -u

OUT=""
PARENT_PID=""
MPSTAT_S=10
PROC_S=30
THREAD_S=60

while getopts "o:P:c:p:t:h" opt; do
  case "$opt" in
    o) OUT="$OPTARG" ;;
    P) PARENT_PID="$OPTARG" ;;
    c) MPSTAT_S="$OPTARG" ;;
    p) PROC_S="$OPTARG" ;;
    t) THREAD_S="$OPTARG" ;;
    *) sed -n '2,/^set -u/p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
  esac
done

export LC_ALL=C  # sysstat: decimal points and English headers, whatever the locale

[[ -n "$OUT" ]] || OUT="${FIT4MED_DIAG_ROOT:-$HOME/.ros/fit4med_diagnostics}/rt_load_$(date +%Y%m%d-%H%M%S)"
OUT="$OUT/rt_load"
mkdir -p "$OUT" || exit 1

# Lower priority: this must never compete with the bring-up.
renice -n 10 -p $$ > /dev/null 2>&1 || true

have() { command -v "$1" > /dev/null 2>&1; }

rt_threads() {
  ps -eLo tid,cls,rtprio,psr,comm --no-headers \
    | awk '$2 == "FF" || $2 == "RR"' \
    | awk '$5 !~ /^(migration|idle_inject|irq_work|rcuc|rcuog|rcuop|rcub|ktimers|rcu_preempt|psimon|watchdogd)/' \
    | sort -k5,5 -k1,1n
}

{
  echo "DATE=$(date --iso-8601=seconds)  HOST=$(hostname)"
  echo "KERNEL=$(uname -r)  $(uname -v)"
  echo "PREEMPT_RT=$(cat /sys/kernel/realtime 2> /dev/null || echo 0)"
  echo "CMDLINE=$(cat /proc/cmdline)"
  echo "CPUS=$(nproc --all)  ONLINE=$(cat /sys/devices/system/cpu/online)  ISOLATED=$(cat /sys/devices/system/cpu/isolated 2> /dev/null)"
  echo "LIMITS($(id -un)): rtprio=$(ulimit -r) memlock=$(ulimit -l)"
  echo "SAMPLING: mpstat ${MPSTAT_S}s, pidstat processes ${PROC_S}s, pidstat threads ${THREAD_S}s"
  echo
  echo "===== ros2_control_node processes ====="
  for pid in $(pgrep -f '/ros2_control_node( |$)'); do
    echo "--- PID $pid affinity $(taskset -cp "$pid" 2> /dev/null | sed 's/.*: //'): $(tr '\0' ' ' < "/proc/$pid/cmdline" | cut -c1-160)"
  done
  echo
  echo "===== real-time threads (kernel housekeeping threads omitted) ====="
  echo "  TID CLS RTPRIO PSR COMMAND"
  rt_threads
} > "$OUT/config.txt" 2>&1

pids=()
stop() {
  kill "${pids[@]}" 2> /dev/null
  wait 2> /dev/null
  echo "# stopped $(date --iso-8601=seconds) ($1)" >> "$OUT/config.txt"
  exit 0
}
trap 'stop interrupted' INT TERM

if have mpstat && have pidstat; then
  mpstat -P ALL "$MPSTAT_S" > "$OUT/mpstat.txt" 2>&1 & pids+=($!)
  # -h: one line per task and sample, with the time: easy to grep and plot.
  pidstat -u -h "$PROC_S" > "$OUT/pidstat_proc.txt" 2>&1 & pids+=($!)
  pidstat -u -t -h "$THREAD_S" > "$OUT/pidstat_threads.txt" 2>&1 & pids+=($!)
else
  echo "WARNING: sysstat not installed (sudo apt install sysstat): only rt_threads.txt is recorded" >> "$OUT/config.txt"
fi

prev="(not read yet)"
elapsed=0

# Wall clock minus time since boot: constant unless the wall clock is stepped.
clock_offset() {
  awk -v now="$(date +%s.%N)" '{printf "%.3f", now - $1}' /proc/uptime
}
prev_offset="$(clock_offset)"
echo "# wall clock - uptime offset at start: $prev_offset s; a line per step > 0.5 s" > "$OUT/clock_jumps.txt"
while true; do
  if [[ -n "$PARENT_PID" ]] && ! kill -0 "$PARENT_PID" 2> /dev/null; then
    stop "parent $PARENT_PID gone"
  fi
  offset="$(clock_offset)"
  if awk -v a="$offset" -v b="$prev_offset" 'BEGIN { d = a - b; exit !(d > 0.5 || d < -0.5) }'; then
    echo "$(date --iso-8601=seconds) wall clock stepped by $(awk -v a="$offset" -v b="$prev_offset" 'BEGIN { printf "%+.3f", a - b }') s" >> "$OUT/clock_jumps.txt"
  fi
  prev_offset="$offset"
  if (( elapsed % THREAD_S == 0 )); then
    if have sensors; then
      # Under LC_ALL=C sensors prints " C" instead of "°C": keep every line.
      { echo "===== $(date +%H:%M:%S)"; LC_ALL=C.UTF-8 sensors 2> /dev/null | grep -v '^$' ; } >> "$OUT/sensors.txt"
    fi
    cur="$(rt_threads)"
    if [[ "$cur" != "$prev" ]]; then
      { echo "===== $(date +%H:%M:%S) real-time threads: TID CLS RTPRIO PSR COMMAND"; echo "$cur"; } >> "$OUT/rt_threads.txt"
      prev="$cur"
    fi
  fi
  sleep 5
  elapsed=$(( elapsed + 5 ))
done
