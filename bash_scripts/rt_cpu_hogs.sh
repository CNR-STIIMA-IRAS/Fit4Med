#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Find what takes the CPU away from the EtherCAT control loop, while the real
# system runs. In one time window it collects:
#   config.txt     real-time setup: PREEMPT_RT, isolcpus, governor, irqbalance,
#                  rtprio/memlock limits, scheduling of the ros2_control_node
#                  threads and of the network IRQ threads
#   rtla_timerlat.txt  (if rtla is available) per-CPU latency and, at the first
#                  wake-up later than -a us, the kernel analysis of who blocked
#                  it (IRQ, softirq, thread name and PID); timerlat_trace.txt
#   pidstat_cpu.txt, pidstat_cswch.txt  CPU use and context switches per thread
#   mpstat.txt     load per CPU, including %irq and %soft
#   ethercat_start.txt, ethercat_end.txt  master statistics (lost frames)
#   kernel.txt     kernel warnings and EtherCAT messages of the window
#   ros_overruns.txt  controller_manager overrun / RT setup messages
#   summary.txt    the essentials of all the above
#
# Usage:
#   rt_cpu_hogs.sh [-d SECONDS] [-a THRESHOLD_US] [-p PRIO] [-o OUT_DIR]
#     -d  window length in seconds (default 600)
#     -a  rtla stops and analyses at the first latency above this (default 1000)
#     -p  SCHED_FIFO priority of the rtla measuring threads (default 50)
#     -o  output folder (default ~/.ros/fit4med_diagnostics)
#
# Needs: sudo; sysstat (pidstat, mpstat); rtla (optional, strongly suggested).
# Run it as fit4med, not with sudo: it calls sudo itself where needed.

set -uo pipefail

DURATION=600
THRESHOLD=1000
PRIO=50
OUT_ROOT="${FIT4MED_DIAG_ROOT:-$HOME/.ros/fit4med_diagnostics}"
FIT4MED_LOG_ROOT="${FIT4MED_LOG_ROOT:-$HOME/.ros/fit4med_log}"
STEP=5  # pidstat/mpstat sampling period [s]

while getopts "d:a:p:o:h" opt; do
  case "$opt" in
    d) DURATION="$OPTARG" ;;
    a) THRESHOLD="$OPTARG" ;;
    p) PRIO="$OPTARG" ;;
    o) OUT_ROOT="$OPTARG" ;;
    *) sed -n 's/^#   rt_cpu_hogs.sh/  rt_cpu_hogs.sh/p; s/^#     /    /p' "$0"; exit 1 ;;
  esac
done
[[ "$DURATION" =~ ^[0-9]+$ && "$DURATION" -ge "$STEP" ]] || { echo "[ERROR] -d: seconds, at least $STEP" >&2; exit 1; }

export LC_ALL=C  # sysstat: "Average:" and decimal points, whatever the robot locale

have() { command -v "$1" > /dev/null 2>&1; }

OUT="$OUT_ROOT/cpu_hogs_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT" || exit 1
sudo -v || exit 1  # ask the password now, not in the middle of the window

# ---------------------------------------------------------------------------
# Real-time setup
# ---------------------------------------------------------------------------

# ethercat_masters: indices of the EtherCAT masters (/dev/EtherCAT<N>).
ethercat_masters() {
  local d
  for d in /dev/EtherCAT[0-9]*; do [[ -e "$d" ]] && echo "${d#/dev/EtherCAT}"; done
}

ethercat_snapshot() {
  local m
  have ethercat || { echo "ethercat command not available"; return; }
  for m in $(ethercat_masters); do
    echo "===== ethercat master -m $m ====="
    timeout 5 ethercat master -m "$m"
  done
}

# ethercat_lost <file>: "master lost_frames" per master in an ethercat_snapshot.
ethercat_lost() {
  awk '/^===== ethercat master -m/ { m = $5 } /Lost frames:/ && m != "" { print m, $3; m = "" }' "$1"
}

# ethercat_ifaces: network interfaces used by the EtherCAT masters, from the
# MAC addresses in /etc/ethercat.conf (MASTERn_DEVICE).
ethercat_ifaces() {
  local mac
  sed -n 's/^MASTER[0-9]*_DEVICE="\{0,1\}\([0-9A-Fa-f:]\{17\}\).*/\1/p' /etc/ethercat.conf 2> /dev/null \
    | while read -r mac; do
        ip -o link 2> /dev/null | awk -v mac="${mac,,}" 'tolower($0) ~ mac { sub(/:$/, "", $2); print $2 }'
      done
}

START_EPOCH=$(date +%s)
WARN=()
{
  echo "DATE=$(date --iso-8601=seconds)  HOST=$(hostname)"
  echo "WINDOW=${DURATION}s  RTLA_THRESHOLD=${THRESHOLD}us  RTLA_PRIO=FIFO:$PRIO"
  echo "KERNEL=$(uname -r)  $(uname -v)"
  echo "PREEMPT_RT=$(cat /sys/kernel/realtime 2> /dev/null || echo 0)"
  echo "CMDLINE=$(cat /proc/cmdline)"
  echo "CPUS=$(nproc)  ISOLATED=$(cat /sys/devices/system/cpu/isolated 2> /dev/null)"
  echo "GOVERNORS=$(cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor 2> /dev/null | sort | uniq -c | xargs)"
  echo "IDLE_DRIVER=$(cat /sys/devices/system/cpu/cpuidle/current_driver 2> /dev/null)" \
       "C-STATES=$(cat /sys/devices/system/cpu/cpu0/cpuidle/state*/name 2> /dev/null | xargs)"
  echo "IRQBALANCE=$(systemctl is-active irqbalance 2> /dev/null)"
  echo "LIMITS($(id -un)): rtprio=$(ulimit -r) memlock=$(ulimit -l)"
  echo "ETHERCAT_CONF: $(grep -E '^(MASTER[0-9]*_DEVICE|DEVICE_MODULES)=' /etc/ethercat.conf 2> /dev/null | xargs)"
  echo
  echo "===== ros2_control_node threads (CLS FF = SCHED_FIFO, PSR = CPU it runs on) ====="
  for pid in $(pgrep -f '/ros2_control_node( |$)'); do
    echo "--- PID $pid: $(tr '\0' ' ' < "/proc/$pid/cmdline" | cut -c1-200)"
    echo "    affinity: $(taskset -cp "$pid" 2> /dev/null | sed 's/.*: //')"
    ps -L -o tid,cls,rtprio,ni,psr,pcpu,comm -p "$pid"
  done
  echo
  echo "===== IRQ threads of the EtherCAT interfaces ($(ethercat_ifaces | xargs)) ====="
  ifaces="$(ethercat_ifaces | paste -sd'|')"
  ps -eLo pid,cls,rtprio,psr,pcpu,comm | awk -v re="irq/[0-9]+-(${ifaces:-ecm|eth|enp|eno})" 'NR == 1 || $NF ~ re'
  echo
  echo "===== all real-time threads ====="
  ps -eLo pid,tid,cls,rtprio,psr,pcpu,comm | awk 'NR == 1 || ($3 != "TS" && $3 != "IDL" && $3 != "B")' \
    | { IFS= read -r header; echo "$header"; sort -k4,4nr; }
} > "$OUT/config.txt" 2>&1

grep -q '^PREEMPT_RT=1' "$OUT/config.txt" || WARN+=("kernel is not PREEMPT_RT")
grep -q '^GOVERNORS=.*\(powersave\|schedutil\|ondemand\)' "$OUT/config.txt" && WARN+=("CPU governor is not 'performance' on every CPU")
grep -q '^IRQBALANCE=active' "$OUT/config.txt" && WARN+=("irqbalance is active: it can move the EtherCAT NIC interrupt")
[[ "$(ulimit -r)" == "0" ]] && WARN+=("rtprio limit of $(id -un) is 0: ros2_control_node cannot get SCHED_FIFO")
for pid in $(pgrep -f '/ros2_control_node( |$)'); do
  ps -L -o cls= -p "$pid" | grep -qw FF \
    || WARN+=("ros2_control_node PID $pid has no SCHED_FIFO thread: its loop runs with the normal scheduler")
done
pgrep -f '/ros2_control_node( |$)' > /dev/null || WARN+=("no ros2_control_node running: is the robot on?")

ethercat_snapshot > "$OUT/ethercat_start.txt" 2>&1

# ---------------------------------------------------------------------------
# Measurement window
# ---------------------------------------------------------------------------

pids=()
cleanup() {
  # SIGINT, not SIGTERM: rtla must remove its trace instance, or the timerlat
  # threads keep running.
  sudo pkill -INT -x rtla 2> /dev/null
  kill "${pids[@]}" 2> /dev/null
  wait
  sudo chown -R "$(id -u):$(id -g)" "$OUT" 2> /dev/null
  echo "[fit4med] Interrupted: raw data (no summary) in $OUT"
  exit 130
}
trap cleanup INT TERM

samples=$(( DURATION / STEP ))
if have pidstat && have mpstat; then
  pidstat -u -t "$STEP" "$samples" > "$OUT/pidstat_cpu.txt" 2>&1 & pids+=($!)
  pidstat -w -t "$STEP" "$samples" > "$OUT/pidstat_cswch.txt" 2>&1 & pids+=($!)
  mpstat -P ALL "$STEP" "$samples" > "$OUT/mpstat.txt" 2>&1 & pids+=($!)
else
  WARN+=("sysstat not installed (sudo apt install sysstat): no per-thread CPU data")
fi

RTLA=0
if have rtla; then
  RTLA=1
  # -a: stop at the first latency above THRESHOLD, save timerlat_trace.txt in
  # the current folder and print the analysis of the blocking context.
  (cd "$OUT" && exec sudo rtla timerlat top -q -a "$THRESHOLD" -P "f:$PRIO" -d "${DURATION}s") \
    > "$OUT/rtla_timerlat.txt" 2>&1 &
else
  WARN+=("rtla/timerlat not available (sudo apt install rtla, or linux-tools-$(uname -r)): no 'who blocked the CPU' analysis")
fi

echo "[fit4med] Collecting for ${DURATION}s. Use the robot as usual. Results in $OUT"
wait
trap - INT TERM
sudo chown -R "$(id -u):$(id -g)" "$OUT" 2> /dev/null

ethercat_snapshot > "$OUT/ethercat_end.txt" 2>&1
END_EPOCH=$(( $(date +%s) + 1 ))

{
  echo "===== kernel warnings and errors ====="
  journalctl -k --since "@$START_EPOCH" --until "@$END_EPOCH" -p warning -o short-precise --no-pager
  echo; echo "===== kernel EtherCAT messages ====="
  journalctl -k --since "@$START_EPOCH" --until "@$END_EPOCH" -o short-precise --no-pager | grep -i ethercat
} > "$OUT/kernel.txt" 2>&1

# Overruns of the controller managers: in the user journal (sickPLC, under
# systemd) and in the console.log/ROS logs of the running session.
OVERRUN_RE='overrun|missed its desired rate|real-time kernel|FIFO RT|thread priority'
{
  journalctl --user --since "@$START_EPOCH" --until "@$END_EPOCH" -o short-precise --no-pager 2> /dev/null \
    | grep -iE "$OVERRUN_RE"
  find "$FIT4MED_LOG_ROOT" -path "$FIT4MED_LOG_ROOT/run_*" -type f -newermt "@$START_EPOCH" \
       \( -name '*.log' -o -name '*.txt' \) -print0 2> /dev/null \
    | xargs -0 -r grep -iEH "$OVERRUN_RE"
} > "$OUT/ros_overruns.txt" 2>&1

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------

# sysstat_avg <file> <sort column> <n> <columns...>: the "Average:" rows of a
# sysstat report, sorted by a column (by name), with only the given columns.
sysstat_avg() {
  local file="$1" key="$2" n="$3"
  shift 3
  awk -v key="$key" -v cols="$*" '
    BEGIN { nc = split(cols, C, " ") }
    /^Average:/ && /Command|%idle/ { for (i = 1; i <= NF; i++) H[$i] = i; hdr = 1; next }
    /^Average:/ && hdr {
      line = ""
      for (k = 1; k <= nc; k++) line = line sprintf("%-12s ", $(H[C[k]]))
      print $(H[key]) "\t" line
    }' "$file" | sort -t $'\t' -k1,1gr | head -n "$n" | cut -f2-
}

{
  echo "rt_cpu_hogs: ${DURATION}s window from $(date -d "@$START_EPOCH" --iso-8601=seconds), $(hostname)"
  echo
  echo "== Real-time setup =="
  if (( ${#WARN[@]} )); then printf '  WARNING: %s\n' "${WARN[@]}"; else echo "  no problems found"; fi

  if (( RTLA )); then
    echo
    echo "== rtla timerlat (threshold ${THRESHOLD}us) =="
    if grep -q 'hit stop tracing' "$OUT/rtla_timerlat.txt"; then
      echo "  A wake-up was later than ${THRESHOLD}us. Analysis (who blocked the CPU):"
      sed -n '/hit stop tracing/,$p' "$OUT/rtla_timerlat.txt" | sed 's/^/  /' | head -n 40
      echo "  Full trace: timerlat_trace.txt"
    else
      echo "  No latency above ${THRESHOLD}us (or rtla failed: see the output below):"
      sed 's/^/  /' "$OUT/rtla_timerlat.txt" | tail -n 25
    fi
  fi

  if [[ -s "$OUT/pidstat_cpu.txt" ]]; then
    echo
    echo "== Top threads by CPU (average over the window; |__ = thread of the TGID above) =="
    printf '  %-12s %-12s %-12s %-12s %s\n' TGID TID %CPU CPU Command
    sysstat_avg "$OUT/pidstat_cpu.txt" %CPU 20 TGID TID %CPU CPU Command | sed 's/^/  /'
    echo
    echo "== Top threads by involuntary context switches (preempted by someone else) =="
    printf '  %-12s %-12s %-12s %-12s %s\n' TGID TID nvcswch/s cswch/s Command
    sysstat_avg "$OUT/pidstat_cswch.txt" nvcswch/s 15 TGID TID nvcswch/s cswch/s Command | sed 's/^/  /'
    echo
    echo "== Load per CPU (average) =="
    printf '  %-12s %-12s %-12s %-12s %-12s %s\n' CPU %usr %sys %irq %soft %idle
    sysstat_avg "$OUT/mpstat.txt" CPU 99 CPU %usr %sys %irq %soft %idle | sort -k1,1V | sed 's/^/  /'
  fi

  echo
  echo "== EtherCAT lost frames in the window =="
  join <(ethercat_lost "$OUT/ethercat_start.txt" | sort) <(ethercat_lost "$OUT/ethercat_end.txt" | sort) \
    | awk '{ printf "  master %s: %d lost frames (%s -> %s)\n", $1, $3 - $2, $2, $3; n++ }
           END { if (!n) print "  not available (see ethercat_start.txt)" }'

  echo
  echo "== Kernel warnings / EtherCAT messages: $(grep -vc -e '^=====' -e '^-- ' -e '^$' "$OUT/kernel.txt") lines (kernel.txt) =="
  grep -v -e '^=====' -e '^-- ' -e '^$' "$OUT/kernel.txt" | tail -n 15 | sed 's/^/  /'

  echo
  echo "== controller_manager overrun / RT messages: $(wc -l < "$OUT/ros_overruns.txt") (ros_overruns.txt) =="
  tail -n 10 "$OUT/ros_overruns.txt" | sed 's/^/  /'
} > "$OUT/summary.txt"

cat "$OUT/summary.txt"
echo
echo "[fit4med] All files in $OUT"
