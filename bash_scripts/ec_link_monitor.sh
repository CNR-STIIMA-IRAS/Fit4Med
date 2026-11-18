#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Watch the EtherCAT link and error counters of the platform bus (master 1)
# while the robot runs, to tell what happens when a slave drops off the bus
# (runs 0040-0048: the joint_z drive, slave 2, vanished and came back 18.2 s
# later).
#
# Every PERIOD seconds it reads, through the EtherCAT master:
#   - which slaves answer and their state       (ethercat slaves)
#   - lost frames of the master                  (ethercat master)
#   - per slave, ESC registers:
#       0x0300  8 bytes  RX error counters, 2 per port (invalid frame, RX error)
#       0x0308  4 bytes  forwarded RX error counters, 1 per port
#       0x0310  4 bytes  lost link counters, 1 per port
#       0x0110  2 bytes  DL status: link / loop state of each port
#       0x0134  2 bytes  AL status code: why the slave is not in OP (0 = none)
# and writes a line only when something changed, plus a full snapshot when
# the number of slaves changes.
#
# It also reads the alarm history of the Delta ASDA-A2-E drives (joint_x/y/z,
# slaves 0-2): P4-00...P4-04, the last 5 alarms, CoE objects 0x2400...0x2404
# (low word: ALnnn, high word: CANopen error code). Read at start, again when
# the slaves are back after a drop (mailbox reads, only on these events), and
# on demand with -H. Codes seen so far: AL185 (8120h) EtherCAT connection
# error, AL3E1 (6200h) SYNC failed, AL500 (9000h) STO/estop, AL180 (8130h)
# PDOs not received. The history is read again when the monitor stops.
#
# It also writes to ec_drives_<date>.log, per Delta drive, through the
# mailbox, every DRIVE_S seconds (default 10), or every FAST_S seconds
# (default 2) for the FAST_DRIVES (default: slave 2, joint_z, the gravity
# axis that brakes the load in descent):
#   - current (0x6078) and torque (0x6077), per mille of the rated value;
#   - velocity (0x606C) and following error (0x60F4), drive units;
#   - regen=1 when torque and velocity have opposite signs: the motor is
#     braking and returning energy to the drive (brake chopper/resistor);
#   - the drive's status monitor registers P0-09...P0-11 (0x2009...0x200B),
#     labelled with what P0-17...P0-19 (0x2011...0x2013) select: 14 = DC bus
#     voltage [V], 16 = IGBT temperature [C], 12 = average load [%],
#     13 = peak load [%]. Selectors at 0 (default) are skipped, so this works
#     before the drives are configured; ec_delta_monitor_setup.sh sets them.
# At start it logs, per drive: following error window (0x6065) and the
# regenerative resistor settings P1-52 [Ohm] (0x2134) and P1-53 [W] (0x2135).
#
# launch_ros2_env.sh prints and clears these counters at each platform start
# (ec_error_counters.sh), so here they count what happened during this start.
# Reading it after a drop:
#   - lost link of slave 1 (joint_y) port 1 AND of slave 2 (joint_z) port 0
#     both increased: the link itself dropped while both were powered
#     (cable, connectors, PHY, shield/ground);
#   - only slave 1 port 1 increased and slave 2 came back with all counters
#     at 0: joint_z (its EtherCAT interface) was reset or power-cycled;
#   - RX errors (rx) rising on that segment before a drop: bad signal on it.
#
# Cost: about 11 short `ethercat` calls every PERIOD s (default 2 s), i.e.
# 1-2% of one housekeeping CPU. The register reads travel in the cyclic frames
# of the running application.
#
# Usage: ec_link_monitor.sh [-m MASTER] [-p PERIOD_S] [-d DRIVE_S] [-F FAST_S] [-f "FAST_DRIVES"]
#                           [-s "SLAVES"] [-o OUT_DIR] [-H]
#   -s  slave positions whose registers are read (default "1 2 3": joint_y,
#       joint_z and the ATI sensor behind it)
#   -H  print the drives' alarm history and exit
#   -P  stop when this process exits (used by launch_ros2_env.sh, which starts
#       the monitor with each platform start when FIT4MED_EC_MONITOR=1)
# Run it as fit4med while the stack runs; stop it with Ctrl+C.

set -u

MASTER=1
PERIOD=2
OUT_ROOT="${FIT4MED_DIAG_ROOT:-$HOME/.ros/fit4med_diagnostics}"
SLAVES=(1 2 3)
DRIVES=(0 1 2)
DRIVE_NAMES=(joint_x joint_y joint_z)
HISTORY_ONLY=false
PARENT_PID=""
DRIVE_S=10
FAST_S=2
FAST_DRIVES=(2)

while getopts "m:p:d:F:f:s:o:P:Hh" opt; do
  case "$opt" in
    m) MASTER="$OPTARG" ;;
    p) PERIOD="$OPTARG" ;;
    d) DRIVE_S="$OPTARG" ;;
    F) FAST_S="$OPTARG" ;;
    f) read -r -a FAST_DRIVES <<< "$OPTARG" ;;
    s) read -r -a SLAVES <<< "$OPTARG" ;;
    o) OUT_ROOT="$OPTARG" ;;
    H) HISTORY_ONLY=true ;;
    P) PARENT_PID="$OPTARG" ;;
    *) sed -n '2,/^set -u/p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
  esac
done

mkdir -p "$OUT_ROOT"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG="$OUT_ROOT/ec_link_monitor_$STAMP.log"
DRIVES_LOG="$OUT_ROOT/ec_drives_$STAMP.log"

# Lower priority: this must never compete with the bring-up.
renice -n 10 -p $$ > /dev/null 2>&1 || true

ts() { date +%H:%M:%S.%3N; }

# One register read as a single line of hex bytes; "offline" if it fails
# (the slave is not on the bus) or takes too long.
reg() {
  local out
  if out=$(timeout 2 ethercat -m "$MASTER" -p "$1" reg_read "$2" "$3" 2> /dev/null); then
    echo "$out" | tr -s ' \n' ' ' | sed 's/ $//'
  else
    echo "offline"
  fi
}

# One SDO read (subindex 0) as a decimal number; "?" if it fails.
sdo() {
  local out
  out=$(timeout 2 ethercat -m "$MASTER" -p "$1" upload "$2" 0 --type "$3" 2> /dev/null \
        | awk '{print $NF; exit}')
  if [[ -z "$out" ]]; then echo "?"; return; fi
  [[ "$out" == 0x* ]] && out=$((out))
  echo "$out"
}

# Name of a Delta monitoring variable (ASDA-A2 manual, 7.2.1).
monitor_name() {
  case "$1" in
    12) echo avg_load_pct ;;
    13) echo peak_load_pct ;;
    14) echo dc_bus_V ;;
    16) echo igbt_C ;;
    *) echo "code$1" ;;
  esac
}

SELECTORS=(0x2011 0x2012 0x2013)   # P0-17, P0-18, P0-19
MONITORS=(0x2009 0x200A 0x200B)    # P0-09, P0-10, P0-11

# One line per drive: current, torque and the configured monitor registers.
# drives_lines <index>...: one line per drive (indices into DRIVES).
drives_lines() {
  local idx p k sel line cur tq vel ferr regen
  for idx in "$@"; do
    p="${DRIVES[$idx]}"
    line="$(ts) ${DRIVE_NAMES[$idx]}:"
    cur=$(sdo "$p" 0x6078 int16)
    if [[ "$cur" == "?" ]]; then
      echo "$line offline"
      continue
    fi
    tq=$(sdo "$p" 0x6077 int16)
    vel=$(sdo "$p" 0x606C int32)
    ferr=$(sdo "$p" 0x60F4 int32)
    regen="?"
    if [[ "$tq" =~ ^-?[0-9]+$ && "$vel" =~ ^-?[0-9]+$ ]]; then
      regen=0
      (( tq * vel < 0 )) && regen=1
    fi
    line+=" current_permille=$cur torque_permille=$tq velocity=$vel following_error=$ferr regen=$regen"
    for k in 0 1 2; do
      sel=$(sdo "$p" "${SELECTORS[$k]}" uint16)
      [[ "$sel" == "?" || "$sel" == "0" ]] && continue
      line+=" $(monitor_name "$sel")=$(sdo "$p" "${MONITORS[$k]}" int32)"
    done
    echo "$line"
  done
}

# Drive settings that matter for tracking and braking, once at start.
drives_settings() {
  local idx p
  for idx in "${!DRIVES[@]}"; do
    p="${DRIVES[$idx]}"
    echo "$(ts) settings slave $p (${DRIVE_NAMES[$idx]}):" \
      "following_error_window(0x6065)=$(sdo "$p" 0x6065 uint32)" \
      "regen_resistor_P1-52=$(sdo "$p" 0x2134 uint16)Ohm" \
      "regen_resistor_P1-53=$(sdo "$p" 0x2135 uint16)W"
  done
}

# Alarm history of one Delta drive: "AL185(8120h) AL500(9000h) ..." from the
# newest (P4-00) to the oldest (P4-04); "-" for an empty record, "?" if the
# mailbox read failed (slave absent or not yet in PREOP).
alarm_history() {
  local i raw v out=""
  for i in 0 1 2 3 4; do
    raw=$(timeout 3 ethercat -m "$MASTER" -p "$1" upload "0x240$i" 0 --type uint32 2> /dev/null \
          | awk '{print $1; exit}')
    if [[ -z "$raw" ]]; then out+="? "; continue; fi
    printf -v v '%08X' "$((raw))"
    if [[ "$v" == "00000000" ]]; then out+="- "; continue; fi
    out+="AL$(sed 's/^0*//' <<< "${v:4:4}")(${v:0:4}h) "
  done
  echo "${out% }"
}

# Returns non-zero if any record could not be read.
alarm_histories() {
  local idx h ok=0
  for idx in "${!DRIVES[@]}"; do
    h=$(alarm_history "${DRIVES[$idx]}")
    [[ "$h" == *"?"* ]] && ok=1
    echo "$(ts) alarm history slave ${DRIVES[$idx]} (${DRIVE_NAMES[$idx]}), newest first: $h"
  done
  return $ok
}

if $HISTORY_ONLY; then
  alarm_histories
  exit 0
fi

slaves_line() {
  timeout 2 ethercat -m "$MASTER" slaves 2> /dev/null \
    | awk '{printf "%s:%s%s ", $1, $3, ($4 == "E" ? "+E" : "")}'
}

lost_frames() {
  timeout 2 ethercat -m "$MASTER" master 2> /dev/null \
    | awk '/Lost frames/ {print $3; exit}'
}

counters_line() {
  local p line=""
  for p in "${SLAVES[@]}"; do
    line+="s$p[rx $(reg "$p" 0x0300 8) | fwd $(reg "$p" 0x0308 4) | lost_link $(reg "$p" 0x0310 4)"
    line+=" | dl $(reg "$p" 0x0110 2) | al_code $(reg "$p" 0x0134 2)] "
  done
  echo "$line"
}

{
  echo "# ec_link_monitor: master $MASTER, every ${PERIOD}s, started $(date -Iseconds) on $(hostname)"
  echo "# slaves: position:state (+E = error flag)"
  echo "# registers per slave: rx = 0x0300 (2 bytes per port: invalid frame, RX error),"
  echo "#   fwd = 0x0308 (1 byte per port), lost_link = 0x0310 (1 byte per port),"
  echo "#   dl = 0x0110 DL status (link/loop per port), al_code = 0x0134 AL status code"
  echo "# drives every ${DRIVE_S}s (slaves ${FAST_DRIVES[*]}: every ${FAST_S}s) in $(basename "$DRIVES_LOG")"
} | tee "$LOG"
alarm_histories | tee -a "$LOG"
{
  echo "# ec_drives: every ${DRIVE_S}s, slaves ${FAST_DRIVES[*]} every ${FAST_S}s; current/torque per mille of"
  echo "# rated (0x6078/0x6077); velocity 0x606C and following_error 0x60F4 in drive units;"
  echo "# regen=1: torque and velocity of opposite sign (the motor brakes, energy back to the drive);"
  echo "# other fields: status monitor registers P0-09..11 named after P0-17..19 (unset ones skipped)"
} > "$DRIVES_LOG"

prev_slaves=""
prev_lost=""
lost_window=()
prev_counters=""
max_slaves=0
history_due=0   # epoch time after which the alarm history is read again (0: no)
next_drives=0
next_fast=0
# Indices (into DRIVES) of the fast and of the slow drives.
fast_idx=(); slow_idx=()
for i in "${!DRIVES[@]}"; do
  if [[ " ${FAST_DRIVES[*]} " == *" ${DRIVES[$i]} "* ]]; then fast_idx+=("$i"); else slow_idx+=("$i"); fi
done
drives_settings >> "$DRIVES_LOG"

finish() {
  echo "$(ts) $1: alarm history at stop" | tee -a "$LOG"
  alarm_histories | tee -a "$LOG"
  echo "# stopped $(date -Iseconds)" | tee -a "$LOG"
  exit 0
}
trap 'finish interrupted' INT TERM

while true; do
  if [[ -n "$PARENT_PID" ]] && ! kill -0 "$PARENT_PID" 2> /dev/null; then
    finish "parent $PARENT_PID gone"
  fi
  if (( ${#slow_idx[@]} > 0 && $(date +%s) >= next_drives )); then
    drives_lines "${slow_idx[@]}" >> "$DRIVES_LOG"
    next_drives=$(( $(date +%s) + DRIVE_S ))
  fi
  if (( ${#fast_idx[@]} > 0 && $(date +%s) >= next_fast )); then
    drives_lines "${fast_idx[@]}" >> "$DRIVES_LOG"
    next_fast=$(( $(date +%s) + FAST_S ))
  fi
  s=$(slaves_line)
  l_raw=$(lost_frames)
  # The master counts lost frames as sent minus received, so frames still on
  # the wire add 1 now and then: log the minimum of the last 5 samples. A real
  # loss raises it for good and shows up within 5 samples.
  l="$l_raw"
  if [[ "$l_raw" =~ ^[0-9]+$ ]]; then
    lost_window+=("$l_raw")
    (( ${#lost_window[@]} > 5 )) && lost_window=("${lost_window[@]:1}")
    l=$(printf '%s\n' "${lost_window[@]}" | sort -n | head -1)
  fi
  c=$(counters_line)
  n=$(wc -w <<< "$s")
  pn=$(wc -w <<< "$prev_slaves")

  if [[ -n "$prev_slaves" && "$n" != "$pn" ]]; then
    echo "$(ts) !!! SLAVE COUNT $pn -> $n: $s" | tee -a "$LOG"
    if (( n > pn && n >= max_slaves )); then
      # All back: read the alarms once their mailbox works (a few s later).
      history_due=$(( $(date +%s) + 5 ))
      history_deadline=$(( history_due + 60 ))
    fi
  fi
  (( n > max_slaves )) && max_slaves=$n
  if (( history_due > 0 )) && (( $(date +%s) >= history_due )); then
    if h=$(alarm_histories); then
      echo "$h" | tee -a "$LOG"
      history_due=0
    elif (( $(date +%s) >= history_deadline )); then
      echo "$h" | tee -a "$LOG"
      echo "$(ts) alarm history still incomplete after 60 s, giving up" | tee -a "$LOG"
      history_due=0
    else
      history_due=$(( $(date +%s) + 5 ))
    fi
  fi
  if [[ "$s" != "$prev_slaves" ]]; then
    echo "$(ts) slaves: $s" | tee -a "$LOG"
  fi
  # Only increases: the real counter never goes down.
  if [[ -z "$prev_lost" ]] || { [[ "$l" =~ ^[0-9]+$ && "$prev_lost" =~ ^[0-9]+$ ]] && (( l > prev_lost )); }; then
    echo "$(ts) master $MASTER lost frames: $l" | tee -a "$LOG"
    prev_lost="$l"
  fi
  if [[ "$c" != "$prev_counters" ]]; then
    echo "$(ts) counters: $c" | tee -a "$LOG"
  fi

  prev_slaves="$s"
  prev_counters="$c"
  sleep "$PERIOD"
done
