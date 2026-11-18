#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Print, and optionally clear, the error counters of the EtherCAT slave
# controllers (ESC) of a master. The counters only reset when a slave is
# powered up or when they are cleared, and saturate at 255.
#
# Per port (0-1 printed, ports 2-3 are not used by these slaves):
#   inv   invalid frames received          (0x0300 + 2*port)
#   rx    RX errors (physical layer)       (0x0301 + 2*port)
#   fwd   forwarded RX errors              (0x0308 + port)
#   lost  link losses                      (0x0310 + port)
# Per slave: ecat = EtherCAT processing unit errors (0x030C), pdi = PDI errors
# (0x030D).
#
# Clearing: writing 0x0300 clears 0x0300-0x030B, writing 0x030C and 0x030D
# clears those, writing 0x0310 clears the lost-link counters.
#
# Usage: ec_error_counters.sh [-m MASTER] [-s "SLAVES"] dump|clear|dump-clear
#   SLAVES default: every slave the master sees.
# launch_ros2_env.sh runs "dump-clear" at each platform start
# (FIT4MED_EC_CLEAR_COUNTERS=0 to skip it), so the counters printed in
# console.log are those since the previous start.

set -u

MASTER=1
SLAVES=()

while getopts "m:s:h" opt; do
  case "$opt" in
    m) MASTER="$OPTARG" ;;
    s) read -r -a SLAVES <<< "$OPTARG" ;;
    *) sed -n '2,/^set -u/p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
  esac
done
shift $(( OPTIND - 1 ))
ACTION="${1:-dump}"

if [[ ${#SLAVES[@]} -eq 0 ]]; then
  read -r -a SLAVES <<< "$(timeout 3 ethercat -m "$MASTER" slaves 2> /dev/null | awk '{print $1}' | xargs)"
fi
if [[ ${#SLAVES[@]} -eq 0 ]]; then
  echo "ERROR: no slave found on master $MASTER" >&2
  exit 1
fi

# reg <slave> <address> <bytes>: the bytes as decimal numbers, or "?" each.
reg() {
  local out
  if out=$(timeout 3 ethercat -m "$MASTER" -p "$1" reg_read "$2" "$3" 2> /dev/null); then
    local b
    for b in $out; do printf '%d ' "$b"; done
  else
    printf '? %.0s' $(seq "$3")
  fi
}

dump() {
  local p rx fwd lost misc
  for p in "${SLAVES[@]}"; do
    read -r -a rx <<< "$(reg "$p" 0x0300 8)"
    read -r -a fwd <<< "$(reg "$p" 0x0308 4)"
    read -r -a misc <<< "$(reg "$p" 0x030C 2)"
    read -r -a lost <<< "$(reg "$p" 0x0310 4)"
    printf 'slave %s: port0 inv=%s rx=%s fwd=%s lost=%s | port1 inv=%s rx=%s fwd=%s lost=%s | ecat=%s pdi=%s\n' \
      "$p" "${rx[0]}" "${rx[1]}" "${fwd[0]}" "${lost[0]}" \
      "${rx[2]}" "${rx[3]}" "${fwd[1]}" "${lost[1]}" "${misc[0]}" "${misc[1]}"
  done
}

clear_counters() {
  local p a ok=0
  for p in "${SLAVES[@]}"; do
    for a in 0x0300 0x030C 0x030D 0x0310; do
      timeout 3 ethercat -m "$MASTER" -p "$p" reg_write --type uint8 "$a" 0 > /dev/null 2>&1 || ok=1
    done
  done
  return $ok
}

case "$ACTION" in
  dump) dump ;;
  clear) clear_counters || { echo "WARNING: some counters could not be cleared" >&2; exit 1; } ;;
  dump-clear)
    dump
    if clear_counters; then
      echo "(counters cleared on slaves ${SLAVES[*]})"
    else
      echo "WARNING: some counters could not be cleared" >&2
      exit 1
    fi
    ;;
  *) echo "Usage: $0 [-m MASTER] [-s \"SLAVES\"] dump|clear|dump-clear" >&2; exit 1 ;;
esac
