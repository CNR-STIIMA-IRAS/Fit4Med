#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Select what the Delta ASDA-A2-E drives expose in their status monitor
# registers, so that ec_link_monitor.sh can log it:
#   P0-17 (0x2011) -> P0-09 (0x2009): 14 = DC bus voltage [V]
#   P0-18 (0x2012) -> P0-10 (0x200A): 16 = IGBT temperature [C]
#   P0-19 (0x2013) -> P0-11 (0x200B): 12 = average load [%]
# (codes: ASDA-A2 user manual, 7.2.1 "Description of Monitoring Variable").
# These parameters only choose what is displayed/monitored: they do not change
# the control of the drive. The drive may keep them across power cycles.
#
# Usage: ec_delta_monitor_setup.sh [-m MASTER] [-s "SLAVES"] [--apply | --restore FILE]
#   (no option)  show the current selectors and monitor values, change nothing
#   --apply      save the current selectors to a restore file, then write the
#                codes above and read them back
#   --restore F  write back the selectors saved in F by a previous --apply
#   -s           drive positions on the master (default "0 1 2": joint_x/y/z)
# Run it with the motors off (no movement needed); the stack may be running.

set -u

MASTER=1
SLAVES=(0 1 2)
MODE=show
RESTORE_FILE=""
OUT_ROOT="${FIT4MED_DIAG_ROOT:-$HOME/.ros/fit4med_diagnostics}"

SELECTORS=(0x2011 0x2012 0x2013)   # P0-17, P0-18, P0-19
MONITORS=(0x2009 0x200A 0x200B)    # P0-09, P0-10, P0-11
CODES=(14 16 12)

while [[ $# -gt 0 ]]; do
  case "$1" in
    -m) MASTER="$2"; shift 2 ;;
    -s) read -r -a SLAVES <<< "$2"; shift 2 ;;
    --apply) MODE=apply; shift ;;
    --restore) MODE=restore; RESTORE_FILE="$2"; shift 2 ;;
    *) sed -n '2,/^set -u/p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
  esac
done

sdo_read() {
  local out
  out=$(timeout 3 ethercat -m "$MASTER" -p "$1" upload "$2" 0 --type "$3" 2> /dev/null \
        | awk '{print $NF; exit}')
  if [[ -z "$out" ]]; then echo "?"; return 1; fi
  [[ "$out" == 0x* ]] && out=$((out))
  echo "$out"
}

sdo_write() {
  timeout 3 ethercat -m "$MASTER" -p "$1" download "$2" 0 "$3" --type uint16
}

show() {
  local p k
  for p in "${SLAVES[@]}"; do
    echo "slave $p:"
    for k in 0 1 2; do
      printf '  P0-%d (%s) = %-4s -> P0-%02d (%s) = %s\n' \
        $(( 17 + k )) "${SELECTORS[$k]}" "$(sdo_read "$p" "${SELECTORS[$k]}" uint16)" \
        $(( 9 + k )) "${MONITORS[$k]}" "$(sdo_read "$p" "${MONITORS[$k]}" int32)"
    done
  done
}

case "$MODE" in
  show)
    show
    echo "(nothing written; --apply sets P0-17/18/19 to ${CODES[*]})"
    ;;
  apply)
    mkdir -p "$OUT_ROOT"
    RESTORE_FILE="$OUT_ROOT/delta_monitor_selectors_$(date +%Y%m%d-%H%M%S).txt"
    for p in "${SLAVES[@]}"; do
      for k in 0 1 2; do
        v=$(sdo_read "$p" "${SELECTORS[$k]}" uint16) || {
          echo "ERROR: cannot read ${SELECTORS[$k]} on slave $p: nothing written" >&2; exit 1; }
        echo "$p ${SELECTORS[$k]} $v" >> "$RESTORE_FILE"
      done
    done
    echo "Previous selectors saved in $RESTORE_FILE (use --restore to put them back)"
    for p in "${SLAVES[@]}"; do
      for k in 0 1 2; do
        sdo_write "$p" "${SELECTORS[$k]}" "${CODES[$k]}" \
          || echo "ERROR: write ${SELECTORS[$k]}=${CODES[$k]} on slave $p failed" >&2
      done
    done
    show
    ;;
  restore)
    [[ -f "$RESTORE_FILE" ]] || { echo "ERROR: $RESTORE_FILE not found" >&2; exit 1; }
    while read -r p obj v; do
      sdo_write "$p" "$obj" "$v" || echo "ERROR: write $obj=$v on slave $p failed" >&2
    done < "$RESTORE_FILE"
    show
    ;;
esac
