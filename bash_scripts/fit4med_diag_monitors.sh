#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Turn on/off the diagnostic monitors that launch_ros2_env.sh starts with each
# platform start (ec_link_monitor.sh, rt_load_monitor.sh), for both ways of
# starting the stack:
#   - the teach pendant (fmrr_bringup.ps1 -> systemd user unit
#     fit4med-bringup@<ip>.service): a drop-in for every instance of the unit;
#   - a shell (ros2 launch ... run_sickPLC.launch.py): a marked block in
#     ~/.bashrc.
# Both set FIT4MED_EC_MONITOR=1 and FIT4MED_CPU_MONITOR=1. The change applies
# from the next start of the stack.
#
# Usage (on the robot, as fit4med): fit4med_diag_monitors.sh enable|disable|status

set -u

DROPIN_DIR="$HOME/.config/systemd/user/fit4med-bringup@.service.d"
DROPIN="$DROPIN_DIR/20-diag-monitors.conf"
BASHRC="$HOME/.bashrc"
BEGIN="# >>> fit4med diagnostic monitors >>>"
END="# <<< fit4med diagnostic monitors <<<"

remove_bashrc_block() {
  [[ -f "$BASHRC" ]] || return 0
  sed -i "/^$BEGIN\$/,/^$END\$/d" "$BASHRC"
}

reload() {
  systemctl --user daemon-reload 2> /dev/null \
    || echo "NOTE: systemctl --user not reachable here; the unit change applies after the next login." >&2
}

status() {
  local unit=off shell=off
  [[ -f "$DROPIN" ]] && grep -q 'FIT4MED_EC_MONITOR=1' "$DROPIN" && unit=on
  grep -q "^$BEGIN\$" "$BASHRC" 2> /dev/null && shell=on
  echo "pendant (systemd unit): $unit"
  echo "shell (~/.bashrc):      $shell"
  if [[ "$unit" == on && "$shell" == on ]]; then
    echo "=> monitors ENABLED (from the next start of the stack)"
  elif [[ "$unit" == off && "$shell" == off ]]; then
    echo "=> monitors DISABLED"
  else
    echo "=> monitors only partly enabled: run '$0 enable' again"
  fi
}

case "${1:-status}" in
  enable)
    mkdir -p "$DROPIN_DIR"
    printf '[Service]\nEnvironment=FIT4MED_EC_MONITOR=1\nEnvironment=FIT4MED_CPU_MONITOR=1\n' > "$DROPIN"
    reload
    remove_bashrc_block
    printf '%s\nexport FIT4MED_EC_MONITOR=1\nexport FIT4MED_CPU_MONITOR=1\n%s\n' "$BEGIN" "$END" >> "$BASHRC"
    status
    ;;
  disable)
    rm -f "$DROPIN"
    rmdir "$DROPIN_DIR" 2> /dev/null || true
    reload
    remove_bashrc_block
    status
    ;;
  status)
    status
    ;;
  *)
    sed -n '2,/^set -u/p' "$0" | sed 's/^# \{0,1\}//'
    exit 1
    ;;
esac
