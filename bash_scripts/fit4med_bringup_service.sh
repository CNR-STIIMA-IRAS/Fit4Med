#!/usr/bin/env bash
set -Eeuo pipefail

DEFAULT_EEG_DELAY_MS="6000"
# EEG_DELAY_MS is optionally configured via a JSON file -- see
# fit4med_bringup_config.template.json (next to this script) for the format
# and setup instructions. The real, deployment-specific file lives outside
# this repo so it survives a `git pull`/reinstall; FIT4MED_BRINGUP_CONFIG is
# normally set via Environment= in systemctl_services/fit4med-bringup@.service.
# If unset/missing here, falls back to DEFAULT_EEG_DELAY_MS above.
CONFIG_FILE="${FIT4MED_BRINGUP_CONFIG:-$(dirname "${BASH_SOURCE[0]}")/fit4med_bringup_config.json}"

# GUI_IP is a fact about whichever Windows PC is running the GUI right now --
# the robot PC has no way to know it on its own, so it must always be passed
# in (systemd passes the instance name, e.g. an IP, as $1). Required, no default.
GUI_IP="${1:-}"
if [[ -z "${GUI_IP}" ]]; then
    echo "ERROR: GUI_IP must be provided as the first argument (systemd instance name)." >&2
    exit 1
fi

EEG_DELAY_MS="${DEFAULT_EEG_DELAY_MS}"
if [[ -f "${CONFIG_FILE}" ]]; then
    EEG_DELAY_MS="$(
        python3 - "${CONFIG_FILE}" "${DEFAULT_EEG_DELAY_MS}" <<'PY'
import json
import sys

path, default_eeg_delay_ms = sys.argv[1:]

try:
    with open(path, "r", encoding="utf-8") as config_file:
        config = json.load(config_file)
except Exception as exc:
    print(f"ERROR: cannot read bringup config {path}: {exc}", file=sys.stderr)
    sys.exit(1)

if not isinstance(config, dict):
    print(f"ERROR: bringup config {path} must contain a JSON object.", file=sys.stderr)
    sys.exit(1)

try:
    eeg_delay_ms = int(config.get("EEG_DELAY_MS", default_eeg_delay_ms))
except (TypeError, ValueError):
    print("ERROR: EEG_DELAY_MS must be an integer number of milliseconds.", file=sys.stderr)
    sys.exit(1)

if eeg_delay_ms < 0:
    print("ERROR: EEG_DELAY_MS must be greater than or equal to 0.", file=sys.stderr)
    sys.exit(1)

print(eeg_delay_ms)
PY
    )"
else
    echo "WARNING: bringup config not found at ${CONFIG_FILE}; using default EEG_DELAY_MS=${EEG_DELAY_MS}." >&2
fi

# Optional manual override of EEG_DELAY_MS only, for ad-hoc testing.
EEG_DELAY_MS="${2:-${EEG_DELAY_MS}}"

if ! [[ "${EEG_DELAY_MS}" =~ ^[0-9]+$ ]]; then
    echo "ERROR: EEG_DELAY_MS must be an integer number of milliseconds." >&2
    exit 1
fi

source /opt/ros/jazzy/setup.bash
source /home/fit4med/fit4med_ws/install/setup.bash

/home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/log.sh || true

exec ros2 launch tecnobody_workbench run_sickPLC.launch.py gui_ip:="${GUI_IP}" eeg_delay_ms:="${EEG_DELAY_MS}"
