#!/usr/bin/env bash

# Exit on error, undefined var, or failed pipeline
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=fit4med_robot_sync_common.sh
source "${SCRIPT_DIR}/fit4med_robot_sync_common.sh"

fit4med_sync_from_robot "$0" "$@"
