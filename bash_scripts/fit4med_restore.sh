#!/usr/bin/env bash
# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# List the robot source backups and restore the chosen one.
# Works on the robot itself or, from the user PC, through ssh (see --help).

set -uo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# shellcheck source=fit4med_backup_common.sh
source "${SCRIPT_DIR}/fit4med_backup_common.sh"

fit4med_backup_main "$0" fit4med_backup_restore_interactive "$@"
