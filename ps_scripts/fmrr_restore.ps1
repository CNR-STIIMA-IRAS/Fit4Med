# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# List the robot source backups and restore the chosen one.
# Windows counterpart of bash_scripts/fit4med_restore.sh.

param([string]$RobotHost = "192.168.1.1")

. (Join-Path $PSScriptRoot "fmrr_robot_common.ps1")

try {
    Test-FmrrRobot $RobotHost
    Send-FmrrLibrary $RobotHost
    # The stamp names the optional safety backup taken before restoring.
    Invoke-FmrrRobotFunction $RobotHost "fit4med_backup_restore_interactive" @(Get-FmrrStamp)
    exit $LASTEXITCODE
}
catch {
    Write-Host "[ERROR] $_" -ForegroundColor Red
    exit 1
}
