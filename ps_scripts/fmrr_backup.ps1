# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Create a backup of the robot sources in /home/fit4med/bkp/YYYYMMDD/HHMM/.
# Windows counterpart of bash_scripts/fit4med_backup.sh.

param([string]$RobotHost = "192.168.1.1")

. (Join-Path $PSScriptRoot "fmrr_robot_common.ps1")

try {
    Test-FmrrRobot $RobotHost
    Send-FmrrLibrary $RobotHost
    Invoke-FmrrRobotFunction $RobotHost "fit4med_backup_create" @(Get-FmrrStamp)
    exit $LASTEXITCODE
}
catch {
    Write-Host "[ERROR] $_" -ForegroundColor Red
    exit 1
}
