# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Shared by fmrr_to_robot_sync.ps1, fmrr_backup.ps1 and fmrr_restore.ps1
# (dot-sourced). The work on the robot is done by the same bash functions the
# Linux scripts use (bash_scripts/fit4med_backup_common.sh), uploaded at each
# run, so Windows and Linux behave the same. Needs the OpenSSH client
# (ssh.exe/scp.exe) and tar.exe, both part of Windows 10/11.

$ErrorActionPreference = "Stop"

$FmrrRobotUser = "fit4med"
$FmrrScriptDir = if ($PSScriptRoot) { $PSScriptRoot } else { Split-Path -Parent $MyInvocation.MyCommand.Path }
$FmrrRepoRoot = Split-Path -Parent $FmrrScriptDir
$FmrrBashLibrary = Join-Path $FmrrRepoRoot "bash_scripts\fit4med_backup_common.sh"
$FmrrRemoteLibrary = "/tmp/fit4med_backup_common.sh"

function Test-FmrrRobot([string]$RobotHost) {
    Write-Host "[INFO] Robot: $FmrrRobotUser@$RobotHost"
    if (-not (Test-Connection -ComputerName $RobotHost -Count 1 -Quiet)) {
        throw "Robot not reachable: $RobotHost"
    }
    Write-Host "[OK] Robot reachable"
}

# Ask a yes/no question. $Default is "y" or "n".
function Read-FmrrYesNo([string]$Question, [string]$Default) {
    $hint = if ($Default -eq "y") { "[Y/n]" } else { "[y/N]" }
    $answer = Read-Host "$Question $hint"
    if ([string]::IsNullOrWhiteSpace($answer)) { $answer = $Default }
    return $answer -match '^(y|yes|s|si)$'
}

# Timestamp of the backup folder, from this PC's clock (the robot is offline
# and its clock may be wrong).
function Get-FmrrStamp { return (Get-Date -Format "yyyyMMdd/HHmm") }

# Upload the bash library, with Linux line endings whatever the checkout has.
function Send-FmrrLibrary([string]$RobotHost) {
    $text = [System.IO.File]::ReadAllText($FmrrBashLibrary) -replace "`r`n", "`n"
    $tmp = Join-Path $env:TEMP "fit4med_backup_common.sh"
    [System.IO.File]::WriteAllText($tmp, $text, (New-Object System.Text.UTF8Encoding($false)))
    try {
        & scp.exe -q $tmp "${FmrrRobotUser}@${RobotHost}:$FmrrRemoteLibrary"
        if ($LASTEXITCODE -ne 0) { throw "Upload to the robot failed (scp exit code $LASTEXITCODE)" }
    }
    finally {
        Remove-Item $tmp -ErrorAction SilentlyContinue
    }
}

# Run one library function on the robot, with a terminal for its questions.
# Called as a statement (not captured, or the live output and the prompts
# would be swallowed); the result is in $LASTEXITCODE. Arguments are plain
# words, so no quoting is needed (Windows PowerShell 5.1 mangles embedded
# double quotes in native arguments).
function Invoke-FmrrRobotFunction([string]$RobotHost, [string]$Function, [string[]]$Arguments) {
    foreach ($a in $Arguments) {
        if ($a -notmatch '^[A-Za-z0-9._/:-]+$') { throw "Unsupported character in remote argument: '$a'" }
    }
    & ssh.exe -t "${FmrrRobotUser}@${RobotHost}" "bash -c '. $FmrrRemoteLibrary && $Function $($Arguments -join ' ')'"
}
