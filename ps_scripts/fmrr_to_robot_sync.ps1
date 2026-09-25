# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0
#
# Make the robot sources identical to the local workspace sources.
# Windows counterpart of bash_scripts/fit4med_to_robot_sync.sh: Windows has no
# rsync, so the sources are packed with tar.exe, uploaded with scp and
# mirrored on the robot (bash_scripts/fit4med_backup_common.sh).

param(
    [string]$RobotHost = "192.168.1.1",
    # Workspace "src" folder; default: the one containing this repository.
    [string]$LocalPath = "",
    # Sync only this folder, relative to the workspace src (e.g. Fit4Med/rehab_gui).
    [string]$Folder = "",
    [switch]$DryRun,
    [switch]$Backup,    # back up the robot sources first, without asking
    [switch]$NoBackup,  # no backup, no question
    [switch]$Yes        # delete robot-only files without asking
)

. (Join-Path $PSScriptRoot "fmrr_robot_common.ps1")

$archive = $null
try {
    if ($Backup -and $NoBackup) { throw "-Backup and -NoBackup cannot be used together" }

    # --- local sources -----------------------------------------------------
    if (-not $LocalPath) { $LocalPath = Split-Path -Parent $FmrrRepoRoot }
    $LocalPath = (Resolve-Path $LocalPath).Path.TrimEnd('\', '/')
    # Mirroring a folder that is not a workspace "src" would copy its unrelated
    # siblings and delete the robot packages.
    if ((Split-Path -Leaf $LocalPath) -ne "src") {
        throw "$LocalPath is not a workspace 'src' folder. Pass it with -LocalPath <...\src>."
    }
    Write-Host "[INFO] Local workspace sources: $LocalPath"

    $Folder = $Folder.Replace('\', '/').Trim('/')
    while ($Folder.StartsWith('./')) { $Folder = $Folder.Substring(2) }
    if ($Folder) {
        # Also keeps the value safe inside the quoted remote command.
        if ($Folder -notmatch '^[A-Za-z0-9._/-]+$' -or $Folder -match '(^|/)\.\.(/|$)') {
            throw "Invalid -Folder '$Folder': use a path inside the workspace src, e.g. Fit4Med/rehab_gui"
        }
        if (-not (Test-Path -PathType Container (Join-Path $LocalPath $Folder))) {
            throw "Local folder not found: $(Join-Path $LocalPath $Folder)"
        }
        Write-Host "[INFO] Selected folder: $Folder"
    }

    Test-FmrrRobot $RobotHost

    # --- pack and upload ---------------------------------------------------
    $archive = Join-Path $env:TEMP "fit4med_sync.tar"
    $target = if ($Folder) { $Folder } else { "." }
    Write-Host "[INFO] Packing the sources..."
    & tar.exe -c -f $archive --exclude=.git --exclude=.github --exclude=__pycache__ --exclude=*.pyc -C $LocalPath $target
    if ($LASTEXITCODE -ne 0) { throw "tar failed (exit code $LASTEXITCODE)" }
    Write-Host ("[INFO] Archive: {0:N1} MB" -f ((Get-Item $archive).Length / 1MB))

    $remoteArchive = "/tmp/fit4med_sync_$(Get-Date -Format yyyyMMdd_HHmmss).tar"
    Write-Host "[INFO] Uploading to the robot..."
    Send-FmrrLibrary $RobotHost
    & scp.exe -q $archive "${FmrrRobotUser}@${RobotHost}:$remoteArchive"
    if ($LASTEXITCODE -ne 0) { throw "Upload to the robot failed (scp exit code $LASTEXITCODE)" }

    # --- backup ------------------------------------------------------------
    if (-not $DryRun) {
        if ($Backup) { $doBackup = $true }
        elseif ($NoBackup) { $doBackup = $false }
        else { $doBackup = Read-FmrrYesNo "Create a backup of the robot sources before the sync?" "y" }
        if ($doBackup) {
            Invoke-FmrrRobotFunction $RobotHost "fit4med_backup_create" @(Get-FmrrStamp)
            if ($LASTEXITCODE -ne 0) {
                & ssh.exe "${FmrrRobotUser}@${RobotHost}" "rm -f $remoteArchive"
                throw "Backup failed: sync aborted, nothing changed on the robot"
            }
        }
        else {
            Write-Host "[INFO] No backup of the robot sources"
        }
    }

    # --- mirror on the robot -----------------------------------------------
    $dry = if ($DryRun) { "true" } else { "false" }
    $assumeYes = if ($Yes) { "true" } else { "false" }
    # "." = whole workspace (an empty argument would not survive the remote shell).
    $remoteFolder = if ($Folder) { $Folder } else { "." }
    Invoke-FmrrRobotFunction $RobotHost "fit4med_sync_apply_archive" @($remoteArchive, $remoteFolder, $dry, $assumeYes)
    exit $LASTEXITCODE
}
catch {
    Write-Host "[ERROR] $_" -ForegroundColor Red
    exit 1
}
finally {
    if ($archive) { Remove-Item $archive -ErrorAction SilentlyContinue }
}
