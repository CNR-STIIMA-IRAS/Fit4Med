# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

# Copy to this PC the log archives of the last bring-ups and the GUI logs.
#
#   .\fmrr_retrieve_logs.ps1             the last 3 bring-ups, plus the one running
#   .\fmrr_retrieve_logs.ps1 -Last 10    the last 10
#   .\fmrr_retrieve_logs.ps1 -All        all of them
#
# The robot writes one zip per bring-up when run_sickPLC.launch.py exits
# (~/.ros/fit4med_log/run_NNNN_YYYYMMDD-HHMMSS.zip, NNNN growing at each
# bring-up), with the ROS logs and the bring-up / ethercat.service journals of
# that bring-up only (bash_scripts/fit4med_session_log.sh). The bring-up still
# running, if any, is copied as run_NNNN_..._partial.zip, its logs up to now.
# Archives already on this PC are not copied again.
#
# The EtherCAT part of the journals needs fit4med in the systemd-journal group
# (see the end of bash_scripts/README.md).

param(
    # Not used any more (the journals are in the archives); kept so existing
    # shortcuts that pass it keep working.
    [string]$GuiIp = "",
    # Where FMRRMainProgram.py writes its session logs and their backups (see rehab_gui/session_log.py)
    [string]$GuiLogDir = "C:\temp",
    [ValidateRange(1, 100000)]
    [int]$Last = 3,
    [switch]$All
)

$remoteUser = "fit4med"
$remoteHost = "192.168.1.1"
$robot = "${remoteUser}@${remoteHost}"
$robotScript = "/home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/fit4med_session_log.sh"
$localDestination = Join-Path $env:USERPROFILE "Desktop\fit4med_logs"

if (!(Test-Path $localDestination)) {
    New-Item -ItemType Directory -Path $localDestination | Out-Null
}

# "path<TAB>start time<TAB>end reason", as printed by fit4med_session_log.sh
function ConvertFrom-FmrrArchiveLine([string]$Line) {
    $fields = $Line -split "`t"
    [pscustomobject]@{
        Path   = $fields[0]
        Name   = ($fields[0] -split '/')[-1]
        Start  = if ($fields.Count -gt 1) { $fields[1] } else { "" }
        Reason = if ($fields.Count -gt 2) { $fields[2] } else { "" }
    }
}

# The robot logs use the robot clock (offline, may drift), the GUI logs this
# PC's clock: record both now, to line the two sets of logs up.
$clocksFile = Join-Path $localDestination "clocks.txt"
$robotNow = ssh -T $robot "date --iso-8601=ns"
@(
    "Clocks at log retrieval (compare to align robot and GUI logs):",
    "  robot   : $robotNow",
    "  this PC : $(Get-Date -Format o)"
) | Out-File -FilePath $clocksFile -Encoding utf8

# Bring-up still running: zipped on the robot in a temporary folder. This also
# archives the bring-ups left open by a launch that was killed.
Write-Output "Pack the bring-up still running (if any) on the robot ..."
$partials = @(ssh -T $robot "bash $robotScript snapshot" | Where-Object { $_ } |
              ForEach-Object { ConvertFrom-FmrrArchiveLine $_ })
if ($LASTEXITCODE -ne 0) {
    Write-Warning "Robot side failed (exit code $LASTEXITCODE). Is $robotScript on the robot? Update the robot software with fmrr_to_robot_sync.ps1."
}

$count = if ($All) { 0 } else { $Last }
$archives = @(ssh -T $robot "bash $robotScript list $count" | Where-Object { $_ } |
              ForEach-Object { ConvertFrom-FmrrArchiveLine $_ })

$copied = @()
$present = @()
foreach ($archive in $archives) {
    if (Test-Path (Join-Path $localDestination $archive.Name)) {
        $present += $archive
        continue
    }
    scp -q "${robot}:$($archive.Path)" "$localDestination"
    if ($LASTEXITCODE -eq 0) { $copied += $archive }
    else { Write-Warning "Copy failed: $($archive.Path)" }
}
foreach ($partial in $partials) {
    # always copied again: it is a picture of a bring-up still going on
    scp -q "${robot}:$($partial.Path)" "$localDestination"
    if ($LASTEXITCODE -eq 0) { $copied += $partial }
    else { Write-Warning "Copy failed: $($partial.Path)" }
    if ($partial.Path -match '^(/tmp/fit4med_snapshot\.[A-Za-z0-9]+)/[^/]+\.zip$') {
        $remoteTmp = $Matches[1]
        ssh -T $robot "rm -rf -- '$remoteTmp'"
    }
}

# A partial copy is outdated once the archive of the same bring-up is here.
Get-ChildItem -Path $localDestination -Filter "run_*_partial.zip" -ErrorAction SilentlyContinue | ForEach-Object {
    if (Test-Path (Join-Path $localDestination ($_.Name -replace '_partial\.zip$', '.zip'))) {
        Remove-Item $_.FullName
    }
}

if ($archives.Count -eq 0 -and $partials.Count -eq 0) {
    Write-Output "No bring-up archive on the robot."
}
if ($copied.Count -gt 0) {
    Write-Output "`nCopied to ${localDestination}:"
    $copied | Format-Table -AutoSize -Wrap Name, Start, @{ Label = "How it ended"; Expression = { $_.Reason } } | Out-String | Write-Output
}
if ($present.Count -gt 0) {
    Write-Output "Already on this PC: $(($present | ForEach-Object { $_.Name }) -join ', ')"
}

# Local GUI logs (current session + rotated backups).
$guiLogDestination = Join-Path $localDestination "gui"
$guiLogs = @(foreach ($pattern in "pyqt_hang*.log", "gui_errors*.log", "gui_console*.log") {
    Get-ChildItem -Path $GuiLogDir -Filter $pattern -ErrorAction SilentlyContinue
})
if ($guiLogs.Count -gt 0) {
    if (!(Test-Path $guiLogDestination)) {
        New-Item -ItemType Directory -Path $guiLogDestination | Out-Null
    }
    Write-Output "Copy $($guiLogs.Count) GUI log(s) from $GuiLogDir to $guiLogDestination"
    $guiLogs | Copy-Item -Destination $guiLogDestination -Force
}
else {
    Write-Output "No GUI log found in $GuiLogDir"
}
