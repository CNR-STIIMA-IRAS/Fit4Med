# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

$ErrorActionPreference = "Stop"

try {
    python "C:\Fit4Med\rehab_gui\rehab_gui\FMRRMainProgram.py" --remote-ip 192.168.1.1 --maximise-window
    $exitCode = $LASTEXITCODE

    if ($exitCode -ne 0) {
        Write-Host ""
        Write-Host "GUI exited with error code $exitCode" -ForegroundColor Red
        Read-Host "Press Enter to close"
        exit $exitCode
    }
}
catch {
    Write-Host ""
    Write-Host "Failed to launch GUI:" -ForegroundColor Red
    Write-Host $_
    Read-Host "Press Enter to close"
    exit 1
}