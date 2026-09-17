# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

$ErrorActionPreference = "Stop"

# Paths are resolved relative to this script, so the repository can sit in any
# folder. On the platform PC this still resolves to C:\Fit4Med.
$ScriptDir = if ($PSScriptRoot) { $PSScriptRoot } else { Split-Path -Parent $MyInvocation.MyCommand.Path }
$RepoRoot = Split-Path -Parent $ScriptDir
$GuiScript = Join-Path $RepoRoot "rehab_gui\rehab_gui\FMRRMainProgram.py"

# Use the repository's uv environment when there is one, otherwise the system
# python, exactly as before.
$VenvPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
$Python = if (Test-Path $VenvPython) { $VenvPython } else { "python" }

try {
    & $Python $GuiScript --remote-ip 192.168.1.1 --maximise-window
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