# Copyright 2026 CNR-STIIMA
# SPDX-License-Identifier: Apache-2.0

# IP of the PC running the GUI: auto-detected as the local address used to
# reach the robot PC, override with
#   .\fmrr_bringup.ps1 -GuiIp <this_pc_static_ip>
# This is the single source of truth for GUI_IP -- it names the systemd
# instance, is passed through to `ros2 launch ... gui_ip:=`, and is where
# this script listens locally for the readiness signal. Nothing on the robot
# PC hardcodes it.
param(
    [string]$GuiIp = $(
        try {
            Find-NetRoute -RemoteIPAddress "192.168.1.1" -ErrorAction Stop |
                Select-Object -First 1 -ExpandProperty IPAddress
        }
        catch {
            "192.168.1.2"
        }
    ),
    [ValidateSet("service", "legacy")]
    [string]$Mode = "service",
    [int]$ReadyTimeoutSec = 25
)

# Define variables
$remoteUser = "fit4med"
$remoteHost = "192.168.1.1"

function Wait-Fit4MedReady {
    param(
        [string]$Unit,
        [int]$TimeoutSec
    )

    $udpClient = $null
    try {
        $udpClient = New-Object System.Net.Sockets.UdpClient(5005)
    }
    catch {
        Write-Error "Bringup check FAILED: could not bind local UDP port 5005 ($($_.Exception.Message)). Is the GUI already running?"
        return $false
    }
    $udpClient.Client.ReceiveTimeout = 1000
    $deadline = (Get-Date).AddSeconds($TimeoutSec)
    $remoteEndpoint = New-Object System.Net.IPEndPoint([System.Net.IPAddress]::Any, 0)

    try {
        while ((Get-Date) -lt $deadline) {
            # Fast-fail: don't wait out the full timeout if systemd already
            # gave up (e.g. malformed bringup config, node crash).
            $unitState = ssh -T "${remoteUser}@${remoteHost}" "systemctl --user is-active $Unit" 2>$null
            if ($unitState -eq "failed") {
                Write-Error "Bringup FAILED: systemd unit '$Unit' is in 'failed' state. Recent log:"
                ssh -T "${remoteUser}@${remoteHost}" "journalctl --user -u $Unit -n 20 --no-pager"
                return $false
            }

            try {
                $bytes = $udpClient.Receive([ref]$remoteEndpoint)
                $status = [System.Text.Encoding]::UTF8.GetString($bytes) | ConvertFrom-Json
                Write-Host "Bringup OK: FSM status '$($status.state)' received from $($remoteEndpoint.Address)."
                return $true
            }
            catch [System.Net.Sockets.SocketException] {
                # Receive timed out for this iteration; loop and re-check unit state / deadline.
            }
        }
    }
    finally {
        $udpClient.Close()
    }

    Write-Error "Bringup TIMEOUT: no FSM status received on UDP 5005 within ${TimeoutSec}s. Verify the detected/specified GuiIp ($GuiIp) is actually this machine's address (override with -GuiIp if auto-detection picked the wrong adapter), and that no firewall blocks inbound UDP 5005."
    return $false
}

if ($Mode -eq "service") {
    $unit = "fit4med-bringup@$GuiIp.service"
    ssh -T "${remoteUser}@${remoteHost}" "systemctl --user stop 'fit4med-bringup@*.service'; systemctl --user start $unit"

    if (-not (Wait-Fit4MedReady -Unit $unit -TimeoutSec $ReadyTimeoutSec)) {
        # Launched from a desktop shortcut, the window would otherwise close
        # immediately and the operator would never see the error above.
        Read-Host "Bringup FAILED - review the error above, then press Enter to close this window"
        exit 1
    }
}
else {
    $remoteScript = "source /home/fit4med/fit4med_ws/install/setup.bash; /home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/./log.sh; ros2 launch tecnobody_workbench run_sickPLC.launch.py gui_ip:=$GuiIp"
    ssh -t "${remoteUser}@${remoteHost}" "bash -l -i -c '$remoteScript'"
}

