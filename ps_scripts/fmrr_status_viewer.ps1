param(
    [string]$GuiIp = $(
        try {
            Find-NetRoute -RemoteIPAddress "192.168.1.1" -ErrorAction Stop |
                Select-Object -First 1 -ExpandProperty IPAddress
        }
        catch {
            "192.168.1.2"
        }
    )
)

$remoteUser = "fit4med"
$remoteHost = "192.168.1.1"
$unit = "fit4med-bringup@$GuiIp.service"

$stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
ssh -t "${remoteUser}@${remoteHost}" "journalctl --user -fu $unit -o short-precise"
$stopwatch.Stop()

if ($LASTEXITCODE -ne 0 -and $stopwatch.Elapsed.TotalSeconds -lt 3) {
    # Failed almost immediately (e.g. host unreachable) rather than being
    # stopped by the operator watching live logs -- launched from a desktop
    # shortcut, the window would otherwise close before this could be read.
    Read-Host "Could not read the log for '$unit' (ssh exit code $LASTEXITCODE). Review the error above, then press Enter to close this window"
}
