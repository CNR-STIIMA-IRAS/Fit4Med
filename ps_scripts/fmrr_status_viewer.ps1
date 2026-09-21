param([string]$GuiIp = "192.168.1.2")

$remoteUser = "fit4med"
$remoteHost = "192.168.1.1"
$unit = "fit4med-bringup@$GuiIp.service"

ssh -t "${remoteUser}@${remoteHost}" "journalctl --user -fu $unit -o short-precise"
