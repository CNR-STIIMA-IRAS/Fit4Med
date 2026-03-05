# Define variables
$remoteUser = "fit4med"
$remoteHost = "192.168.1.1"
$scpPathFiles = "/home/fit4med/.ros/fit4med_log/*"
$localDestination = "C:\Users\keba\Desktop\fit4med_logs\"

# Ensure local destination exists
if (!(Test-Path $localDestination)) {
    New-Item -ItemType Directory -Path $localDestination
}

# Build remote path correctly
$remoteTarget = "${remoteUser}@${remoteHost}:$scpPathFiles"

# Execute the SCP command
Write-Output "Copy the remote files ... from $remoteTarget to $localDestination"
scp -r $remoteTarget $localDestination
