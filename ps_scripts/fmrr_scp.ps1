# Define variables
$remoteUser = "fit4med"
$remoteHost = "192.168.1.1"
$scpPathFiles = "/home/fit4med/fit4med_ws/src/Fit4Med/*"
$localDestination = "C:\Fit4Med"

# Ensure local destination exists
if (!(Test-Path $localDestination)) {
    New-Item -ItemType Directory -Path $localDestination
}

# Build remote path correctly
$remoteTarget = "${remoteUser}@${remoteHost}:$scpPathFiles"

# Execute the SCP command
Write-Output "Copy the remote files ... from $remoteTarget to $localDestination"
scp -r $remoteTarget $localDestination

# Remove all .pyc files recursively from the local path
Write-Output "Removing the pyc files ..."
Get-ChildItem -Path $localDestination -Filter *.pyc -Recurse | Remove-Item -Force
Get-ChildItem -Path $localDestination -Filter __pycache__ -Recurse | Remove-Item -Force
