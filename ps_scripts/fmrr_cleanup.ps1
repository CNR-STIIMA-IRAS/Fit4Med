# Define variables
$remoteUser = "fit4med"            # Replace with your SSH username
$remoteHost = "192.168.1.1"   # Replace with the remote server's hostname or IP address
$remoteScript = "/home/fit4med/fit4med_ws/src/Fit4Med/bash_scripts/kill_ros_apps.sh"  # Path to the Python script on the remote server

# Execute the Python script remotely
$output = ssh -t "${remoteUser}@${remoteHost}" "bash -l -c '$remoteScript'"
Write-Output $output

