#!/bin/bash
source /home/$USER/.bashrc

# Create log directory if it doesn't exist
logdir="$(pwd)/logs"
mkdir -p $logdir

# Get current date and time
datetime=$(date '+%Y-%m-%d_%H-%M-%S')

# Print both to the screen and the log file
logfile="$logdir/kill_ros2_nodes_$datetime.log"

# Kill all ROS 2 nodes
echo "Killing all ROS 2 nodes..." | tee -a $logfile

nodes=$(ros2 node list)
for node in $nodes; do
    pid=$(ps aux | grep $node | grep -v grep | awk '{print $2}')
    if [ ! -z "$pid" ]; then
        echo "Killing node $node with PID $pid" | tee -a $logfile
        kill -SIGINT $pid
    fi
done

# Check if any nodes are still running
remaining_nodes=$(ros2 node list)
if [ -z "$remaining_nodes" ]; then
    echo "All ROS 2 nodes successfully killed." | tee -a $logfile
else
    echo "Remaining ROS 2 nodes: $remaining_nodes" | tee -a $logfile
fi

