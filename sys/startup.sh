#!/bin/bash

# Define log file
LOG_FILE="$HOME/sailbot/sys/sys.log"

# Log start time
echo "ROS Startup Script Started: $(date)" >> "$LOG_FILE"

# Source ROS 2 setup 
echo "Sourcing ROS 2 setup file..." >> "$LOG_FILE"
source /opt/ros/humble/setup.bash
source ~/.bashrc

# Navigate to ROS workspace
echo "Navigating to ROS workspace..." >> "$LOG_FILE"
cd ~/sailbot || { echo "Workspace directory not found!" >> "$LOG_FILE"; exit 1; }

# Build the workspace
echo "Building the workspace using colcon build..." >> "$LOG_FILE"
colcon build >> "$LOG_FILE" 2>&1
if [ $? -ne 0 ]; then
    echo "Failed to build workspace. Check the log file for details." >> "$LOG_FILE"
    exit 1
fi

# Source the setup file
echo "Sourcing environment..." >> "$LOG_FILE"
source install/setup.bash >> "$LOG_FILE" 2>&1

# Run Ngrok for rosbridge on 9090
ngrok http 9090 &

# Run ROS
echo "Starting ROS @ ros.out..." >> "$LOG_FILE"
ros2 launch sailboat_launch sailboat.launch_sim.py > sys/ros.out

echo "ROS application started successfully!" >> "$LOG_FILE"
exit 0
