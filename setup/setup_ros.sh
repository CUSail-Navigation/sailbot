#!/bin/bash

# Navigate to the ROS2 workspace
cd /home/ros2_user/ros2_ws

# Build the ROS2 workspace
colcon build

# Source the workspace
source install/setup.bash

# Keep the container running interactively (optional)
exec bash


