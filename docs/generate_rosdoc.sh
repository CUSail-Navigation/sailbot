#!/bin/bash

# Array of ROS 2 packages
packages=(
    "sailboat_launch"
    "sailboat_sensors"
    "sailboat_interface"
    "sailboat_main"
    "sailbot_events"
)

# Iterate over each package and generate documentation
for package in "${packages[@]}"; do
    echo "Building documentation for $package"
    
    # Build the documentation using rosdoc2
    rosdoc2 build --package-path ../$package --output-directory ./docs
    
    # Check if the command was successful
    if [ $? -eq 0 ]; then
        echo "Documentation for $package generated successfully."
    else
        echo "Failed to generate documentation for $package."
    fi
done

