#!/bin/bash

# Script to add ROS2 setup script sourcing to Bash configuration

ROS_DISTRO="foxy"  # Set your ROS2 distribution

# Define the ROS2 setup script path
ROS_SETUP_SCRIPT="/opt/ros/${ROS_DISTRO}/setup.bash"

# Check if the line is already present in the Bash configuration
if ! grep -q "source ${ROS_SETUP_SCRIPT}" ~/.bashrc; then
    # Add the line to source the ROS2 setup script at the end of ~/.bashrc
    echo "source ${ROS_SETUP_SCRIPT}" >> ~/.bashrc
    echo "ROS2 setup script sourced added to ~/.bashrc"
else
    echo "ROS2 setup script already sourced in ~/.bashrc"
fi

# Inform the user to open a new terminal for changes to take effect
echo "Please open a new terminal to apply the changes."
