#!/bin/bash

source /opt/ros/iron/setup.bash


# Function to check if a package is installed
check_package_installed() {
    dpkg -s "$1" &> /dev/null
    return $?
}

# Function to install a package if it's not already installed
install_package() {
    if ! check_package_installed "$1"; then
        echo "Installing $1..."
        sudo apt install -y "$1"
    else
        echo "$1 is already installed."
    fi
}

# Function to check ROS 2 distribution
check_ros_distro() {
    if [ -z "$ROS_DISTRO" ]; then
        echo "ROS_DISTRO is not set. Please make sure you have sourced the ROS 2 setup file."
        exit 1
    fi
    echo "ROS 2 distribution: $ROS_DISTRO"
}

# Install ROS packages
install_ros_packages() {
    echo "Installing ROS packages..."
    install_package "ros-$ROS_DISTRO-navigation2"
    install_package "ros-$ROS_DISTRO-nav2-bringup"
    install_package "ros-$ROS_DISTRO-turtlebot3-gazebo"
}


# Main function
main() {
    # Check ROS 2 distribution
    check_ros_distro
    
    install_ros_packages
    
    # Source ROS setup file
    source /opt/ros/iron/setup.bash
    
    # Set environment variables
    export TURTLEBOT3_MODEL=waffle
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/iron/share/turtlebot3_gazebo/models
    
    # Launch ROS node
    echo "Launching ROS node..."
    ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
}

# Execute main function
main
