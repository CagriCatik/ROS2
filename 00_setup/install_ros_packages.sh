#!/bin/bash

# Function to check and install a package
install_package() {
    package_name=$1
    if ! dpkg -l | grep -q $package_name; then
        echo "Installing $package_name..."
        sudo apt install -y $package_name
        echo "$package_name installed successfully."
    else
        echo "$package_name is already installed."
    fi
}

echo "Installing necessary packages for ROS2 development..."

# Install Terminator
install_package terminator

# Install Python3 and pip
install_package python3
install_package python3-pip

# Install Python3 argcomplete
install_package python3-argcomplete

# Install Python3 colcon common extensions
install_package python3-colcon-common-extensions

# Install ROS Galactic Gazebo messages
install_package ros-galactic-gazebo-msgs

# Install ROS Galactic Gazebo plugins
install_package ros-galactic-gazebo-plugins

echo "Packages installation completed successfully."
