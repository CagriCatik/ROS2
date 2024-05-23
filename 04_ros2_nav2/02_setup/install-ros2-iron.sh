#!/bin/bash

# Check if user is running as root
if [ "$(id -u)" -ne 0 ]; then
  echo "Please run this script as root"
  exit 1
fi

# Set locale
echo "Setting locale..."
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS 2 repository
echo "Adding ROS 2 repository..."
apt-get update && apt-get install -y software-properties-common curl
add-apt-repository universe
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install development tools (optional)
echo "Installing development tools..."
apt-get update && apt-get install -y ros-dev-tools

# Install ROS 2 Iron
echo "Installing ROS 2 Iron..."
apt-get update && apt-get install -y ros-iron-desktop

# Source ROS 2 environment setup file
echo "Sourcing ROS 2 environment setup file..."
source /opt/ros/iron/setup.bash

echo "ROS 2 Iron installation completed successfully."

