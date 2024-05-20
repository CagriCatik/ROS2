#!/usr/bin/env bash
set -eu

# Author: YOUR_NAME
# Last updated: $(date)

CHOOSE_ROS_DISTRO=eloquent
INSTALL_PACKAGE=desktop

# Check if the architecture is aarch64 (ARM 64-bit)
if [[ $(uname -m) != "aarch64" ]]; then
    echo "This script is intended for aarch64 (ARM 64-bit) architecture. Exiting."
    exit 1
fi

# Add the NVIDIA Jetson ROS repository
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update

printf '%s\n' "ROS Eloquent installation script"

sudo apt-get install -y ros-$CHOOSE_ROS_DISTRO-$INSTALL_PACKAGE
sudo apt-get install -y python3-argcomplete
sudo apt-get install -y python3-colcon-common-extensions
sudo apt-get install -y python-rosdep python3-vcstool # https://index.ros.org/doc/ros2/Installation/Linux-Development-Setup/

# Initialize rosdep if not done already
[ -e /etc/ros/rosdep/sources/list.d/20-default.list ] || sudo rosdep init

# Update rosdep including end-of-life distros
rosdep update --include-eol-distros # https://discourse.ros.org/t/rosdep-and-eol-distros/7640

# Add ROS source to bashrc if not present
grep -F "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash" ~/.bashrc || echo "source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash" >> ~/.bashrc

# Deactivate strict mode for the following commands
set +u

# Source ROS setup.bash to make ROS commands available in this script
source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash

# Reactivate strict mode
set -u

echo "Success installing ROS2 $CHOOSE_ROS_DISTRO"
echo "Run 'source /opt/ros/$CHOOSE_ROS_DISTRO/setup.bash'"
