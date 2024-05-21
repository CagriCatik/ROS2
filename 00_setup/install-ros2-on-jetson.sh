#!/bin/bash

set -e  # Exit immediately if a command exits with a non-zero status

echo "Setting Locale..."
locale 
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale

echo "Setting up Sources..."
apt-cache policy | grep universe 
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo "Installing ROS packages..."
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-foxy-desktop
sudo apt install -y ros-foxy-ros-base

echo "Setting up Environment..."
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc  # Adding ROS sourcing to bashrc for persistent setup
source /opt/ros/foxy/setup.bash

echo "ROS 2 installation completed successfully!"
