#!/bin/bash

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Check if the system is Ubuntu 18.04 or 16.04
UBUNTU_VERSION=$(lsb_release -cs)
if [ "$UBUNTU_VERSION" != "bionic" ] && [ "$UBUNTU_VERSION" != "xenial" ]; then
    echo -e "${RED}Unsupported Ubuntu version. Exiting.${NC}"
    exit 1
fi

echo -e "${CYAN}Adding ROS 2 apt repository...${NC}"
# Add ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $UBUNTU_VERSION main" > /etc/apt/sources.list.d/ros2-latest.list'

echo -e "${CYAN}Downloading ROS 2...${NC}"
# Download ROS 2
ROS2_VERSION="eloquent" # Change this if using a different version
mkdir -p ~/ros2_$ROS2_VERSION
cd ~/ros2_$ROS2_VERSION
ROS2_PACKAGE_URL="https://github.com/ros2/ros2/releases/download/$ROS2_VERSION/ros2-$ROS2_VERSION-linux-x86_64.tar.bz2"
curl -L $ROS2_PACKAGE_URL -o ros2-$ROS2_VERSION-linux-x86_64.tar.bz2
tar xf ros2-$ROS2_VERSION-linux-x86_64.tar.bz2

echo -e "${CYAN}Installing and initializing rosdep...${NC}"
# Install and initialize rosdep
sudo apt update
sudo apt install -y python-rosdep
sudo rosdep init
rosdep update

echo -e "${CYAN}Installing missing dependencies...${NC}"
# Install missing dependencies
rosdep install --from-paths ros2-linux/share --ignore-src --rosdistro $ROS2_VERSION -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 libopensplice69 osrf_testing_tools_cpp poco_vendor rmw_connext_cpp rosidl_typesupport_connext_c rosidl_typesupport_connext_cpp rti-connext-dds-5.3.1 tinyxml_vendor tinyxml2_vendor urdfdom urdfdom_headers"

echo -e "${CYAN}Installing python3 libraries...${NC}"
# Install python3 libraries
sudo apt install -y libpython3-dev python3-pip
pip3 install -U argcomplete

echo -e "${CYAN}Setting up environment...${NC}"
# Environment setup
source ~/ros2_$ROS2_VERSION/ros2-linux/setup.bash

echo -e "${YELLOW}-----------------------------------"
echo "Test examples:${NC}"
echo -e "${YELLOW}-----------------------------------${NC}"
echo -e "${GREEN}In one terminal, run a C++ talker:${NC}"
echo -e "${GREEN}. ~/ros2_$ROS2_VERSION/ros2-linux/setup.bash${NC}"
echo -e "${GREEN}ros2 run demo_nodes_cpp talker${NC}"

echo -e "${GREEN}In another terminal, run a Python listener:${NC}"
echo -e "${GREEN}. ~/ros2_$ROS2_VERSION/ros2-linux/setup.bash${NC}"
echo -e "${GREEN}ros2 run demo_nodes_py listener${NC}"

echo -e "${NC} "
echo -e "${GREEN}ROS 2 Eloquent installation on Jetson completed.${NC}"
