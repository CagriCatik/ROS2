#!/bin/bash

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}Adding ROS 2 repository for ARM64 architecture (Jetson)...${NC}"
sudo sh -c 'echo "deb [arch=arm64] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list'

echo -e "${CYAN}Setting up the ROS 2 key...${NC}"
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

echo -e "${CYAN}Updating package list...${NC}"
sudo apt-get update

echo -e "${CYAN}Installing ROS 2 Foxy Fitzroy Desktop...${NC}"
sudo apt-get install -y ros-foxy-desktop

echo -e "${CYAN}Sourcing the ROS 2 setup script...${NC}"
source /opt/ros/foxy/setup.bash

echo -e "${CYAN}Installing additional dependencies...${NC}"
sudo apt-get install -y python3-argcomplete python3-colcon-common-extensions

echo -e "${CYAN}Initializing rosdep...${NC}"
sudo rosdep init
rosdep update

echo -e "${CYAN}Adding ROS 2 setup script to bashrc...${NC}"
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

echo -e "${CYAN}Installing rosinstall and vcstool...${NC}"
sudo apt-get install -y python3-rosinstall python3-vcstool

# Test examples
echo -e "${YELLOW}-----------------------------------"
echo "Test examples:${NC}"
echo -e "${YELLOW}-----------------------------------${NC}"
echo -e "${GREEN}In one terminal, run a C++ talker:${NC}"
echo -e "${GREEN}. /opt/ros/foxy/setup.bash${NC}"
echo -e "${GREEN}ros2 run demo_nodes_cpp talker${NC}"

echo -e "${GREEN}In another terminal, run a Python listener:${NC}"
echo -e "${GREEN}. /opt/ros/foxy/setup.bash${NC}"
echo -e "${GREEN}ros2 run demo_nodes_py listener${NC}"

echo -e "${NC} "
echo -e "${GREEN}ROS 2 Foxy Fitzroy installation on Jetson completed!${NC}"
