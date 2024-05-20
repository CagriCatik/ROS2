#!/usr/bin/env bash
set -eu

# Colors for better output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Function to echo green-colored messages
print_success() {
    echo -e "${GREEN}$1${NC}"
}

# Function to echo red-colored messages
print_error() {
    echo -e "${RED}$1${NC}"
    exit 1
}

# Check if script is run as root
if [ "$EUID" -ne 0 ]; then
    print_error "Please run this script as root"
fi

# Update locale to support UTF-8
print_success "Setting up UTF-8 locale..."
locale-gen en_US en_US.UTF-8
update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 repository
print_success "Adding ROS2 repository..."
if [ ! -e /usr/share/keyrings/ros-archive-keyring.gpg ]; then
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg || print_error "Failed to download ROS2 repository key"
fi

source /etc/os-release
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null || print_error "Failed to add ROS2 repository"

# Update package lists
print_success "Updating package lists..."
apt update || print_error "Failed to update package lists"

# Install ROS2 packages
print_success "Installing ROS2 packages..."
apt install -y \
    build-essential \
    cmake \
    git \
    libbullet-dev \
    python3-colcon-common-extensions \
    python3-flake8 \
    python3-pip \
    python3-pytest-cov \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    wget || print_error "Failed to install ROS2 packages"

# Install ROS2 Galactic Desktop version
print_success "Installing ROS2 Galactic Desktop version..."
apt install -y ros-galactic-desktop || print_error "Failed to install ROS2 Galactic Desktop version"

# Install pip packages for testing
print_success "Installing pip packages for testing..."
python3 -m pip install -U \
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures \
    pytest || print_error "Failed to install pip packages"

# Source ROS2 setup script
print_success "Sourcing the setup script..."
echo "source /opt/ros/galactic/setup.bash" >> ~/.bashrc

# Upgrade packages
print_success "Upgrading packages..."
apt autoremove
apt-get upgrade

print_success "ROS2 Galactic installation completed successfully."
