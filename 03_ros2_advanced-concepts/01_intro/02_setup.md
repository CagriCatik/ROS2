# Installing and Setting Up ROS2 

In this tutorial, we will walk you through the process of installing and setting up ROS2 on your computer. We will also introduce some useful tools that will aid you during your ROS2 development journey.

## Prerequisites

- **Operating System:** Ubuntu (preferably 22.04 LTS)
- **Installation Mode:** Dual boot is recommended for the best performance, though you can start with a virtual machine if you're a beginner.

## Step-by-Step Installation

### 1. Install Ubuntu

To begin, ensure that Ubuntu is installed on your computer. While virtual machines are acceptable for initial learning, it is recommended to use a dual boot setup for serious development work.

### 2. Install ROS2 Humble

We will use ROS2 Humble in this tutorial. Follow these steps:

1. **Set Up Locale**

   ```bash
   sudo locale-gen en_US en_US.UTF-8
   sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
   export LANG=en_US.UTF-8
   ```
2. **Add ROS2 Sources**

   ```bash
   sudo apt update
   sudo apt install curl gnupg2 lsb-release
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
   sudo sh -c 'echo "deb [trusted=yes] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
   ```
3. **Install ROS2 Packages**
   Update your package index:

   ```bash
   sudo apt update
   ```

   Then, install the ROS2 desktop version:

   ```bash
   sudo apt install ros-humble-desktop
   ```

### 3. Set Up Environment

Add the following line to your `.bashrc` file to source the ROS2 setup script automatically:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Install Colcon

Colcon is the build tool for ROS2. Install it using:

```bash
sudo apt install python3-colcon-common-extensions
```

To enable autocompletion for Colcon, add the following to your `.bashrc`:

```bash
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
source ~/.bashrc
```

## Additional Tools

### Terminator

Terminator is a terminal emulator that allows you to split your terminal into multiple windows.

**Install Terminator:**

```bash
sudo apt install terminator
```

**Usage:**

- Split terminal horizontally: `Ctrl+Shift+O`
- Split terminal vertically: `Ctrl+Shift+E`

### Visual Studio Code

VS Code is a powerful text editor for coding.

**Install VS Code:**

```bash
sudo snap install code --classic
```

**Open a Folder in VS Code:**

```bash
code .
```

**Recommended Extensions:**

1. **ROS Extension** by Microsoft
2. **CMake Tools Extension** by twxs

To install these, go to the extensions view (`Ctrl+Shift+X`) in VS Code and search for them.

## Summary

1. **Install Ubuntu:** Preferably as a dual boot for better performance.
2. **Install ROS2 Humble:** Follow the steps to set up your locale, add sources, and install ROS2 packages.
3. **Set Up Environment:** Add ROS2 and Colcon sourcing to your `.bashrc`.
4. **Install Additional Tools:** Terminator for terminal management and VS Code with recommended extensions for development.

With these tools and configurations, you are well-equipped to start your ROS2 development journey. If you encounter any issues, refer to the official ROS2 documentation or community forums for help.
