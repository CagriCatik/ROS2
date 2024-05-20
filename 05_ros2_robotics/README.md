<div align="center">

  <img src="assets/img/logo.png" alt="logo" width="200" height="auto" />
  <h1>ROS2 Robotics for Beginners</h1>
  <p>
    Building Custom Robot, Nodes, Workspaces and Packages
  </p>

  
<!-- Badges -->
<p>
  <a href="https://github.com/CagriCatik/ros2robotics/graphs/contributors">
    <img src="https://img.shields.io/github/contributors/cagricatik/ros2robotics" alt="contributors" />
  </a>
  <a href="">
    <img src="https://img.shields.io/github/last-commit/CagriCatik/ros2robotics" alt="last update" />
  </a>
  <a href="https://github.com/CagriCatik/ros2robotics/network/members">
    <img src="https://img.shields.io/github/forks/CagriCatik/ros2robotics" alt="forks" />
  </a>
  <a href="https://github.com/CagriCatik/ros2robotics/stargazers">
    <img src="https://img.shields.io/github/stars/CagriCatik/ros2robotics" alt="stars" />
  </a>
  <a href="https://github.com/CagriCatik/ros2robotics/issues/">
    <img src="https://img.shields.io/github/issues/CagriCatik/ros2robotics" alt="open issues" />
  </a>
</p>
   
 </div>



<!-- Table of Contents -->
# Table of Contents

This repository covers all the new features of ROS 2 using various examples from different sources:

1. [Custom Robot Creation for ROS Projects](https://github.com/CagriCatik/pg-ros/tree/main/5_ROS2_Robotics/part1)
2. [Rover Custom Robot for Wall Following and Obstacle Avoiding](https://github.com/CagriCatik/pg-ros/tree/main/5_ROS2_Robotics/part2)
3. [Toyota Prius Computer Vision for Line Following](https://github.com/CagriCatik/pg-ros/tree/main/5_ROS2_Robotics/part3)


<!-- About the Project -->
## About this Repository

Some text

## Install ROS2 and Setup Your Environment

### Install development packages

- Different packages, which provide to development in ROS2
  
  ```sh
  chmod +x /install/requirements.sh
  bash install.sh
  ```

### Install ROS2 Galactic Geochelone on Ubuntu 20.04

- Run single line code for installation Galactic Geochelone

  ```sh
  chmod +x ros2beginners/install/ros2_install_galactic.sh
  bash ros2_install_galactic.sh
  ```

### Setup your Environment for ROS2

- Source the environment before you use ROS2 and set up it by sourcing the following echo command in bashrc so that bash runs whenever it is started interactively.

  ```sh
  echo 'source /opt/ros/galactic/setup.bash' >> ~/.bashrc 
  echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> ~/.bashrc 
  ```

## Configuring the ROS 2 environment and workspace

### Create a ROS2 workspace in your home directory

- Make a directory for source code and build it with colcon command
- In ROS2, no more catkin. Ament is the new building system, and on top of that you get the colcon command line tool.
- To compile, you’ll use the command “colcon build” in your ROS2 workspace.

  ```sh
  mkdir .p ~/beginners_ws/src
  cd ws
  colcon build
  cd install
  echo 'source ~/<>/ws/<>/install/local_setup.bash' >> ~/.bashrc 
  echo 'source ~/<>/ws/<>/install/setup.bash' >> ~/.bashrc 
  ```

### Create a Package inside this workspace

- Create Python package
  
  ```sh
  cd beginners_ws/src/
  # Create a package with ament_python and rclpy client library
  ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
  cd beginners_ws
  colcon build --packages-select my_py_pkg
  ```


## Acknowledgements

 - [ROS2 Robotics for Beginners](https://www.udemy.com/course/ros2-ultimate-mobile-robotics-course-for-beginners-opencv/)
 - [Readme Template](https://github.com/othneildrew/Best-README-Template)
