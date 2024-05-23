# ROS2- TF|URDF|RViz|Gazebo

Welcome to the exploration of ROS2, where we delve into the intricacies of TF (Transforms), URDF (Unified Robot Description Format), RViz (ROS Visualization), and Gazebo simulation.

Throughout our journey, we'll uncover fundamental concepts and practical applications of ROS2, a prominent robotics middleware platform. Our discussions will span various topics, including visualizing robot transformations, crafting detailed robot descriptions using URDF, leveraging RViz for visualization and debugging purposes, and simulating robots in Gazebo for testing and development.

Whether you're a novice curious about robotics or a seasoned developer aiming to deepen your understanding of ROS2, our exploration offers a structured path. Each section provides clear explanations, hands-on exercises, and real-world examples to enhance your knowledge and skills.

Join us as we unravel the complexities of TF, construct intricate robot models with URDF, visualize robot configurations with RViz, and simulate robots within Gazebo's virtual environment. Let's embark on this enlightening journey into the realm of ROS2!

## Table of Contents

1. [Introduction](#introduction) 🛠️
2. [TF Overview](#tf-overview) 🌐
3. [URDF](#urdf) 🤖
4. [Broadcast TFs with the Robot State Publisher](#broadcast-tfs-with-the-robot-state-publisher) 📡
5. [Improve the URDF with Xacro](#improve-the-urdf-with-xacro) 🔧
6. [Simulate the Robot with Gazebo](#simulate-the-robot-with-gazebo) 🏗️
7. [Add a Sensor in Gazebo](#add-a-sensor-in-gazebo) 🎥

## 1. Introduction 🛠️

- Install and setup ROS2
- Install Quick Fix (Gazebo)
- Programming tools for this course

## 2. TF Overview 🌐

- Visualise a Robot TFs in RViz
- Relationship Between TFs, TF tree
- What problem are we trying to solve with TF?

## 3. URDF 🤖

- What is URDF
- First URDF file: Create and Visualize a Link
- Material - Add Some Colors
- Combine Links with a Joint
- Another example of the process to write the URDF right the first time
- Different Types of Joints in a URDF
- Add a Wheel to the Robot
- Activity - Complete the URDF for the Robot
- Activity - Solution

## 4. Broadcast TFs with the Robot State Publisher 📡

- How the Robot State Publisher and URDF Work Together
- Run the Robot State Publisher with URDF in the Terminal (Command Line)
- Create a Robot Description Package to Install the URDF
- Write a Launch file to Start the Robot State Publisher with URDF (XML)
- Python Launch File
- Activity - Add Rviz Config in the Launch File
- Activity - Solution

## 5. Improve the URDF with Xacro 🔧

- Make the URDF Compatible with Xacro
- Create Variables with Xacro Properties
- Activity - Xacro Properties
- Activity - Solution
- Create Functions with Xacro Macros
- Include a Xacro File in Another Xacro File
- The Xacro Command to Generate the URDF
- Real Meshes - Quick Overview

## 6. Simulate the Robot with Gazebo 🏗️

- Run Gazebo
- How Gazebo Works with ROS
- Add Inertia Tags in the URDF
- Activity - Inertia Macros
- Activity - Solution
- Add Collision Tags in the URDF
- Spawn the Robot in Gazebo
- Activity - Launch File to Start Robot in Gazebo
- Activity - Solution
- Fixing the Inertia Values
- Fixing the Colors with Gazebo Material
- Add a Gazebo Plugin to Control the Robot
- Create a World in Gazebo
- Launch the Robot in the World

## 7. Add a Sensor in Gazebo 🎥

- Add a Camera to the URDF
- Add a Gazebo Plugin for the Camera
- Quick Fix For the Camera to Work with ROS
- Intro - Final Project Overview
- Step - URDF Links and Joints
- Step - Adapt the Robot for Gazebo
- Step - Add Gazebo Plugins
- Step - Combine the Robots
