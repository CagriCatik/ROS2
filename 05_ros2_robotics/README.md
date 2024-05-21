# ROS2: Robotics

This guide is structured to provide you with a detailed and practical approach to creating and managing robots using ROS2, offering step-by-step instructions and insights into various aspects of robotic development.

## Custom Robot Creation

In this section, we delve into the fundamentals of creating a custom robot from scratch. You'll learn about the essential workflows, setting up your workspace, and creating packages. We will guide you through the process of defining your robot using URDF (Unified Robot Description Format) and building a functional robot model.

1. [Introduction](01_intro.md) - An overview of the concepts and goals of custom robot creation.
2. [Workflow](02_workflow.md) - Detailed explanation of the development workflow in ROS2.
3. [Creating Workspace](03_create-ws.md) - Steps to set up your ROS2 workspace.
4. [Creating Package](04_creating-package.md) - How to create and organize packages in ROS2.
5. [URDF Robot](05_urdf-robot.md) - Introduction to URDF and how to define your robot.
6. [URDF Tree](06_urdf-tree.md) - Understanding the URDF tree structure.
7. [Rover URDF](07_rover-urdf.md) - Creating a URDF model for a rover.
8. [Launch Files](08_launch-files.md) - How to create and use launch files in ROS2.
9. [Gazebo](09_gazebo.md) - Introduction to Gazebo simulation with ROS2.
10. [Gazebo Plugins](10_gazebo-plugins.md) - Utilizing plugins in Gazebo for extended functionalities.
11. [Differential Drive](11_differential-drive.md) - Implementing differential drive for your robot.
12. [Camera and Lidar](12_camera-lidar.md) - Integrating camera and Lidar sensors with your robot.

## Rover Custom Robot

This section focuses on a specific example of a custom robot: a rover designed for obstacle avoidance and navigation. It covers the use of Lidar for perception and algorithms for obstacle avoidance and wall-following behaviors.

1. [Introduction](01_intro.md) - Overview of the rover project and its objectives.
2. [Avoid Obstacles](02_avoid-obstacles.md) - Techniques and strategies for obstacle avoidance.
3. [Lidar Plugin](03_lidar-plugin.md) - Setting up and using the Lidar plugin in your rover.
4. [Lidar Data Manipulation](04_lidar-data-manipulation.md) - Processing Lidar data for navigation.
5. [Obstacle Avoiding](05_obstacle-avoiding.md) - Implementing obstacle avoidance behavior.
6. [Follow Wall](06_follow-wall.md) - Methods for wall-following navigation.
7. [Following Wall](07_following-wall.md) - Practical application of wall-following algorithms.

## Toyota Prius CV Lane Following

In this section, we explore the application of computer vision in autonomous driving using a Toyota Prius model. You'll learn about obtaining the car model, creating the necessary packages, and using computer vision techniques for lane following.

1. [Prius Car](01_prius-car.md) - Introduction to the Prius car model.
2. [Obtain Model](02_obtain-model.md) - Steps to obtain and integrate the Prius car model.
3. [Create Package](03_create-package.md) - Setting up the package for the Prius project.
4. [World Files](04_world-files.md) - Creating and configuring world files for simulation.
5. [Camera Node](05_camera-node.md) - Setting up the camera node for image capture.
6. [CV Image Segmentation](06_cv-image-segmentation.md) - Techniques for image segmentation using computer vision.
7. [CV Boundary Extraction](07_cv-boundary-extraction.md) - Extracting boundaries from segmented images.
8. [CV Frame Car Mid-Point](08_cv-frame-car-mid-point.md) - Calculating the car's mid-point using computer vision.
9. [Programming Mid-Point](09_programming-mid-point.md) - Implementing mid-point calculations in your program.
10. [Control Algorithm Output](10_control-algorithm-output.md) - Developing and testing the control algorithm for lane following.

This guide aims to equip you with the knowledge and tools needed to build sophisticated robotic systems using ROS2. Each section is designed to be comprehensive and practical, ensuring you gain hands-on experience and a deep understanding of the topics covered.