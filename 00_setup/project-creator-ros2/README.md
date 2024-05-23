# ROS2 Project

This repository serves as the foundation for a ROS2 project, inspired by the [joshnewans](https://github.com/joshnewans/my_bot). It provides a well-organized folder structure and essential files.

## Project Structure

The project structure is thoughtfully organized:

- **config**: Holds configuration files.
- **description**: Contains files pertinent to the robot's description.
- **launch**: Encompasses launch files for initiating ROS2 nodes.
- **utils**: Hosts utility scripts and tools.
- **worlds**: Stores simulation world files.

At the project's root, find crucial files:

- **CMakeLists.txt**: Configures the CMake build system.
- **LICENSE.md**: Prescribes the project's license.
- **package.xml**: Supplies ROS package information.
- **README.md**: This very document offering project insights.

## Template Files

The `templates` folder provides invaluable starting points, directly borrowed from the [joshnewans](https://github.com/joshnewans/my_bot):

- **CMakeLists.txt**: CMake configuration for building.
- **empty.world**: Blank template for simulation worlds.
- **empty.yaml**: Skeleton YAML file for configuration.
- **LICENSE.md**: Template license for your project.
- **package.xml**: Template ROS package information.
- **robot.urdf.xacro**: Xacro file for robot description.
- **rsp.launch.py**: ROS2 launch file.
- **README.md**: Template README file for documentation.

## Python Script

The create_ros2_project.py Python script has the purpose of creating the initial structure for a ROS2 project based on predefined templates. Here are some of the main features and reasons for using this script:

### Key Features:

1. Create Predefined Structure: The script automatically generates the predefined folder structure for a ROS2 project, including subfolders for configurations, descriptions, launch files, utilities, and simulations.
2. Insert Template Files: It copies predefined template files into the corresponding folders. These templates include basic configurations such as CMake lists, license information, ROS package descriptions, robot URDF descriptions, and launch scripts.
3. Easy to Use: Users only need to execute the script to generate the project structure and files. This greatly simplifies setting up a new ROS2 project and ensures consistency in project configuration.

Customization: Users can customize the script as needed, such as including additional files or folders in the templates.

## Python Script

Use the `create_ros2_project.py` Python script to seamlessly integrate template files into your project structure. The script accomplishes the following:

1. Copies template files into their designated folders.
2. Offers detailed logging messages for transparency.

### Usage

Run the script using Python:

```bash
python create_ros2_project.py
```

Feel free to adapt the `root_path` variable within the script to match your desired project location. For any inquiries or improvements, don't hesitate to reach out!
