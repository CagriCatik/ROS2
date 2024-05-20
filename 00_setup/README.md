
# ROS2 Installation and Package Setup

This repository is designed to expedite and simplify the ROS2 distribution setup process, harnessing time-saving benefits and customization possibilities. It includes scripts and instructions specifically crafted for an efficient ROS2 setup, aiming to streamline installation and configuration tasks.

## Installation Script

The `install_ros2_<distribution>.sh` script automates the entire ROS2 Galactic installation process, ensuring the environment is correctly configured for seamless ROS2 development.

### Why Use This Script?

- **Time Efficiency:** This script automates multiple installation steps, reducing manual effort and accelerating the ROS2 setup.
- **Ease of Use:** Simplifies the ROS2 installation and package setup, making it accessible even for those less familiar with detailed procedures.
- **Customization:** The script can be tailored to fit specific project requirements or preferences.

### Usage

Execute the following command to run the installation script, replacing `<distribution>` with the desired ROS2 version (e.g., `galactic`):

```bash
sudo ./install_ros2_<distribution>.sh
```

Ensure you have root privileges to execute the script.

### Features

- **UTF-8 Locale:** Sets up UTF-8 locale for compatibility.
- **Repository Addition:** Adds the ROS2 repository and updates package lists.
- **Tool Installation:** Installs essential build tools, ROS2 Galactic Desktop version, and Python packages for testing.
- **Environment Configuration:** Sources the ROS2 setup script to configure the development environment.

## Usage

1. Clone this repository to your local machine:

   ```bash
   git clone <repository_url>
   ```
2. Navigate to the cloned directory:

   ```bash
   cd ros2-setup
   ```
3. Run the installation script:

   ```bash
   chmod +x install_ros2_<distro>.sh
   sudo ./install_ros2_<distribution>.sh
   ```
4. Run the package script:

   ```bash
   chmod +x install_ros2_packages.sh
   sudo ./install_ros2_packages.sh
   ```
5. Install the required Python packages:

   ```bash
   pip install -r requirements.txt
   ```

## Acknowledgments

- The ROS2 installation script is adapted from the [ROS2 Documentation](https://index.ros.org/doc/ros2/Installation/Linux-Install-Debians/) by Open Robotics.

Feel free to contribute, modify, or provide feedback to enhance the scripts and improve the overall ROS2 setup experience.

---
