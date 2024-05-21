# ROS 2 Installation Guide for Jetson Nano

This guide provides step-by-step instructions to install ROS 2 on the Jetson Nano development board running XUbuntu 20.

## Prerequisites

- Jetson Nano Development Kit
- XUbuntu 20 image for Jetson Nano
- SD card
- Computer with internet access
- balenaEtcher software for flashing the SD card

## Installation Steps

### 1. Install XUbuntu 20 Image

- Download the XUbuntu 20 image for Jetson Nano from [here](https://forums.developer.nvidia.com/t/xubuntu-20-04-focal-fossa-l4t-r32-3-1-custom-image-for-the-jetson-nano/121768).
- Flash the image onto the SD card using [balenaEtcher](https://github.com/balena-io/etcher/releases/download/v1.18.11/balenaEtcher-1.18.11-x64.AppImage).

### 2. Flash the Image

- Launch balenaEtcher.
- Select the XUbuntu image file.
- Choose the target device (SD card).
- Click on 'Flash' to begin the flashing process.
- Wait for the process to complete (approximately 30 minutes).

### 3. Extract XUbuntu Image

- Once the flashing is complete, extract the downloaded zip file.
- Open the XUbuntu image file.
- Extract the contents using the following command in the terminal:

  ```sh
  tar -xvjf Xubuntu-20.04-l4t-r32.3.1.tar.tbz2
  ```

### 4. ROS 2 Installation

- After booting into XUbuntu, open a terminal.
- Follow the ROS 2 installation commands provided in the [official documentation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html).
- Alternatively, use the following commands:

  ```bash
  # Set Locale
  locale 
  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  locale

  # Setup Sources
  apt-cache policy | grep universe 
  sudo apt install software-properties-common
  sudo add-apt-repository universe
  sudo apt update && sudo apt install curl gnupg2 lsb-release
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

  # Installing ROS packages
  sudo apt update
  sudo apt upgrade
  sudo apt install ros-foxy-desktop
  sudo apt install ros-foxy-ros-base

  # Environment Setup
  source /opt/ros/foxy/setup.bash
  ```

### 5. Verify Installation

Sure, here's an example of how you can create dummy ROS 2 nodes to test your installation:

#### Publisher Node

- Create a publisher node that publishes dummy messages to a topic.

    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    import time

    class PublisherNode(Node):
        def __init__(self):
            super().__init__('publisher_node')
            self.publisher_ = self.create_publisher(String, 'test_topic', 10)
            self.timer_ = self.create_timer(1, self.publish_message)
            self.counter_ = 0

        def publish_message(self):
            msg = String()
            msg.data = 'Hello ROS 2! Count: {}'.format(self.counter_)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: {}'.format(msg.data))
            self.counter_ += 1

    def main(args=None):
        rclpy.init(args=args)
        publisher_node = PublisherNode()
        rclpy.spin(publisher_node)
        publisher_node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

#### Subscriber Node

- Create a subscriber node that subscribes to the same topic and prints received messages.

    ```python
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SubscriberNode(Node):
        def __init__(self):
            super().__init__('subscriber_node')
            self.subscription_ = self.create_subscription(String, 'test_topic', self.callback, 10)

        def callback(self, msg):
            self.get_logger().info('Received: {}'.format(msg.data))

    def main(args=None):
        rclpy.init(args=args)
        subscriber_node = SubscriberNode()
        rclpy.spin(subscriber_node)
        subscriber_node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()
    ```

### Running the Nodes

1. Open two terminal windows.
2. In the first terminal, run the publisher node:

   ```bash
   ros2 run <package_name> publisher_node
   ```

   Replace `<package_name>` with the actual name of your ROS 2 package containing the `publisher_node.py` script.

3. In the second terminal, run the subscriber node:

   ```bash
   ros2 run <package_name> subscriber_node
   ```

   Replace `<package_name>` with the actual name of your ROS 2 package containing the `subscriber_node.py` script.

    Make sure you have sourced the ROS 2 environment before running these commands:

    ```bash
    source /opt/ros/foxy/setup.bash
    ```
    This will ensure that the ROS 2 command line tools are available in your terminal session.You should see the publisher node publishing messages and the subscriber node receiving and printing them. This verifies that your ROS 2 installation is working correctly.
