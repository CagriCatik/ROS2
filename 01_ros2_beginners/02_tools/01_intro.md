# ROS2 Tools

## Overview

In this section, we will delve into some of the most essential tools available in the ROS2 ecosystem. These tools are indispensable for debugging nodes and ROS2 applications. By exploring these utilities, you will enhance your understanding and capability to manage and troubleshoot your ROS2 projects effectively.

## Objectives

By the end of this section, you will be proficient in:

1. Running ROS2 nodes with various options.
2. Debugging nodes using the ROS2 Command Line Interface (CLI).
3. Visualizing the graph of your ROS2 application using `rqt_graph`.
4. Exploring Turtlesim, a basic 2D simulation tool.

## Running Nodes with `ros2 run`

The `ros2 run` command is fundamental for launching nodes. It allows you to execute nodes with different configurations, such as changing the node's name dynamically. Here is a step-by-step guide on how to use `ros2 run`:

1. **Basic Node Execution:**

   ```bash
   ros2 run <package_name> <node_executable>
   ```

   Example:

   ```bash
   ros2 run demo_nodes_cpp talker
   ```

   This command runs the `talker` node from the `demo_nodes_cpp` package.
2. **Running with a Custom Node Name:**
   You can change the node's name using the `--ros-args` flag followed by the `-r` option:

   ```bash
   ros2 run <package_name> <node_executable> --ros-args -r __node:=custom_node_name
   ```

   Example:

   ```bash
   ros2 run demo_nodes_cpp talker --ros-args -r __node:=my_custom_talker
   ```

## Debugging Nodes with ROS2 CLI

The ROS2 Command Line Interface (CLI) offers a suite of commands to interact with and debug nodes. Here are some crucial commands:

1. **Listing Active Nodes:**

   ```bash
   ros2 node list
   ```

   This command displays all currently running nodes.
2. **Getting Node Information:**

   ```bash
   ros2 node info <node_name>
   ```

   Example:

   ```bash
   ros2 node info /my_custom_talker
   ```

   This provides detailed information about the specified node, such as its publishers, subscribers, services, and parameters.
3. **Introspecting Topics:**

   - **Listing Topics:**

     ```bash
     ros2 topic list
     ```
   - **Displaying Topic Information:**

     ```bash
     ros2 topic info <topic_name>
     ```

     Example:
     ```bash
     ros2 topic info /chatter
     ```
   - **Echoing Topic Data:**

     ```bash
     ros2 topic echo <topic_name>
     ```

     Example:
     ```bash
     ros2 topic echo /chatter
     ```
4. **Service Introspection:**

   - **Listing Services:**
     ```bash
     ros2 service list
     ```
   - **Displaying Service Information:**
     ```bash
     ros2 service info <service_name>
     ```

## Visualizing the ROS2 Graph with `rqt_graph`

The `rqt_graph` tool provides a graphical representation of the nodes and topics in your ROS2 application, facilitating a comprehensive overview of your system's structure and communication flow.

1. **Launching `rqt_graph`:**

   ```bash
   ros2 run rqt_graph rqt_graph
   ```

   This opens the `rqt_graph` interface, displaying nodes and their connections.
2. **Interpreting the Graph:**

   - **Nodes** are represented as rectangles.
   - **Topics** are shown as ellipses.
   - **Connections** between nodes and topics indicate the flow of messages.

## Exploring Turtlesim

Turtlesim is a simple 2D simulation tool often used for introductory ROS2 tutorials. It helps users understand basic ROS2 concepts in a visual and interactive way.

1. **Launching Turtlesim:**

   ```bash
   ros2 run turtlesim turtlesim_node
   ```
2. **Controlling Turtlesim:**

   - **Starting the Control Node:**
     ```bash
     ros2 run turtlesim turtle_teleop_key
     ```
   - Use the keyboard to control the turtle's movement within the simulation window.

## Practical Activity

As a hands-on exercise, create a new ROS2 package, write a simple publisher and subscriber node, and use the tools discussed to debug and visualize your application.

1. **Creating a Package:**

   ```bash
   ros2 pkg create --build-type ament_cmake my_package
   ```
2. **Writing the Publisher (Python):**

   - Create `publisher_node.py` in `my_package/my_package` directory.
   - Add the following code:
     ```python
     import rclpy
     from rclpy.node import Node
     from std_msgs.msg import String

     class MinimalPublisher(Node):
         def __init__(self):
             super().__init__('minimal_publisher')
             self.publisher_ = self.create_publisher(String, 'topic', 10)
             timer_period = 0.5  # seconds
             self.timer = self.create_timer(timer_period, self.timer_callback)

         def timer_callback(self):
             msg = String()
             msg.data = 'Hello, world!'
             self.publisher_.publish(msg)

     def main(args=None):
         rclpy.init(args=args)
         minimal_publisher = MinimalPublisher()
         rclpy.spin(minimal_publisher)
         minimal_publisher.destroy_node()
         rclpy.shutdown()

     if __name__ == '__main__':
         main()
     ```
3. **Writing the Subscriber (Python):**

   - Create `subscriber_node.py` in `my_package/my_package` directory.
   - Add the following code:
     ```python
     import rclpy
     from rclpy.node import Node
     from std_msgs.msg import String

     class MinimalSubscriber(Node):
         def __init__(self):
             super().__init__('minimal_subscriber')
             self.subscription = self.create_subscription(
                 String,
                 'topic',
                 self.listener_callback,
                 10)
             self.subscription  # prevent unused variable warning

         def listener_callback(self, msg):
             self.get_logger().info('I heard: "%s"' % msg.data)

     def main(args=None):
         rclpy.init(args=args)
         minimal_subscriber = MinimalSubscriber()
         rclpy.spin(minimal_subscriber)
         minimal_subscriber.destroy_node()
         rclpy.shutdown()

     if __name__ == '__main__':
         main()
     ```
4. **Building and Running the Package:**

   - Build the package:
     ```bash
     cd ~/ros2_ws
     colcon build
     ```
   - Source the setup file:
     ```bash
     source install/setup.bash
     ```
   - Run the publisher and subscriber:
     ```bash
     ros2 run my_package publisher_node
     ros2 run my_package subscriber_node
     ```
5. **Using the Tools:**

   - List and get information about the nodes:
     ```bash
     ros2 node list
     ros2 node info /minimal_publisher
     ros2 node info /minimal_subscriber
     ```
   - Visualize the communication graph:
     ```bash
     ros2 run rqt_graph rqt_graph
     ```

## Conclusion

Mastering these ROS2 tools will significantly enhance your ability to develop, debug, and maintain robust ROS2 applications. By understanding how to run nodes with different options, leverage the ROS2 CLI for debugging, visualize node interactions, and experiment with Turtlesim, you are well-equipped to tackle more complex ROS2 projects.
