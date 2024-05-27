# ROS2 Topics

## Conceptual Overview

A ROS 2 topic facilitates message passing between nodes in a publish/subscribe model. Nodes can publish messages to a topic or subscribe to receive messages from a topic. This decouples the producer and consumer of information, enhancing modularity and scalability in robotic systems.

## Real-Life Analogy

Consider a radio station (publisher) broadcasting on a specific frequency (topic). Listeners (subscribers) tune in to that frequency to receive the broadcast. Similarly, in ROS 2, a node publishes messages on a topic, and other nodes subscribe to this topic to receive the messages.

### Writing Your Own Topic (Publisher/Subscriber)

## In Python

1. **Publisher Node**:

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
           msg.data = 'Hello, ROS 2!'
           self.publisher_.publish(msg)
           self.get_logger().info('Publishing: "%s"' % msg.data)

   def main(args=None):
       rclpy.init(args=args)
       minimal_publisher = MinimalPublisher()
       rclpy.spin(minimal_publisher)
       minimal_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
2. **Subscriber Node**:

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

## In C++

1. **Publisher Node**:

   ```cpp
   #include <chrono>
   #include <memory>
   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   using namespace std::chrono_literals;

   class MinimalPublisher : public rclcpp::Node {
   public:
       MinimalPublisher() : Node("minimal_publisher"), count_(0) {
           publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
           timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
       }

   private:
       void timer_callback() {
           auto message = std_msgs::msg::String();
           message.data = "Hello, ROS 2: " + std::to_string(count_++);
           RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
           publisher_->publish(message);
       }

       rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
       rclcpp::TimerBase::SharedPtr timer_;
       size_t count_;
   };

   int main(int argc, char *argv[]) {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<MinimalPublisher>());
       rclcpp::shutdown();
       return 0;
   }
   ```
2. **Subscriber Node**:

   ```cpp
   #include <memory>
   #include "rclcpp/rclcpp.hpp"
   #include "std_msgs/msg/string.hpp"

   class MinimalSubscriber : public rclcpp::Node {
   public:
       MinimalSubscriber() : Node("minimal_subscriber") {
           subscription_ = this->create_subscription<std_msgs::msg::String>(
               "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
       }

   private:
       void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
           RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
       }

       rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
   };

   int main(int argc, char *argv[]) {
       rclcpp::init(argc, argv);
       rclcpp::spin(std::make_shared<MinimalSubscriber>());
       rclcpp::shutdown();
       return 0;
   }
   ```

### Debugging Topics with ROS 2 Tools

1. **Inspecting Active Topics**:

   ```bash
   ros2 topic list
   ```
2. **Displaying Topic Information**:

   ```bash
   ros2 topic info /topic
   ```
3. **Echoing Topic Messages**:

   ```bash
   ros2 topic echo /topic
   ```
4. **Visualizing Data in RQT**:

   ```bash
   rqt
   ```

### Experimenting with Turtlesim

1. **Installing Turtlesim**:

   ```bash
   sudo apt update
   sudo apt install ros-foxy-turtlesim
   ```
2. **Launching Turtlesim**:

   ```bash
   ros2 run turtlesim turtlesim_node
   ```
3. **Controlling Turtlesim with Topics**:

   ```bash
   ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
   ```

### Practical Activity

To consolidate your knowledge, create a publisher and subscriber pair where the publisher sends randomly generated numbers and the subscriber calculates and logs their average. Use either Python or C++ based on your preference.

1. **Publisher Node** (Python):

   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Float64
   import random

   class NumberPublisher(Node):
       def __init__(self):
           super().__init__('number_publisher')
           self.publisher_ = self.create_publisher(Float64, 'numbers', 10)
           timer_period = 1.0  # seconds
           self.timer = self.create_timer(timer_period, self.timer_callback)

       def timer_callback(self):
           num = Float64()
           num.data = random.uniform(0, 100)
           self.publisher_.publish(num)
           self.get_logger().info('Publishing: "%f"' % num.data)

   def main(args=None):
       rclpy.init(args=args)
       number_publisher = NumberPublisher()
       rclpy.spin(number_publisher)
       number_publisher.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```
2. **Subscriber Node** (Python):

   ```python
   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import Float64

   class NumberSubscriber(Node):
       def __init__(self):
           super().__init__('number_subscriber')
           self.subscription = self.create_subscription(
               Float64,
               'numbers',
               self.listener_callback,
               10)
           self.subscription  # prevent unused variable warning
           self.numbers = []

       def listener_callback(self, msg):
           self.numbers.append(msg.data)
           avg = sum(self.numbers) / len(self.numbers)
           self.get_logger().info('I heard: "%f" - Average: "%f"' % (msg.data, avg))

   def main(args=None):
       rclpy.init(args=args)
       number_subscriber = NumberSubscriber()
       rclpy.spin(number_subscriber)
       number_subscriber.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

### Conclusion

By completing this tutorial, you have gained a comprehensive understanding of ROS 2 topics and their implementation in Python and C++. You have also learned how to debug topics using ROS 2 tools and experimented with Turtlesim. The practical activity provided an opportunity to apply these concepts, reinforcing your understanding of inter-node communication in ROS 2.
