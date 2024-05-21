#### Introduction to ROS 2 Topics

In the context of Robot Operating System (ROS) 2, a *topic* serves as a named bus over which nodes exchange messages. This mechanism allows nodes to communicate in a decoupled and scalable manner, facilitating the development of complex robotic systems. To grasp the concept of topics, we will draw an analogy with a real-life example involving a radio transmitter and receiver.

#### Analogy: Radio Transmitter and Receiver

Consider a radio transmitter broadcasting music on a specific frequency, say 98.7 FM. For simplicity, let's equate the frequency 98.7 to a topic name in ROS 2. Here’s how the analogy unfolds:

1. **Radio Transmitter (Publisher)**: The radio transmitter sends out music data on the 98.7 frequency. In ROS 2, this transmitter acts as a publisher, publishing data on the 98.7 topic.
2. **Radio Receiver (Subscriber)**: A radio receiver (such as a smartphone or car radio) tuned to 98.7 receives the broadcast music. This receiver is a subscriber in ROS 2, subscribing to the 98.7 topic to receive data.

In this analogy:

- **Publisher**: The entity that sends data.
- **Subscriber**: The entity that receives data.
- **Topic**: The channel (frequency) on which data is sent.

#### Key Characteristics of ROS 2 Topics

1. **Uni-directional Communication**: Topics facilitate unidirectional data streams from publishers to subscribers. There is no feedback from subscribers to publishers.
2. **Anonymity**: Publishers and subscribers remain anonymous to each other. A publisher does not know which nodes are subscribed to its topic, and a subscriber does not know which nodes are publishing to the topic.
3. **Message Type Consistency**: All publishers and subscribers of a topic must use the same message type. This ensures compatibility and successful communication.

#### Multiple Publishers and Subscribers

In ROS 2, a topic can have multiple publishers and subscribers:

- **Multiple Publishers**: Different nodes can publish to the same topic. For instance, if two radio transmitters broadcast on 98.7, a receiver tuned to 98.7 will receive data from both. This can cause interference, which is managed differently in ROS 2 compared to radio broadcasting.
- **Multiple Subscribers**: Many nodes can subscribe to the same topic. Each subscriber independently receives the data stream.

#### Nodes and Topics

A ROS 2 node can contain multiple publishers and subscribers. For example:

- A car's node might subscribe to the 98.7 topic for music and publish its location to a different topic, say `car_location`.
- Another node, perhaps a central server, can subscribe to the `car_location` topic to track the car’s position.

#### Creating and Using Topics in ROS 2

**Steps to Create a Publisher and Subscriber in ROS 2:**

1. **Define the Topic Name and Message Type**: Choose a descriptive name for the topic and ensure all interacting nodes use the same message type.
2. **Create the Publisher**:

   - In Python (using `rclpy`):
     ```python
     import rclpy
     from rclpy.node import Node
     from std_msgs.msg import String

     class Talker(Node):
         def __init__(self):
             super().__init__('talker')
             self.publisher_ = self.create_publisher(String, 'chatter', 10)
             timer_period = 0.5
             self.timer = self.create_timer(timer_period, self.timer_callback)

         def timer_callback(self):
             msg = String()
             msg.data = 'Hello, world!'
             self.publisher_.publish(msg)
             self.get_logger().info(f'Publishing: "{msg.data}"')

     def main(args=None):
         rclpy.init(args=args)
         node = Talker()
         rclpy.spin(node)
         node.destroy_node()
         rclpy.shutdown()

     if __name__ == '__main__':
         main()
     ```
   - In C++ (using `rclcpp`):
     ```cpp
     #include <rclcpp/rclcpp.hpp>
     #include <std_msgs/msg/string.hpp>

     class Talker : public rclcpp::Node
     {
     public:
         Talker() : Node("talker")
         {
             publisher_ = this->create_publisher<std_msgs::msg::String>("chatter", 10);
             timer_ = this->create_wall_timer(
                 500ms, std::bind(&Talker::timer_callback, this));
         }

     private:
         void timer_callback()
         {
             auto message = std_msgs::msg::String();
             message.data = "Hello, world!";
             RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
             publisher_->publish(message);
         }
         rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
         rclcpp::TimerBase::SharedPtr timer_;
     };

     int main(int argc, char *argv[])
     {
         rclcpp::init(argc, argv);
         rclcpp::spin(std::make_shared<Talker>());
         rclcpp::shutdown();
         return 0;
     }
     ```
3. **Create the Subscriber**:

   - In Python (using `rclpy`):
     ```python
     import rclpy
     from rclpy.node import Node
     from std_msgs.msg import String

     class Listener(Node):
         def __init__(self):
             super().__init__('listener')
             self.subscription = self.create_subscription(
                 String,
                 'chatter',
                 self.listener_callback,
                 10)

         def listener_callback(self, msg):
             self.get_logger().info(f'I heard: "{msg.data}"')

     def main(args=None):
         rclpy.init(args=args)
         node = Listener()
         rclpy.spin(node)
         node.destroy_node()
         rclpy.shutdown()

     if __name__ == '__main__':
         main()
     ```
   - In C++ (using `rclcpp`):
     ```cpp
     #include <rclcpp/rclcpp.hpp>
     #include <std_msgs/msg/string.hpp>

     class Listener : public rclcpp::Node
     {
     public:
         Listener() : Node("listener")
         {
             subscription_ = this->create_subscription<std_msgs::msg::String>(
                 "chatter", 10, std::bind(&Listener::topic_callback, this, std::placeholders::_1));
         }

     private:
         void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
         {
             RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
         }
         rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
     };

     int main(int argc, char *argv[])
     {
         rclcpp::init(argc, argv);
         rclcpp::spin(std::make_shared<Listener>());
         rclcpp::shutdown();
         return 0;
     }
     ```

#### Naming Conventions and Best Practices

- **Topic Naming**: Topic names must start with a letter and can include letters, numbers, underscores (`_`), tildes (`~`), and slashes (`/`).
- **Message Types**: Ensure that all nodes publishing or subscribing to a topic agree on the message type.
- **Decoupling**: Use topics to decouple nodes, allowing them to operate independently and facilitating easier testing and scaling.

#### Conclusion

ROS 2 topics provide a robust framework for node communication in robotic systems. By understanding the principles of publishers, subscribers, and message types, developers can design modular and scalable robotic applications. The use of topics enhances the flexibility and maintainability of code, ensuring efficient data exchange across different components of a robotic system.
