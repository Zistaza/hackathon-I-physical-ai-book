# Chapter 2: Nodes & Topics for Humanoid Robotics

## Learning Objectives

By the end of this chapter, you will be able to:

- Create and manage ROS 2 nodes using rclpy
- Implement publisher/subscriber communication patterns
- Understand message types and custom message creation
- Design efficient topic-based communication for humanoid robots

## 1. Node Lifecycle Management

### Node initialization and destruction

In ROS 2, nodes are the fundamental building blocks of a robot application. Each node is responsible for a specific task and communicates with other nodes through topics, services, and actions.

To create a node in Python using rclpy, you need to:

1. Import the necessary modules
2. Create a class that inherits from `rclpy.node.Node`
3. Initialize the node with a unique name
4. Implement the node's functionality
5. Properly shut down the node when done

### Parameter handling and configuration

Nodes can accept parameters that allow them to be configured without recompilation. Parameters can be:

- Declared during node initialization
- Set from launch files
- Changed at runtime using the parameter service

### Error handling and recovery

Robust nodes should handle errors gracefully and implement recovery strategies when possible. This is especially important in humanoid robotics where failures could lead to physical damage or safety issues.

## 2. Publisher-Subscriber Pattern

### Creating publishers and subscribers

The publisher-subscriber pattern is the primary method of asynchronous communication in ROS 2. Publishers send messages to topics, and subscribers receive messages from topics.

### Message serialization and deserialization

ROS 2 handles the serialization and deserialization of messages automatically, but understanding the process helps with performance optimization and debugging.

### Quality of Service (QoS) configurations

QoS settings allow you to configure how messages are delivered between publishers and subscribers:

- **Reliability**: Whether messages are delivered reliably or on a best-effort basis
- **Durability**: Whether messages are stored for late-joining subscribers
- **History**: How many messages to store in the queue
- **Depth**: The size of the message queue

### Topic naming conventions for humanoid robotics

For humanoid robotics applications, it's important to follow consistent naming conventions:

- `/robot_name/joint_states` - for joint position, velocity, and effort
- `/robot_name/imu/data` - for inertial measurement unit data
- `/robot_name/sensors/` - for various sensor data
- `/robot_name/commands/` - for control commands

## 3. Message Types and Custom Messages

### Built-in message types

ROS 2 provides several standard message types:

- `std_msgs` - Basic data types (Int, Float, String, etc.)
- `geometry_msgs` - Geometric primitives (Point, Pose, Twist, etc.)
- `sensor_msgs` - Sensor data types (JointState, Imu, LaserScan, etc.)
- `nav_msgs` - Navigation-specific messages
- `trajectory_msgs` - Trajectory-related messages

### Creating custom message definitions

For humanoid robotics, you may need custom messages to represent specific data structures like:

- Custom joint configurations
- Humanoid-specific sensor data
- Complex control commands

### Message compatibility and versioning

Understanding message compatibility is crucial when working with distributed systems and long-running robots.

## 4. Advanced Topic Patterns

### Latching and persistent connections

Latching ensures that the last published message is sent to new subscribers immediately upon connection.

### Throttling and filtering

For high-frequency topics, it's often necessary to throttle or filter messages to reduce computational load.

### Multiple publisher/subscriber scenarios

Understanding how multiple publishers and subscribers interact on the same topic is important for complex humanoid robotics systems.

## Code Examples

### Basic publisher node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Basic subscriber node

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
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Hands-On Exercises

### Exercise 2.1: Create a publisher node that simulates sensor data from a humanoid robot

Create a publisher that simulates sensor data from a humanoid robot, such as joint positions, IMU data, or camera feeds.

### Exercise 2.2: Implement a subscriber that processes sensor data and logs it

Create a subscriber that receives the sensor data from Exercise 2.1 and logs it with timestamps for analysis.

### Exercise 2.3: Design custom messages for humanoid robot joint states

Design custom message definitions that extend the standard JointState message to include humanoid-specific information.

### Exercise 2.4: Implement multiple publishers and subscribers communicating via topics

Create a system with multiple publishers and subscribers to simulate a more complex humanoid robot communication network.

## Summary

This chapter covered the fundamentals of nodes and topics in ROS 2, with a focus on applications in humanoid robotics. We explored node lifecycle management, publisher-subscriber patterns, message types, and advanced communication patterns. These concepts form the foundation for more complex communication patterns like services and actions.

## References

- [ROS 2 Node Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Nodes.html)
- [ROS 2 Topics Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Topics.html)
- [Quality of Service in ROS 2](https://docs.ros.org/en/humble/Concepts/About-Quality-of-Service-Settings.html)