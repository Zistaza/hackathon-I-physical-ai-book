#!/usr/bin/env python3

"""
Hello World Node Example
This is a basic ROS 2 node that demonstrates the fundamental concepts
of ROS 2 programming for the ROS 2 for Humanoid Robotics educational module.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloWorldNode(Node):
    """
    A simple ROS 2 node that publishes a 'Hello World' message periodically
    and subscribes to messages to demonstrate basic communication.
    """

    def __init__(self):
        # Initialize the node with a name
        super().__init__('hello_world_node')

        # Create a publisher for the 'hello_world' topic
        self.publisher = self.create_publisher(String, 'hello_world', 10)

        # Create a subscription to the 'hello_world_reply' topic
        self.subscription = self.create_subscription(
            String,
            'hello_world_reply',
            self.listener_callback,
            10
        )

        # Set a timer to publish messages every 2 seconds
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Counter to keep track of published messages
        self.i = 0

        # Log that the node has started
        self.get_logger().info('Hello World Node has started')

    def timer_callback(self):
        """Callback function for the timer that publishes messages"""
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

    def listener_callback(self, msg):
        """Callback function for the subscription that receives messages"""
        self.get_logger().info(f'Received: "{msg.data}"')


def main(args=None):
    """Main function to run the Hello World node"""
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the node
    hello_world_node = HelloWorldNode()

    try:
        # Spin the node to process callbacks
        rclpy.spin(hello_world_node)
    except KeyboardInterrupt:
        hello_world_node.get_logger().info('Node interrupted by user')
    finally:
        # Destroy the node explicitly
        hello_world_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()