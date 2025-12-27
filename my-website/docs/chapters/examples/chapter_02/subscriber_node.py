#!/usr/bin/env python3

"""
Basic Subscriber Node Example
This node demonstrates the subscriber pattern in ROS 2 for the
ROS 2 for Humanoid Robotics educational module.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import time


class HumanoidSubscriber(Node):
    """
    A subscriber node that receives and processes humanoid robot sensor data.
    This implements the subscriber pattern for Exercise 2.2.
    """

    def __init__(self):
        super().__init__('humanoid_subscriber')

        # Create a subscription for basic string messages
        self.string_subscription = self.create_subscription(
            String,
            'humanoid_status',
            self.string_listener_callback,
            10
        )
        self.string_subscription  # prevent unused variable warning

        # Create a subscription for joint state messages
        self.joint_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_listener_callback,
            10
        )
        self.joint_subscription  # prevent unused variable warning

        self.get_logger().info('Humanoid Subscriber Node has started')

    def string_listener_callback(self, msg):
        """Callback for string messages"""
        current_time = time.strftime("%H:%M:%S")
        self.get_logger().info(f'[{current_time}] Received status: "{msg.data}"')

    def joint_listener_callback(self, msg):
        """Callback for joint state messages"""
        current_time = time.strftime("%H:%M:%S")
        self.get_logger().info(
            f'[{current_time}] Received joint states: {len(msg.name)} joints, '
            f'first joint: {msg.name[0] if msg.name else "none"}'
        )

        # Log joint positions
        if msg.position:
            for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
                if i < 3:  # Only log first 3 joints to avoid spam
                    self.get_logger().info(f'  {name}: {pos:.3f} rad')
                elif i == 3:  # Add indicator if there are more joints
                    if len(msg.name) > 3:
                        self.get_logger().info(f'  ... and {len(msg.name) - 3} more joints')


def main(args=None):
    """Main function to run the subscriber node"""
    rclpy.init(args=args)

    humanoid_subscriber = HumanoidSubscriber()

    try:
        rclpy.spin(humanoid_subscriber)
    except KeyboardInterrupt:
        humanoid_subscriber.get_logger().info('Node interrupted by user')
    finally:
        humanoid_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()