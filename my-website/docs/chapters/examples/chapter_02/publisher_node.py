#!/usr/bin/env python3

"""
Basic Publisher Node Example
This node demonstrates the publisher pattern in ROS 2 for the
ROS 2 for Humanoid Robotics educational module.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math
import time


class HumanoidPublisher(Node):
    """
    A publisher node that simulates humanoid robot sensor data.
    This implements the publisher pattern for Exercise 2.1.
    """

    def __init__(self):
        super().__init__('humanoid_publisher')

        # Create a publisher for basic string messages
        self.string_publisher = self.create_publisher(String, 'humanoid_status', 10)

        # Create a publisher for joint state messages (more relevant for humanoid robots)
        self.joint_publisher = self.create_publisher(JointState, 'joint_states', 10)

        # Timer to publish messages at 2 Hz
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_data)

        # Counter for published messages
        self.i = 0

        self.get_logger().info('Humanoid Publisher Node has started')

    def publish_data(self):
        """Publish both string status and joint state messages"""
        # Publish string status message
        status_msg = String()
        status_msg.data = f'Humanoid Robot Status: {self.i}'
        self.string_publisher.publish(status_msg)
        self.get_logger().info(f'Published status: "{status_msg.data}"')

        # Publish joint state message (simulating humanoid joint positions)
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.header.frame_id = 'base_link'

        # Simulate some joint names for a humanoid robot
        joint_names = [
            'left_hip_joint', 'left_knee_joint', 'left_ankle_joint',
            'right_hip_joint', 'right_knee_joint', 'right_ankle_joint',
            'left_shoulder_joint', 'left_elbow_joint',
            'right_shoulder_joint', 'right_elbow_joint'
        ]
        joint_msg.name = joint_names

        # Simulate joint positions (oscillating for demonstration)
        positions = []
        for j in range(len(joint_names)):
            # Create different oscillation patterns for each joint
            pos = math.sin((self.i * 0.1) + (j * 0.5)) * 0.5
            positions.append(pos)

        joint_msg.position = positions
        joint_msg.velocity = [0.0] * len(joint_names)  # zero velocity for simplicity
        joint_msg.effort = [0.0] * len(joint_names)    # zero effort for simplicity

        self.joint_publisher.publish(joint_msg)
        self.get_logger().info(f'Published joint states for {len(joint_names)} joints')

        self.i += 1


def main(args=None):
    """Main function to run the publisher node"""
    rclpy.init(args=args)

    humanoid_publisher = HumanoidPublisher()

    try:
        rclpy.spin(humanoid_publisher)
    except KeyboardInterrupt:
        humanoid_publisher.get_logger().info('Node interrupted by user')
    finally:
        humanoid_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()