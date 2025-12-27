#!/usr/bin/env python3

"""
Common utility functions for ROS 2 examples
This module contains utility functions that are commonly used across
different ROS 2 examples in the humanoid robotics educational module.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String
import time
import sys
from typing import Optional


def initialize_ros2():
    """Initialize ROS 2 if not already initialized"""
    if not rclpy.ok():
        rclpy.init()


def create_common_qos_profile(depth: int = 10) -> QoSProfile:
    """
    Create a common QoS profile for examples

    Args:
        depth: History depth for the message queue

    Returns:
        QoSProfile: A standard QoS profile for examples
    """
    return QoSProfile(
        depth=depth,
        durability=QoSDurabilityPolicy.VOLATILE,
        reliability=QoSReliabilityPolicy.RELIABLE
    )


def wait_for_message(subscriber, timeout_sec: int = 5):
    """
    Helper function to wait for a message to be received

    Args:
        subscriber: The subscription to wait on
        timeout_sec: Maximum time to wait in seconds

    Returns:
        True if a message was received, False if timeout occurred
    """
    start_time = time.time()
    while rclpy.ok():
        rclpy.spin_once(subscriber, timeout_sec=0.1)
        if hasattr(subscriber, '_received_message') and subscriber._received_message:
            return True
        if time.time() - start_time > timeout_sec:
            return False
    return False


def get_node_logger(node: Node, message: str):
    """
    Helper to log messages from nodes with consistent formatting

    Args:
        node: The ROS 2 node
        message: Message to log
    """
    node.get_logger().info(f"[{node.get_name()}] {message}")


def shutdown_ros2():
    """Shutdown ROS 2 if initialized"""
    if rclpy.ok():
        rclpy.shutdown()


class CommonNode(Node):
    """
    A common node class that provides useful functionality
    for educational examples
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._received_messages = []
        self._published_messages = []

    def log_info(self, message: str):
        """Log an info message with node name prefix"""
        self.get_logger().info(f"[{self.get_name()}] {message}")

    def log_debug(self, message: str):
        """Log a debug message with node name prefix"""
        self.get_logger().debug(f"[{self.get_name()}] {message}")

    def log_error(self, message: str):
        """Log an error message with node name prefix"""
        self.get_logger().error(f"[{self.get_name()}] {message}")

    def store_received_message(self, msg, topic_name: str = None):
        """Store received messages for verification in examples"""
        self._received_messages.append({
            'timestamp': self.get_clock().now(),
            'message': msg,
            'topic': topic_name
        })

    def store_published_message(self, msg, topic_name: str = None):
        """Store published messages for verification in examples"""
        self._published_messages.append({
            'timestamp': self.get_clock().now(),
            'message': msg,
            'topic': topic_name
        })


def run_node_once(node: Node, timeout_sec: float = 0.1):
    """
    Run a single iteration of the node's spin loop

    Args:
        node: The node to run
        timeout_sec: Timeout for the spin operation
    """
    try:
        rclpy.spin_once(node, timeout_sec=timeout_sec)
    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
        return False
    return True


def wait_for_nodes(timeout_sec: int = 10):
    """
    Wait for nodes to be ready

    Args:
        timeout_sec: Maximum time to wait in seconds
    """
    start_time = time.time()
    while time.time() - start_time < timeout_sec:
        if rclpy.ok():
            return True
        time.sleep(0.1)
    return False


def create_test_message(content: str = "test message") -> String:
    """
    Create a standard test message for examples

    Args:
        content: Content for the test message

    Returns:
        String: A std_msgs/String with the specified content
    """
    msg = String()
    msg.data = content
    return msg


def format_duration(seconds: float) -> str:
    """
    Format a duration in seconds to a human-readable string

    Args:
        seconds: Duration in seconds

    Returns:
        str: Formatted duration string
    """
    if seconds < 1:
        return f"{seconds*1000:.1f}ms"
    elif seconds < 60:
        return f"{seconds:.1f}s"
    else:
        minutes = int(seconds // 60)
        remaining_seconds = seconds % 60
        return f"{minutes}m {remaining_seconds:.1f}s"


def main():
    """Example usage of utility functions"""
    print("ROS 2 Common Utilities Module")
    print("This module provides common functions for ROS 2 examples")
    print("Import this module in your ROS 2 example scripts to use utilities")


if __name__ == "__main__":
    main()