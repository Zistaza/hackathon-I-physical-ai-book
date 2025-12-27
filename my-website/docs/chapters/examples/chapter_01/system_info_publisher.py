#!/usr/bin/env python3

"""
System Information Publisher Node
This node publishes system information periodically, implementing Exercise 1.2
from the ROS 2 for Humanoid Robotics educational module.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import platform
import socket
import psutil  # This may not be available in all ROS 2 environments
import time
import subprocess


class SystemInfoPublisher(Node):
    """
    A ROS 2 node that publishes system information including:
    - Current time
    - Hostname
    - CPU usage
    - Memory usage
    - Platform information
    """

    def __init__(self):
        super().__init__('system_info_publisher')

        # Create publisher for system information
        self.publisher = self.create_publisher(String, 'system_info', 10)

        # Timer to publish system info every 5 seconds
        timer_period = 5.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_system_info)

        # Counter for published messages
        self.i = 0

        self.get_logger().info('System Info Publisher Node has started')

    def get_system_info(self):
        """Collect various system information"""
        try:
            # Basic system information
            hostname = socket.gethostname()
            current_time = time.strftime("%Y-%m-%d %H:%M:%S")
            platform_info = f"{platform.system()} {platform.release()}"

            # CPU usage (if psutil is available)
            try:
                cpu_percent = psutil.cpu_percent(interval=1)
                memory_percent = psutil.virtual_memory().percent
                system_info = f"Time: {current_time}, Host: {hostname}, Platform: {platform_info}, CPU: {cpu_percent}%, Memory: {memory_percent}%"
            except:
                # Fallback if psutil is not available
                system_info = f"Time: {current_time}, Host: {hostname}, Platform: {platform_info}"

        except Exception as e:
            # If we can't get system info for any reason, return basic info
            current_time = time.strftime("%Y-%m-%d %H:%M:%S")
            system_info = f"Time: {current_time}, Error collecting detailed info: {str(e)}"

        return system_info

    def publish_system_info(self):
        """Publish system information to the topic"""
        system_info = self.get_system_info()

        msg = String()
        msg.data = f"System Info {self.i}: {system_info}"

        self.publisher.publish(msg)
        self.get_logger().info(f'Published system info: {msg.data}')

        self.i += 1


def main(args=None):
    """Main function to run the System Info Publisher node"""
    rclpy.init(args=args)

    system_info_publisher = SystemInfoPublisher()

    try:
        rclpy.spin(system_info_publisher)
    except KeyboardInterrupt:
        system_info_publisher.get_logger().info('Node interrupted by user')
    finally:
        system_info_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()