#!/usr/bin/env python3

"""
Humanoid Robot Sensor Subscriber
This node receives and processes sensor data from a humanoid robot for Exercise 2.2
in the ROS 2 for Humanoid Robotics educational module.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan
from std_msgs.msg import String
import time
import csv
import os
from datetime import datetime


class HumanoidSensorSubscriber(Node):
    """
    A subscriber node that receives and processes humanoid robot sensor data.
    This implements Exercise 2.2: Create subscriber processing sensor data and logging it.
    """

    def __init__(self):
        super().__init__('humanoid_sensor_subscriber')

        # Create subscriptions for different sensor types
        self.joint_subscription = self.create_subscription(
            JointState,
            'humanoid/joint_states',
            self.joint_callback,
            10
        )

        self.imu_subscription = self.create_subscription(
            Imu,
            'humanoid/imu/data',
            self.imu_callback,
            10
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'humanoid/scan',
            self.scan_callback,
            10
        )

        # Publisher for processed data summary
        self.summary_publisher = self.create_publisher(String, 'sensor_summary', 10)

        # Initialize CSV logging
        self.csv_filename = f"sensor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 'message_type', 'data_summary'
        ])
        self.get_logger().info(f'Logging sensor data to: {self.csv_filename}')

        self.get_logger().info('Humanoid Sensor Subscriber Node has started')

    def joint_callback(self, msg):
        """Process joint state messages and log them"""
        current_time = time.time()
        timestamp = datetime.fromtimestamp(current_time).strftime('%Y-%m-%d %H:%M:%S.%f')

        # Calculate some basic statistics
        if msg.position:
            avg_pos = sum(msg.position) / len(msg.position)
            max_pos = max(msg.position)
            min_pos = min(msg.position)

            summary = f'Joints: {len(msg.name)}, Avg Pos: {avg_pos:.3f}, Range: [{min_pos:.3f}, {max_pos:.3f}]'

            # Log to CSV
            self.csv_writer.writerow([timestamp, 'joint_states', summary])
            self.csv_file.flush()  # Ensure data is written immediately

            # Log to console
            self.get_logger().info(f'[{timestamp}] Joint Data - {summary}')

            # Publish summary
            summary_msg = String()
            summary_msg.data = f'Joint states: {len(msg.name)} joints, avg pos: {avg_pos:.3f}'
            self.summary_publisher.publish(summary_msg)

    def imu_callback(self, msg):
        """Process IMU messages and log them"""
        current_time = time.time()
        timestamp = datetime.fromtimestamp(current_time).strftime('%Y-%m-%d %H:%M:%S.%f')

        # Extract and log orientation info
        orientation_summary = f'X:{msg.orientation.x:.3f}, Y:{msg.orientation.y:.3f}, Z:{msg.orientation.z:.3f}, W:{msg.orientation.w:.3f}'

        # Log to CSV
        self.csv_writer.writerow([timestamp, 'imu_data', orientation_summary])
        self.csv_file.flush()

        # Log to console
        self.get_logger().info(f'[{timestamp}] IMU Data - {orientation_summary}')

    def scan_callback(self, msg):
        """Process laser scan messages and log them"""
        current_time = time.time()
        timestamp = datetime.fromtimestamp(current_time).strftime('%Y-%m-%d %H:%M:%S.%f')

        # Calculate some basic statistics
        if msg.ranges:
            valid_ranges = [r for r in msg.ranges if r >= msg.range_min and r <= msg.range_max]
            if valid_ranges:
                avg_range = sum(valid_ranges) / len(valid_ranges)
                min_range = min(valid_ranges)
                summary = f'Scan points: {len(msg.ranges)}, Valid: {len(valid_ranges)}, Avg: {avg_range:.2f}m, Min: {min_range:.2f}m'
            else:
                summary = f'Scan points: {len(msg.ranges)}, No valid ranges'
        else:
            summary = f'Scan points: 0'

        # Log to CSV
        self.csv_writer.writerow([timestamp, 'laser_scan', summary])
        self.csv_file.flush()

        # Log to console
        self.get_logger().info(f'[{timestamp}] Scan Data - {summary}')

    def destroy_node(self):
        """Override destroy_node to properly close CSV file"""
        if hasattr(self, 'csv_file') and self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f'Closed CSV log file: {self.csv_filename}')
        return super().destroy_node()


def main(args=None):
    """Main function to run the sensor subscriber node"""
    rclpy.init(args=args)

    sensor_subscriber = HumanoidSensorSubscriber()

    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        sensor_subscriber.get_logger().info('Node interrupted by user')
    finally:
        sensor_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()