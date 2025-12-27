#!/usr/bin/env python3

"""
Humanoid Robot Sensor Publisher
This node simulates sensor data from a humanoid robot for Exercise 2.1
in the ROS 2 for Humanoid Robotics educational module.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu, LaserScan
from geometry_msgs.msg import Vector3
import math
import random


class HumanoidSensorPublisher(Node):
    """
    A publisher node that simulates various sensor data from a humanoid robot.
    This implements Exercise 2.1: Create publisher simulating humanoid robot sensor data.
    """

    def __init__(self):
        super().__init__('humanoid_sensor_publisher')

        # Create publishers for different sensor types
        self.joint_pub = self.create_publisher(JointState, 'humanoid/joint_states', 10)
        self.imu_pub = self.create_publisher(Imu, 'humanoid/imu/data', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'humanoid/scan', 10)

        # Timer to publish sensor data at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)

        # Counter for published messages
        self.i = 0

        self.get_logger().info('Humanoid Sensor Publisher Node has started')

    def publish_sensor_data(self):
        """Publish simulated sensor data from humanoid robot"""
        # Publish joint states
        self.publish_joint_states()

        # Publish IMU data
        self.publish_imu_data()

        # Publish laser scan data
        self.publish_laser_scan()

        self.i += 1

    def publish_joint_states(self):
        """Publish simulated joint state data"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'humanoid_base'

        # Define humanoid joint names
        joint_names = [
            # Head
            'head_pan_joint', 'head_tilt_joint',
            # Left arm
            'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint',
            'left_elbow_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
            # Right arm
            'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint',
            'right_elbow_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
            # Left leg
            'left_hip_yaw_joint', 'left_hip_roll_joint', 'left_hip_pitch_joint',
            'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint',
            # Right leg
            'right_hip_yaw_joint', 'right_hip_roll_joint', 'right_hip_pitch_joint',
            'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint'
        ]

        msg.name = joint_names

        # Generate simulated joint positions (oscillating patterns)
        positions = []
        velocities = []
        efforts = []

        for j, name in enumerate(joint_names):
            # Create different oscillation patterns for each joint
            pos = math.sin((self.i * 0.05) + (j * 0.3)) * 0.2
            vel = math.cos((self.i * 0.05) + (j * 0.3)) * 0.2 * 0.05  # derivative of position
            eff = random.uniform(-0.1, 0.1)  # random effort values

            positions.append(pos)
            velocities.append(vel)
            efforts.append(eff)

        msg.position = positions
        msg.velocity = velocities
        msg.effort = efforts

        self.joint_pub.publish(msg)
        self.get_logger().info(f'Published joint states for {len(joint_names)} joints')

    def publish_imu_data(self):
        """Publish simulated IMU data"""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate orientation (with slight variations)
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(self.i * 0.01) * 0.05  # slight yaw variation
        msg.orientation.w = math.cos(self.i * 0.01) * 0.05

        # Simulate angular velocity
        msg.angular_velocity.x = math.sin(self.i * 0.02) * 0.1
        msg.angular_velocity.y = math.cos(self.i * 0.02) * 0.1
        msg.angular_velocity.z = 0.01

        # Simulate linear acceleration
        msg.linear_acceleration.x = math.sin(self.i * 0.03) * 0.5
        msg.linear_acceleration.y = math.cos(self.i * 0.03) * 0.5
        msg.linear_acceleration.z = 9.81  # gravity

        self.imu_pub.publish(msg)
        self.get_logger().info('Published IMU data')

    def publish_laser_scan(self):
        """Publish simulated laser scan data"""
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'laser_link'

        # Laser scan parameters
        msg.angle_min = -math.pi / 2  # -90 degrees
        msg.angle_max = math.pi / 2    # 90 degrees
        msg.angle_increment = math.pi / 180  # 1 degree increment
        msg.time_increment = 0.0
        msg.scan_time = 0.1
        msg.range_min = 0.1
        msg.range_max = 10.0

        # Calculate number of ranges
        num_ranges = int((msg.angle_max - msg.angle_min) / msg.angle_increment) + 1
        msg.ranges = []

        # Generate simulated ranges (simulating a room with obstacles)
        for i in range(num_ranges):
            angle = msg.angle_min + i * msg.angle_increment
            # Simulate a circular obstacle in front
            distance_to_obstacle = 2.0 + 0.5 * math.sin(3 * angle)
            # Add some noise
            range_val = distance_to_obstacle + random.uniform(-0.1, 0.1)
            msg.ranges.append(range_val)

        self.scan_pub.publish(msg)
        self.get_logger().info(f'Published laser scan with {len(msg.ranges)} ranges')


def main(args=None):
    """Main function to run the sensor publisher node"""
    rclpy.init(args=args)

    sensor_publisher = HumanoidSensorPublisher()

    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        sensor_publisher.get_logger().info('Node interrupted by user')
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()