#!/usr/bin/env python3

"""
Humanoid Robot Service Server
This node implements services for humanoid robot operations for the
ROS 2 for Humanoid Robotics educational module.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts, SetBool
from sensor_msgs.msg import JointState
import math


class HumanoidServiceServer(Node):
    """
    A service server that provides services for humanoid robot operations.
    This demonstrates service implementation for humanoid robotics applications.
    """

    def __init__(self):
        super().__init__('humanoid_service_server')

        # Create services
        self.add_two_ints_srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.calibrate_sensor_srv = self.create_service(
            SetBool,
            'calibrate_sensor',
            self.calibrate_sensor_callback
        )

        # Service for calculating distance between two points (simplified IK)
        self.distance_srv = self.create_service(
            AddTwoInts,  # Reusing for demonstration - in practice you'd create custom srv
            'calculate_distance',
            self.calculate_distance_callback
        )

        self.get_logger().info('Humanoid Service Server has started')

    def add_two_ints_callback(self, request, response):
        """Callback for adding two integers service"""
        response.sum = request.a + request.b
        self.get_logger().info(f'AddTwoInts: {request.a} + {request.b} = {response.sum}')
        return response

    def calibrate_sensor_callback(self, request, response):
        """Callback for sensor calibration service"""
        if request.data:
            self.get_logger().info('Calibrating sensors...')
            # Simulate calibration process
            result = True
            message = "Sensors calibrated successfully"
        else:
            self.get_logger().info('Sensor calibration canceled')
            result = False
            message = "Calibration canceled by user"

        response.success = result
        response.message = message
        return response

    def calculate_distance_callback(self, request, response):
        """Callback for calculating distance between two points"""
        # Calculate Euclidean distance between points (0,0) and (request.a, request.b)
        distance = math.sqrt(request.a * request.a + request.b * request.b)
        response.sum = int(distance)  # Converting to int for AddTwoInts, but this would use custom message in practice
        self.get_logger().info(f'Distance from (0,0) to ({request.a}, {request.b}) = {distance:.2f}')
        return response


def main(args=None):
    """Main function to run the service server"""
    rclpy.init(args=args)

    service_server = HumanoidServiceServer()

    try:
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        service_server.get_logger().info('Service server interrupted by user')
    finally:
        service_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()