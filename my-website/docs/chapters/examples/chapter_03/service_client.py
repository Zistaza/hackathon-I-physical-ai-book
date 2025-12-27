#!/usr/bin/env python3

"""
Humanoid Robot Service Client
This node demonstrates how to call services for humanoid robot operations
for the ROS 2 for Humanoid Robotics educational module.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts, SetBool


class HumanoidServiceClient(Node):
    """
    A service client that calls services for humanoid robot operations.
    This demonstrates service client implementation for humanoid robotics applications.
    """

    def __init__(self):
        super().__init__('humanoid_service_client')

        # Create clients for different services
        self.add_two_ints_cli = self.create_client(AddTwoInts, 'add_two_ints')
        self.calibrate_sensor_cli = self.create_client(SetBool, 'calibrate_sensor')
        self.calculate_distance_cli = self.create_client(AddTwoInts, 'calculate_distance')

        # Wait for services to be available
        while not self.add_two_ints_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('AddTwoInts service not available, waiting again...')

        while not self.calibrate_sensor_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Calibrate sensor service not available, waiting again...')

        while not self.calculate_distance_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Calculate distance service not available, waiting again...')

        self.get_logger().info('All services are available')

    def call_add_two_ints(self, a, b):
        """Call the add_two_ints service"""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Calling add_two_ints service with {a} + {b}')
        future = self.add_two_ints_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'Service result: {a} + {b} = {result.sum}')
            return result.sum
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())
            return None

    def call_calibrate_sensor(self, should_calibrate):
        """Call the calibrate sensor service"""
        request = SetBool.Request()
        request.data = should_calibrate

        self.get_logger().info(f'Calling calibrate_sensor service with data={should_calibrate}')
        future = self.calibrate_sensor_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'Calibration result: success={result.success}, message="{result.message}"')
            return result.success
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())
            return False

    def call_calculate_distance(self, x, y):
        """Call the calculate distance service"""
        request = AddTwoInts.Request()
        request.a = x
        request.b = y

        self.get_logger().info(f'Calling calculate_distance service for point ({x}, {y})')
        future = self.calculate_distance_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f'Distance result: {result.sum}')
            return result.sum
        else:
            self.get_logger().error('Exception while calling service: %r' % future.exception())
            return None


def main(args=None):
    """Main function to run the service client"""
    rclpy.init(args=args)

    service_client = HumanoidServiceClient()

    try:
        # Demonstrate calling different services
        # Add two integers
        sum_result = service_client.call_add_two_ints(10, 20)

        # Calibrate sensors (success case)
        cal_result = service_client.call_calibrate_sensor(True)

        # Calibrate sensors (cancel case)
        cancel_result = service_client.call_calibrate_sensor(False)

        # Calculate distance
        dist_result = service_client.call_calculate_distance(3, 4)

        service_client.get_logger().info('All service calls completed')

    except KeyboardInterrupt:
        service_client.get_logger().info('Service client interrupted by user')
    finally:
        service_client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()