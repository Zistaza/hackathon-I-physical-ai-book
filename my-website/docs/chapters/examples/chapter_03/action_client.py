#!/usr/bin/env python3

"""
Humanoid Robot Action Client
This node demonstrates how to use actions for humanoid robot operations
for the ROS 2 for Humanoid Robotics educational module.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# Using standard action types for demonstration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time


class HumanoidActionClient(Node):
    """
    An action client that sends goals to action servers for humanoid robot operations.
    This demonstrates action client implementation for humanoid robotics applications.
    """

    def __init__(self):
        super().__init__('humanoid_action_client')

        # Create action client
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            'move_to_pose'
        )

        self.get_logger().info('Humanoid Action Client has started')

    def send_goal(self, joint_names, positions_list, velocities_list=None):
        """
        Send a goal to the action server
        :param joint_names: List of joint names
        :param positions_list: List of lists of positions for each point in the trajectory
        :param velocities_list: List of lists of velocities for each point (optional)
        """
        # Wait for the action server to be available
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        # Create the goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = joint_names

        # Create trajectory points
        for i, positions in enumerate(positions_list):
            point = JointTrajectoryPoint()
            point.positions = positions

            if velocities_list and i < len(velocities_list):
                point.velocities = velocities_list[i]
            else:
                # Default to zero velocities
                point.velocities = [0.0] * len(positions)

            # Set time from start (incremental)
            point.time_from_start = Duration()
            point.time_from_start.sec = i + 1  # 1 second per point
            point.time_from_start.nanosec = 0

            goal_msg.trajectory.points.append(point)

        self.get_logger().info(f'Sending goal with {len(positions_list)} trajectory points')

        # Send the goal
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the goal response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server"""
        feedback = feedback_msg.feedback
        # Calculate progress percentage from the time field we used for progress
        progress = feedback.error.time_from_start.sec if feedback.error.time_from_start.sec > 0 else 0
        self.get_logger().info(f'Received feedback: {progress}% complete')

    def get_result_callback(self, future):
        """Handle the result from the action server"""
        result = future.result().result
        self.get_logger().info(f'Result received: error_code={result.error_code}')
        rclpy.shutdown()


def main(args=None):
    """Main function to run the action client"""
    rclpy.init(args=args)

    action_client = HumanoidActionClient()

    # Define a simple trajectory for a humanoid robot
    # Using 3 joints for demonstration: hip, knee, ankle
    joint_names = ['left_hip_joint', 'left_knee_joint', 'left_ankle_joint']

    # Define trajectory points (positions for each joint at each point)
    positions_list = [
        [0.0, 0.0, 0.0],      # Starting position
        [0.1, 0.05, -0.05],   # Mid-point
        [0.2, 0.1, -0.1],     # Final position
    ]

    # Define velocities for smoother motion
    velocities_list = [
        [0.05, 0.05, 0.05],
        [0.05, 0.05, 0.05],
        [0.05, 0.05, 0.05]
    ]

    # Send the goal
    action_client.send_goal(joint_names, positions_list, velocities_list)

    try:
        # Use a multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(action_client)
        executor.spin()
    except KeyboardInterrupt:
        action_client.get_logger().info('Action client interrupted by user')
    finally:
        action_client.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()