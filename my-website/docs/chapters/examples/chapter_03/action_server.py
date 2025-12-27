#!/usr/bin/env python3

"""
Humanoid Robot Action Server
This node implements actions for humanoid robot operations for the
ROS 2 for Humanoid Robotics educational module.
"""

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# Using standard action types for demonstration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point
import time
import math


class HumanoidActionServer(Node):
    """
    An action server that handles goal-oriented tasks for humanoid robots.
    This demonstrates action server implementation for humanoid robotics applications.
    """

    def __init__(self):
        super().__init__('humanoid_action_server')

        # Create action server with a callback group for reentrancy
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            'move_to_pose',
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info('Humanoid Action Server has started')

    def destroy_node(self):
        """Clean up the action server"""
        self._action_server.destroy()
        super().destroy_node()

    def cancel_callback(self, goal_handle):
        """Handle action cancellation requests"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Execute the action goal.
        This simulates moving a humanoid robot to a target pose.
        """
        self.get_logger().info('Executing move_to_pose action...')

        # Get the trajectory from the goal
        trajectory = goal_handle.request.trajectory
        n_points = len(trajectory.points)

        if n_points == 0:
            self.get_logger().info('Empty trajectory received')
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            goal_handle.succeed()
            return result

        # Simulate moving through trajectory points
        for i, point in enumerate(trajectory.points):
            # Check if there's a cancel request
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Action canceled')
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                return result

            # Publish feedback
            feedback_msg = FollowJointTrajectory.Feedback()
            feedback_msg.joint_names = trajectory.joint_names
            feedback_msg.actual.positions = point.positions
            feedback_msg.actual.velocities = point.velocities
            feedback_msg.desired = point  # In a real system, this would be the target
            feedback_msg.error.positions = [0.0] * len(point.positions)  # No error in simulation

            # Calculate progress percentage
            progress = float(i + 1) / float(n_points) * 100.0
            feedback_msg.error.time_from_start.sec = int(progress)  # Using this field to show progress

            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: Moving to point {i+1} of {n_points} ({progress:.1f}%)')

            # Simulate movement time
            time.sleep(0.5)

        # Complete the action successfully
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info('Action completed successfully')

        return result


def main(args=None):
    """Main function to run the action server"""
    rclpy.init(args=args)

    action_server = HumanoidActionServer()

    try:
        # Use a multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(action_server)
        executor.spin()
    except KeyboardInterrupt:
        action_server.get_logger().info('Action server interrupted by user')
    finally:
        action_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()