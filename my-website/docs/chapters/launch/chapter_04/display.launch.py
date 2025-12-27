from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    """Launch file for displaying the humanoid robot model in RViz."""

    # Define launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(os.getcwd(), 'urdf', 'simple_humanoid.urdf'),
        description='URDF file path'
    )

    # Robot State Publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(LaunchConfiguration('model')).read(),
            'publish_frequency': 50.0
        }]
    )

    # Joint State Publisher node (GUI for setting joint positions)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz2 node
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])