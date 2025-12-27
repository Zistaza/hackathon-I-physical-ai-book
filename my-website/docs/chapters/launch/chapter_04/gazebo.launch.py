from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Launch file for loading the humanoid robot model in Gazebo simulation."""

    # Define launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(os.getcwd(), 'urdf', 'simple_humanoid.urdf'),
        description='URDF file path'
    )

    # Launch Gazebo with the robot model
    gazebo = ExecuteProcess(
        cmd=[
            'gzserver',
            '--verbose',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_init.so'
        ],
        output='screen'
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

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'simple_humanoid'
        ],
        output='screen'
    )

    # Joint State Publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        joint_state_publisher
    ])