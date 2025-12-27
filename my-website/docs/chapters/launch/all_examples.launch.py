from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Launch file that combines all examples from the ROS 2 for Humanoid Robotics module."""

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

    # Joint State Publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Chapter 1: Hello World Node
    hello_world_node = Node(
        package='std_msgs',  # This will need to be replaced with actual package
        executable='hello_world_node',  # This will be run separately
        name='hello_world_node',
        output='screen'
    )

    # Chapter 2: Publisher Node
    publisher_node = Node(
        package='std_msgs',  # This will need to be replaced with actual package
        executable='publisher_node',  # This will be run separately
        name='publisher_node',
        output='screen'
    )

    # Chapter 2: Subscriber Node
    subscriber_node = Node(
        package='std_msgs',  # This will need to be replaced with actual package
        executable='subscriber_node',  # This will be run separately
        name='subscriber_node',
        output='screen'
    )

    # Chapter 3: Service Server
    service_server = Node(
        package='std_srvs',  # This will need to be replaced with actual package
        executable='service_server',  # This will be run separately
        name='service_server',
        output='screen'
    )

    # Chapter 3: Action Server
    action_server = Node(
        package='control_msgs',  # This will need to be replaced with actual package
        executable='action_server',  # This will be run separately
        name='action_server',
        output='screen'
    )

    # RViz2 node for visualization
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        model_path_arg,
        robot_state_publisher,
        joint_state_publisher,
        # Note: The actual nodes would need to be compiled into packages
        # to be run from launch files. This is a template for the structure.
        rviz2
    ])