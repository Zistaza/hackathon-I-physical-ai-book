# Quickstart Guide: ROS 2 for Humanoid Robotics

This quickstart guide provides a fast introduction to the ROS 2 for Humanoid Robotics educational module.

## Prerequisites

- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill installed
- Python 3.8 or higher
- Basic knowledge of Python programming

## Installation

1. Install ROS 2 Humble Hawksbill following the official installation guide
2. Clone or download this repository
3. Set up your ROS 2 environment:

```bash
source /opt/ros/humble/setup.bash
```

## Getting Started

### 1. Run the Hello World Example

Navigate to the examples directory and run the basic hello world node:

```bash
cd examples/chapter_01
python3 hello_world_node.py
```

In another terminal, you can echo the messages:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /hello_world std_msgs/msg/String
```

### 2. Run Publisher and Subscriber Examples

Run the publisher node:

```bash
cd examples/chapter_02
python3 publisher_node.py
```

In another terminal, run the subscriber:

```bash
cd examples/chapter_02
python3 subscriber_node.py
```

### 3. Visualize the Humanoid Robot Model

Launch RViz to visualize the URDF model:

```bash
# First, make sure you're in the root directory of the project
cd /path/to/ros2-humanoid-robotics

# Launch the display
ros2 launch launch/chapter_04/display.launch.py model:=urdf/simple_humanoid.urdf
```

### 4. Run Service Examples

Start the service server:

```bash
cd examples/chapter_03
python3 service_server.py
```

In another terminal, call the service:

```bash
cd examples/chapter_03
python3 service_client.py
```

## Directory Structure

```
.
├── docs/                   # Documentation (Docusaurus format)
│   ├── chapters/           # Chapter-specific documentation
│   ├── tutorials/
│   └── guides/
├── examples/               # Example code by chapter
│   ├── chapter_01/         # Chapter 1 examples
│   ├── chapter_02/         # Chapter 2 examples
│   ├── chapter_03/         # Chapter 3 examples
│   └── chapter_04/         # Chapter 4 examples
├── urdf/                   # URDF robot models
│   ├── simple_humanoid.urdf
│   ├── complex_humanoid.urdf
│   ├── humanoid_with_sensors.urdf
│   └── humanoid.xacro
├── launch/                 # Launch files
│   └── chapter_04/         # Chapter 4 launch files
└── specs/001-ros2-humanoid-robotics/
    ├── plan.md             # Implementation plan
    ├── tasks.md            # Task breakdown
    └── quickstart.md       # This file
```

## Next Steps

1. Read through the chapters in order:
   - Chapter 1: Introduction to ROS 2
   - Chapter 2: Nodes & Topics
   - Chapter 3: Services & Actions
   - Chapter 4: URDF & Robot Description

2. Complete the exercises in each chapter

3. Experiment with the provided examples

4. Try modifying the URDF models to understand robot description concepts

## Troubleshooting

### Common Issues

- **"command not found"**: Make sure ROS 2 environment is sourced
- **Python import errors**: Ensure rclpy is installed (`pip3 install rclpy`)
- **RViz not showing robot**: Check that the URDF path is correct

### Getting Help

- Check the ROS 2 documentation: https://docs.ros.org/en/humble/
- Visit the ROS Discourse: https://discourse.ros.org/
- Review the examples in this repository

## Resources

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [URDF Documentation](http://wiki.ros.org/urdf)
- [rviz Documentation](http://wiki.ros.org/rviz)