# Chapter 1: Introduction to ROS 2 for Humanoid Robotics

## Learning Objectives

By the end of this chapter, you will be able to:

- Understand the fundamentals of ROS 2 middleware architecture
- Distinguish between ROS 1 and ROS 2 concepts
- Identify core components: nodes, topics, services, actions
- Set up ROS 2 development environment for humanoid robotics

## 1. Introduction to ROS 2

Robot Operating System 2 (ROS 2) is not an operating system but rather a middleware framework designed for robotics applications. It provides services such as hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

### What is ROS 2 and why it matters for humanoid robotics

ROS 2 is the next generation of the Robot Operating System, designed to address the limitations of ROS 1 while maintaining its core philosophy of code reuse and modularity. For humanoid robotics specifically, ROS 2 provides:

- Real-time capabilities for precise control of robot joints
- Improved security features for safe human-robot interaction
- Better support for multi-robot systems
- Enhanced Quality of Service (QoS) policies for reliable communication

### Middleware architecture and distributed computing concepts

ROS 2 uses a middleware layer based on Data Distribution Service (DDS) to enable communication between different processes and nodes. This allows for:

- Language independence (C++, Python, and other languages can communicate)
- Platform independence (different operating systems can participate)
- Network transparency (nodes can run on different machines)

### DDS (Data Distribution Service) overview

DDS is an object-oriented API specification for distributed real-time systems. It provides a publisher-subscriber communication model and is designed to be scalable from small embedded systems to large server systems.

## 2. ROS 1 vs ROS 2 Comparison

### Architectural differences

| ROS 1 | ROS 2 |
|-------|-------|
| Master-based architecture | Masterless architecture (DDS-based) |
| Single-threaded execution | Multi-threaded execution |
| No Quality of Service (QoS) | Configurable QoS policies |
| Limited real-time support | Real-time support |
| No security | Built-in security features |

### Quality of Service (QoS) policies

QoS policies allow you to configure how messages are delivered between nodes:

- **Reliability**: Best effort vs Reliable delivery
- **Durability**: Volatile vs Transient local
- **History**: Keep last N messages vs Keep all messages
- **Deadline**: Maximum time between consecutive messages
- **Liveliness**: How to detect if a publisher is alive

### Package management improvements

ROS 2 uses ament as its build system, which is more flexible than catkin used in ROS 1. It allows for:

- Better dependency management
- Integration with system package managers
- Support for multiple build systems

### Security enhancements

ROS 2 includes built-in security features such as:

- Authentication of nodes
- Encryption of communication
- Authorization of operations

## 3. Core Concepts Overview

### Nodes: the building blocks of ROS

A node is a process that performs computation. In ROS 2, nodes are the fundamental building blocks of a robot application. Each node can publish or subscribe to messages, provide or use services, and send or execute actions.

### Topics: asynchronous communication

Topics enable asynchronous communication between nodes using a publish-subscribe pattern. Publishers send messages to a topic, and subscribers receive messages from a topic. Multiple publishers and subscribers can exist for the same topic.

### Services: synchronous request/response

Services provide synchronous communication with a request/response pattern. A client sends a request to a service, and the service sends back a response. This is useful for operations that need to return a result immediately.

### Actions: goal-oriented communication with feedback

Actions are designed for long-running tasks that require feedback and the ability to cancel. They follow a goal-feedback-result pattern and are ideal for tasks like navigation or manipulation.

## 4. Setting Up Your Environment

### Installing ROS 2 Humble

ROS 2 Humble Hawksbill is the latest long-term support (LTS) release. To install it on Ubuntu 22.04:

```bash
# Add the ROS 2 apt repository
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
```

### Configuring workspace for humanoid robotics

```bash
# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Create a workspace for humanoid robotics
mkdir -p ~/humanoid_ws/src
cd ~/humanoid_ws

# Build the workspace (even if empty)
colcon build

# Source the workspace
source install/setup.bash
```

### Basic command line tools (ros2, colcon)

The `ros2` command provides access to various ROS 2 tools:

- `ros2 run <package> <executable>` - Run a node
- `ros2 topic list` - List all topics
- `ros2 service list` - List all services
- `ros2 node list` - List all nodes
- `ros2 param list` - List parameters of a node

The `colcon` command is used for building packages:

- `colcon build` - Build all packages in the workspace
- `colcon build --packages-select <pkg_name>` - Build specific packages
- `colcon test` - Run tests for packages

## Hands-On Exercises

### Exercise 1.1: Install ROS 2 Humble and verify installation

1. Install ROS 2 Humble on your Ubuntu 22.04 system using the instructions above
2. Verify the installation by running `ros2 --version`
3. Source the ROS 2 environment: `source /opt/ros/humble/setup.bash`

### Exercise 1.2: Create a basic ROS 2 node that publishes system information

Create a simple node that publishes basic system information like the current time, hostname, or other system metrics.

### Exercise 1.3: Explore ROS 2 command line tools and understand node topology

Use ROS 2 command line tools to explore the network of nodes and topics in a running system.

## Summary

In this chapter, we've covered the fundamentals of ROS 2, including its architecture, key differences from ROS 1, and core concepts. We've also set up the development environment and explored basic command-line tools. These fundamentals form the foundation for more advanced topics in humanoid robotics.

## References

- [ROS 2 Documentation](https://docs.ros.org/en/humble/)
- [ROS 2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- [DDS Specification](https://www.omg.org/spec/DDS/About-DDS/)