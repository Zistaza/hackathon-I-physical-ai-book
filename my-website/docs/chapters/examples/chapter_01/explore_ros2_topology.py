#!/usr/bin/env python3

"""
ROS 2 Topology Exploration Script
This script demonstrates how to explore ROS 2 node topology using command line tools,
implementing Exercise 1.3 from the ROS 2 for Humanoid Robotics educational module.
"""

import subprocess
import sys
import time
from datetime import datetime


def run_ros2_command(cmd, description):
    """Run a ROS 2 command and return the output"""
    try:
        print(f"\n{description}")
        print(f"Command: ros2 {cmd}")
        print("-" * 50)

        result = subprocess.run(
            f"ros2 {cmd}",
            shell=True,
            capture_output=True,
            text=True,
            timeout=10
        )

        if result.returncode == 0:
            if result.stdout.strip():
                print(result.stdout.strip())
            else:
                print("(No output)")
        else:
            print(f"Error: {result.stderr.strip()}")

        print("-" * 50)
    except subprocess.TimeoutExpired:
        print("Command timed out after 10 seconds")
    except Exception as e:
        print(f"Error running command: {e}")


def explore_ros2_topology():
    """Explore ROS 2 topology using various commands"""
    print("ROS 2 Topology Exploration")
    print("=" * 60)
    print(f"Exploration started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("This script demonstrates the use of ROS 2 command line tools")
    print("to explore the node topology in a ROS 2 system.")
    print("=" * 60)

    # List all topics
    run_ros2_command("topic list", "Listing all topics in the system")

    # List all services
    run_ros2_command("service list", "Listing all services in the system")

    # List all nodes
    run_ros2_command("node list", "Listing all nodes in the system")

    # Show information about topics
    run_ros2_command("topic list -t", "Listing topics with types")

    # Show information about services
    run_ros2_command("service list -t", "Listing services with types")

    # If there are nodes, show parameter information
    print("\nNote: To see parameters for specific nodes, run:")
    print("ros2 param list <node_name>")
    print("For example: ros2 param list /talker (if a talker node exists)")

    # Show information about actions (if any)
    run_ros2_command("action list", "Listing all actions in the system")

    # Show interfaces information
    run_ros2_command("interface list", "Listing all available interfaces")

    print(f"\nExploration completed at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")


def demonstrate_node_inspection(node_name=None):
    """Demonstrate node inspection commands"""
    print("\n" + "=" * 60)
    print("NODE INSPECTION DEMONSTRATION")
    print("=" * 60)

    if node_name:
        print(f"Inspecting node: {node_name}")
        run_ros2_command(f"node info {node_name}", f"Getting information about node {node_name}")
    else:
        print("To inspect a specific node, use: ros2 node info <node_name>")
        print("For example: ros2 node info /talker")


def demonstrate_topic_inspection(topic_name=None):
    """Demonstrate topic inspection commands"""
    print("\n" + "=" * 60)
    print("TOPIC INSPECTION DEMONSTRATION")
    print("=" * 60)

    if topic_name:
        print(f"Inspecting topic: {topic_name}")
        run_ros2_command(f"topic info {topic_name}", f"Getting information about topic {topic_name}")
    else:
        print("To inspect a specific topic, use: ros2 topic info <topic_name>")
        print("For example: ros2 topic info /chatter")


def main():
    """Main function to run the topology exploration"""
    print("ROS 2 for Humanoid Robotics - Topology Exploration")
    print("Exercise 1.3: Explore ROS 2 command line tools and node topology")
    print("=" * 80)

    # Explore the current ROS 2 topology
    explore_ros2_topology()

    # Demonstrate how to inspect specific nodes and topics
    # (These will only work if specific nodes/topics exist)
    demonstrate_node_inspection()
    demonstrate_topic_inspection()

    print("\n" + "=" * 80)
    print("EXERCISE NOTES:")
    print("- This script shows how to use ROS 2 command line tools")
    print("- In a real scenario, you would run these commands in a terminal")
    print("- The output will vary depending on what nodes are currently running")
    print("- Try running this script while other ROS 2 nodes are active")
    print("=" * 80)


if __name__ == "__main__":
    main()