#!/usr/bin/env python3

"""
ROS 2 Command Line Tools Verification Script
This script verifies that ROS 2 command line tools are working correctly
"""

import subprocess
import sys
import os

def run_command(cmd):
    """Run a command and return the result"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return 1, "", "Command timed out"

def check_ros2_installation():
    """Check if ROS 2 is installed and accessible"""
    print("Checking ROS 2 installation...")

    # Check if ros2 command is available
    returncode, stdout, stderr = run_command("ros2 --version")
    if returncode == 0:
        print(f"✓ ROS 2 is installed: {stdout.strip()}")
        return True
    else:
        print(f"✗ ROS 2 is not installed or not in PATH: {stderr}")
        return False

def check_ros2_commands():
    """Check basic ROS 2 commands"""
    print("\nChecking basic ROS 2 commands...")

    commands = [
        "ros2 topic list",
        "ros2 service list",
        "ros2 node list",
        "ros2 param list",
        "ros2 lifecycle node list"
    ]

    for cmd in commands:
        returncode, stdout, stderr = run_command(cmd)
        if returncode == 0:
            print(f"✓ {cmd}")
        else:
            print(f"✗ {cmd} - {stderr.strip()[:100]}...")

def check_python_packages():
    """Check if required Python packages are available"""
    print("\nChecking Python packages...")

    packages = [
        "rclpy",
        "std_msgs",
        "geometry_msgs",
        "sensor_msgs"
    ]

    for pkg in packages:
        try:
            __import__(pkg.replace('-', '_'))
            print(f"✓ {pkg}")
        except ImportError:
            print(f"✗ {pkg} - not available")

def check_environment_variables():
    """Check important ROS 2 environment variables"""
    print("\nChecking environment variables...")

    env_vars = [
        "ROS_DISTRO",
        "ROS_DOMAIN_ID",
        "RMW_IMPLEMENTATION"
    ]

    for var in env_vars:
        value = os.environ.get(var, "NOT SET")
        if value != "NOT SET":
            print(f"✓ {var}={value}")
        else:
            print(f"✗ {var}={value}")

def main():
    """Main verification function"""
    print("ROS 2 Command Line Tools Verification")
    print("=" * 40)

    if not check_ros2_installation():
        print("\nROS 2 is not properly installed. Please install ROS 2 Humble Hawksbill before proceeding.")
        sys.exit(1)

    check_ros2_commands()
    check_python_packages()
    check_environment_variables()

    print("\nVerification complete!")
    print("If all checks passed, your ROS 2 environment is ready for development.")

if __name__ == "__main__":
    main()