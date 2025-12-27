#!/usr/bin/env python3

"""
Environment Verification Script
This script verifies that the ROS 2 environment is properly set up
for the ROS 2 for Humanoid Robotics educational module.
"""

import subprocess
import sys
import os
import platform
from pathlib import Path


def run_command(cmd, description="Running command"):
    """Run a command and return the result"""
    try:
        print(f"\n{description}")
        print(f"Command: {cmd}")

        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            timeout=15
        )

        print(f"Return code: {result.returncode}")

        if result.stdout.strip():
            print(f"Output: {result.stdout.strip()}")

        if result.stderr.strip():
            print(f"Error: {result.stderr.strip()}")

        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        print("Command timed out after 15 seconds")
        return 1, "", "Command timed out"


def check_system_requirements():
    """Check if the system meets basic requirements"""
    print("\n" + "="*50)
    print("SYSTEM REQUIREMENTS CHECK")
    print("="*50)

    # Check OS
    os_name = platform.system()
    os_version = platform.release()
    print(f"Operating System: {os_name} {os_version}")

    # Check Ubuntu specifically
    if os_name.lower() == "linux":
        try:
            with open("/etc/os-release", "r") as f:
                os_info = f.read()
                if "ubuntu" in os_info.lower():
                    if "22.04" in os_info:
                        print("✓ Ubuntu 22.04 detected - meets requirements")
                    else:
                        print("? Different Ubuntu version detected - may work but not tested")
                else:
                    print("? Non-Ubuntu Linux detected - may work but not tested")
        except FileNotFoundError:
            print("? Could not determine exact Linux distribution")

    # Check Python version
    python_version = sys.version_info
    print(f"Python version: {python_version.major}.{python_version.minor}.{python_version.micro}")

    if python_version.major >= 3 and python_version.minor >= 8:
        print("✓ Python version meets requirements (3.8+)")
    else:
        print("✗ Python version does not meet requirements (need 3.8+)")


def check_ros2_installation():
    """Check if ROS 2 is installed and accessible"""
    print("\n" + "="*50)
    print("ROS 2 INSTALLATION CHECK")
    print("="*50)

    # Check if ros2 command is available
    returncode, stdout, stderr = run_command("ros2 --version", "Checking ROS 2 version")

    if returncode == 0 and stdout.strip():
        print("✓ ROS 2 is installed")
        print(f"  Version: {stdout.strip()}")

        # Check ROS_DISTRO environment variable
        ros_distro = os.environ.get('ROS_DISTRO', 'NOT SET')
        print(f"  Distribution: {ros_distro}")

        if ros_distro.lower() in ['humble', 'humble Hawksbill']:
            print("✓ Correct ROS 2 distribution (Humble Hawksbill) detected")
        else:
            print("? Different ROS 2 distribution detected - may not match educational module")

        return True
    else:
        print("✗ ROS 2 is not installed or not in PATH")
        print("  Please install ROS 2 Humble Hawksbill before proceeding")
        return False


def check_python_packages():
    """Check if required Python packages are available"""
    print("\n" + "="*50)
    print("PYTHON PACKAGES CHECK")
    print("="*50)

    required_packages = [
        "rclpy",
        "std_msgs",
        "geometry_msgs",
        "sensor_msgs"
    ]

    missing_packages = []

    for pkg in required_packages:
        try:
            # For Python packages, we need to import them differently
            if pkg == "rclpy":
                import rclpy
                print(f"✓ {pkg}")
            elif pkg == "std_msgs":
                from std_msgs.msg import String
                print(f"✓ {pkg}")
            elif pkg == "geometry_msgs":
                from geometry_msgs.msg import Twist
                print(f"✓ {pkg}")
            elif pkg == "sensor_msgs":
                from sensor_msgs.msg import JointState
                print(f"✓ {pkg}")
            else:
                __import__(pkg.replace('-', '_'))
                print(f"✓ {pkg}")
        except ImportError as e:
            print(f"✗ {pkg} - import error: {e}")
            missing_packages.append(pkg)

    if missing_packages:
        print(f"\nMissing packages: {', '.join(missing_packages)}")
        print("These packages are required for the educational module")
        return False
    else:
        print("\n✓ All required Python packages are available")
        return True


def check_environment_variables():
    """Check important ROS 2 environment variables"""
    print("\n" + "="*50)
    print("ENVIRONMENT VARIABLES CHECK")
    print("="*50)

    env_vars = [
        ("ROS_DISTRO", "ROS 2 Distribution"),
        ("ROS_DOMAIN_ID", "ROS Domain ID"),
        ("RMW_IMPLEMENTATION", "ROS Middleware Implementation")
    ]

    all_set = True
    for var, description in env_vars:
        value = os.environ.get(var, "NOT SET")
        if value != "NOT SET":
            print(f"✓ {description}: {var}={value}")
        else:
            print(f"✗ {description}: {var}={value}")
            all_set = False

    return all_set


def check_workspace_setup():
    """Check if a ROS 2 workspace is properly set up"""
    print("\n" + "="*50)
    print("WORKSPACE SETUP CHECK")
    print("="*50)

    # Look for common workspace locations
    possible_workspaces = [
        os.path.expanduser("~/ros2_ws"),
        os.path.expanduser("~/humanoid_ws"),
        os.getcwd()
    ]

    workspace_found = False
    for ws_path in possible_workspaces:
        if os.path.exists(ws_path) and os.path.isdir(ws_path):
            src_path = os.path.join(ws_path, "src")
            if os.path.exists(src_path) and os.path.isdir(src_path):
                print(f"✓ Workspace found at: {ws_path}")
                print(f"  Source directory exists: {src_path}")

                # Check if workspace is built
                install_path = os.path.join(ws_path, "install")
                if os.path.exists(install_path):
                    print(f"  Install directory exists: {install_path} (workspace appears built)")
                else:
                    print(f"  Install directory missing: {install_path} (workspace may need building)")

                workspace_found = True
                break

    if not workspace_found:
        print("✗ No ROS 2 workspace found in common locations")
        print("  Consider creating a workspace with: mkdir -p ~/ros2_ws/src")

    return workspace_found


def run_basic_ros2_commands():
    """Run basic ROS 2 commands to verify functionality"""
    print("\n" + "="*50)
    print("BASIC ROS 2 COMMANDS CHECK")
    print("="*50)

    commands = [
        ("ros2 topic list", "List topics"),
        ("ros2 service list", "List services"),
        ("ros2 node list", "List nodes"),
        ("ros2 param list", "List parameters")
    ]

    all_work = True
    for cmd, description in commands:
        returncode, stdout, stderr = run_command(cmd, f"Testing: {description}")
        if returncode != 0:
            all_work = False
            print(f"  Command failed: {cmd}")
        else:
            print(f"  ✓ {description} - OK")

    return all_work


def main():
    """Main verification function"""
    print("ROS 2 for Humanoid Robotics - Environment Verification")
    print("=" * 60)
    print("This script verifies that your environment is ready for the")
    print("ROS 2 for Humanoid Robotics educational module.")
    print("=" * 60)

    # Check system requirements
    check_system_requirements()

    # Check ROS 2 installation
    ros2_ok = check_ros2_installation()

    if not ros2_ok:
        print("\n" + "="*60)
        print("VERIFICATION FAILED")
        print("ROS 2 is not properly installed.")
        print("Please install ROS 2 Humble Hawksbill before proceeding.")
        print("="*60)
        sys.exit(1)

    # Check Python packages
    packages_ok = check_python_packages()

    # Check environment variables
    env_ok = check_environment_variables()

    # Check workspace setup
    workspace_ok = check_workspace_setup()

    # Run basic commands
    commands_ok = run_basic_ros2_commands()

    print("\n" + "="*60)
    print("VERIFICATION SUMMARY")
    print("="*60)

    results = [
        ("System Requirements", True),
        ("ROS 2 Installation", ros2_ok),
        ("Python Packages", packages_ok),
        ("Environment Variables", env_ok),
        ("Workspace Setup", workspace_ok),
        ("Basic Commands", commands_ok)
    ]

    all_passed = True
    for test, passed in results:
        status = "PASS" if passed else "FAIL"
        symbol = "✓" if passed else "✗"
        print(f"{symbol} {test}: {status}")
        if not passed:
            all_passed = False

    print("\n" + "="*60)
    if all_passed:
        print("✓ ALL CHECKS PASSED")
        print("Your environment is ready for the ROS 2 for Humanoid Robotics module!")
    else:
        print("✗ SOME CHECKS FAILED")
        print("Please address the issues above before proceeding with the module.")
        print("Refer to the documentation for setup instructions.")
    print("="*60)

    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())