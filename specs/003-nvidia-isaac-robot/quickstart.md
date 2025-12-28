# Quickstart Guide: NVIDIA Isaac™ AI-Robot Brain Educational Module

## Overview
This quickstart guide provides a fast path to getting started with the NVIDIA Isaac™ educational module for humanoid robots. It covers the essential setup and initial steps to begin learning.

## Prerequisites

### System Requirements
- Ubuntu 20.04 LTS or later (recommended)
- NVIDIA GPU with compute capability 6.0 or higher
- At least 16GB RAM
- 100GB+ free disk space for Isaac Sim
- ROS 2 Humble Hawksbill installed

### Software Dependencies
- NVIDIA Isaac Sim (latest version)
- Isaac ROS packages
- ROS 2 Navigation Stack (Nav2)
- Docker and NVIDIA Container Toolkit (for containerized workflows)

## Installation Steps

### 1. Install NVIDIA Isaac Sim
1. Download Isaac Sim from NVIDIA Developer website
2. Follow the installation guide for your platform
3. Verify installation by launching Isaac Sim

### 2. Set up ROS 2 Environment
1. Install ROS 2 Humble Hawksbill
2. Set up your ROS 2 workspace:
   ```bash
   mkdir -p ~/isaac_ws/src
   cd ~/isaac_ws
   colcon build
   source install/setup.bash
   ```

### 3. Install Isaac ROS Packages
1. Add the Isaac ROS packages to your workspace
2. Build the packages:
   ```bash
   cd ~/isaac_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

### 4. Install Navigation 2 (Nav2)
1. Install Nav2 packages:
   ```bash
   sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup
   ```

## Getting Started with the Module

### Chapter 1: Intro to NVIDIA Isaac Sim
1. Navigate to the module directory:
   ```bash
   cd my-website/docs/module3-ai-robot-brain/
   ```
2. Open `01-intro-isaac-sim.md` in your preferred viewer
3. Follow the installation and basic simulation exercises

### Chapter 2: Isaac ROS - VSLAM and Perception
1. Complete Chapter 1 prerequisites
2. Open `02-isaac-ros-vslam.md`
3. Follow the perception pipeline exercises

### Chapter 3: Nav2 Path Planning for Humanoids
1. Complete Chapters 1 and 2
2. Open `03-nav2-path-planning.md`
3. Follow the navigation and path planning exercises

### Chapter 4: Integration and Deployment
1. Complete all previous chapters
2. Open `04-integration-deployment.md`
3. Follow the integration and deployment exercises

## Hands-on Exercise Quick Start

Each chapter includes hands-on exercises. To get started with the first exercise:

1. Launch Isaac Sim
2. Load the provided sample scene
3. Follow the step-by-step instructions in the chapter
4. Verify your results against the expected outcomes

## Troubleshooting

### Common Issues
- **Isaac Sim won't launch**: Ensure NVIDIA GPU drivers are properly installed
- **ROS 2 packages not found**: Source your ROS 2 setup file in each new terminal
- **Insufficient GPU memory**: Close other GPU-intensive applications

### Getting Help
- Check the troubleshooting section in each chapter
- Refer to official NVIDIA Isaac documentation
- Join the NVIDIA robotics developer community

## Next Steps

After completing this quickstart:
1. Work through Chapter 1 exercises
2. Progress through the remaining chapters in sequence
3. Complete the hands-on exercises for each chapter
4. Apply your learning to your own humanoid robot projects