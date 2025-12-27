# Robotics Terminology Reference for Digital Twin Module

## Core Concepts

- **Digital Twin**: A virtual representation of a physical robot system that enables simulation, testing, and validation of robotic behaviors in a safe environment
- **Physics Simulation**: Computational models that replicate real-world physical behaviors including gravity, collisions, friction, and joint dynamics for humanoid robots
- **Sensor Simulation**: Virtual representations of physical sensors (LiDAR, cameras, IMUs) that generate realistic data for perception pipeline development
- **Humanoid Robot Model**: Virtual representation of a human-like robot using URDF format, including kinematic and dynamic properties
- **Sim-to-Real Gap**: The differences between simulation and real-world performance that affect the transferability of learned behaviors and control strategies

## Gazebo-Specific Terms

- **URDF (Unified Robot Description Format)**: XML-based format to describe robot models including kinematic and dynamic properties
- **Physics Engine**: Computational system that simulates physical phenomena like gravity, collisions, and friction (e.g., ODE, Bullet, Simbody)
- **Collision Detection**: Algorithm to determine when objects in the simulation come into contact
- **Joint Constraints**: Limitations on the movement of joints in a robot model
- **Physical Realism**: The degree to which a simulation accurately represents real-world physics

## Unity-Specific Terms

- **Visual Realism**: The degree to which a simulation accurately represents real-world visual appearance
- **Human-Robot Interaction (HRI)**: The study of interactions between humans and robots
- **Unity-ROS Integration**: Connecting Unity environments with ROS systems for robotics applications

## Sensor Simulation Terms

- **LiDAR (Light Detection and Ranging)**: Sensor that measures distance by illuminating target with laser light
- **Depth Camera**: Camera that captures distance information for each pixel in addition to color information
- **RGB Camera**: Standard camera that captures Red, Green, Blue color channels
- **IMU (Inertial Measurement Unit)**: Sensor that measures specific force and angular rate
- **Sensor Noise**: Random variations in sensor readings that represent real-world sensor imperfections
- **Sensor Fusion**: Combining data from multiple sensors to improve accuracy and reliability

## ROS 2 Integration Terms

- **ROS 2 (Robot Operating System 2)**: Flexible framework for writing robot software
- **Middleware**: Software layer that enables communication between different components
- **Sim-to-Real Transfer**: The process of applying knowledge or control strategies learned in simulation to real robots