# Chapter 4: URDF & Robot Description for Humanoid Robotics

## Learning Objectives

By the end of this chapter, you will be able to:

- Create URDF models for humanoid robots
- Define joints, links, and sensors in URDF
- Visualize robots in RViz and simulate in Gazebo
- Connect URDF models to ROS 2 controllers

## 1. URDF Fundamentals

### XML structure and syntax

URDF (Unified Robot Description Format) is an XML-based format used to describe robots. It defines the physical and visual properties of a robot including:

- Links: rigid parts of the robot
- Joints: connections between links
- Materials: visual appearance
- Inertial properties: mass, center of mass, and inertia

### Links: physical components of the robot

Links represent rigid bodies in the robot. Each link has:

- Visual properties: how it appears in simulation
- Collision properties: how it interacts with other objects
- Inertial properties: mass and inertia characteristics

### Joints: connections between links

Joints define how links connect and move relative to each other. Common joint types include:

- **Fixed**: No movement between links
- **Revolute**: Rotational movement around an axis
- **Continuous**: Continuous rotation around an axis
- **Prismatic**: Linear sliding movement
- **Floating**: 6 degrees of freedom
- **Planar**: Movement on a plane

### Materials and visual properties

Materials define the visual appearance of links, including color and texture.

## 2. Humanoid Robot Modeling

### Humanoid kinematic chains

Humanoid robots have specific kinematic structures:

- **Trunk**: Main body with head attachment
- **Arms**: Shoulder, elbow, and wrist joints
- **Legs**: Hip, knee, and ankle joints
- **Head**: Neck joints for vision systems

### Joint types: revolute, continuous, prismatic, fixed

For humanoid robots, you'll primarily use revolute joints for most articulations, with fixed joints for non-moving connections.

### Inertial properties and dynamics

Accurate inertial properties are crucial for realistic simulation and control of humanoid robots.

### Sensor placement and definition

Sensors like IMUs, cameras, and force-torque sensors need to be properly placed in the URDF.

## 3. Visualization and Simulation

### RViz setup for URDF visualization

RViz is ROS's visualization tool that can display URDF models with joint state information.

### Joint state publisher for animation

The joint state publisher allows you to visualize robot movement by publishing joint state messages.

### Gazebo integration for physics simulation

Gazebo provides physics simulation for testing robot behavior in a realistic environment.

### TF (Transform) tree concepts

URDF creates a transform tree that ROS uses to understand the spatial relationships between robot parts.

## 4. Controller Integration

### ros2_control framework overview

The ros2_control framework provides a standardized way to interface with robot hardware.

### Hardware interface definitions

Hardware interfaces define how ROS 2 communicates with physical or simulated robot hardware.

### Joint state publisher and controller manager

These components manage the flow of joint state information and control commands.

### Real-time control considerations

Humanoid robots require real-time control capabilities for stable operation.

## URDF Examples

### Basic link definition

```xml
<link name="base_link">
  <visual>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <geometry>
      <cylinder length="0.6" radius="0.2"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="10"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
</link>
```

### Basic joint definition

```xml
<joint name="base_to_upper_leg" type="revolute">
  <parent link="base_link"/>
  <child link="upper_leg"/>
  <origin xyz="0 0 -0.3" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="3.14"/>
</joint>
```

## Hands-On Exercises

### Exercise 4.1: Create a basic URDF model of a simple humanoid robot

Create a simple URDF model with a torso, head, two arms, and two legs.

### Exercise 4.2: Add joint limits and visual properties to the model

Enhance your URDF model with appropriate joint limits and visual materials.

### Exercise 4.3: Visualize the robot in RViz with joint state publisher

Load your URDF model in RViz and use the joint state publisher to visualize it.

### Exercise 4.4: Integrate the URDF model with ROS 2 controllers

Connect your URDF model to basic ROS 2 controllers for simulation.

### Exercise 4.5: Simulate the humanoid robot in Gazebo

Load your humanoid robot in Gazebo for physics simulation.

## Summary

This chapter covered URDF (Unified Robot Description Format) for humanoid robotics, including the fundamentals of URDF structure, humanoid-specific modeling considerations, visualization and simulation techniques, and controller integration. URDF is essential for representing robot geometry and kinematics in ROS 2, enabling visualization, simulation, and control.

## References

- [URDF Documentation](http://wiki.ros.org/urdf)
- [URDF Tutorials](http://wiki.ros.org/urdf/Tutorials)
- [ros2_control Documentation](https://control.ros.org/)
- [Gazebo Integration](http://gazebosim.org/tutorials?cat=connect_ros)