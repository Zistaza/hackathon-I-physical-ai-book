---
title: 'Chapter 3: Nav2 Path Planning for Bipedal Humanoids'
sidebar_label: 'Chapter 3: Nav2 & Humanoid Navigation'
description: 'Navigation 2 path planning customized for bipedal humanoid robots'
keywords:
  - Nav2
  - Navigation
  - Path Planning
  - Bipedal Robots
  - Humanoid Robotics
  - ROS 2
---

# Chapter 3: Nav2 Path Planning for Bipedal Humanoids

## Learning Objectives

By the end of this chapter, you will be able to:
- Customize the Navigation 2 stack for bipedal humanoid robots
- Implement footstep planning integration with navigation
- Configure balance-aware path planning
- Use behavior trees for navigation decision-making
- Handle bipedal-specific navigation challenges

## Table of Contents
- [Introduction to Navigation 2 (Nav2)](#introduction-to-navigation-2-nav2)
- [Bipedal Navigation Challenges](#bipedal-navigation-challenges)
- [Customizing Nav2 for Humanoids](#customizing-nav2-for-humanoids)
- [Footstep Planning Integration](#footstep-planning-integration)
- [Balance-Aware Path Planning](#balance-aware-path-planning)
- [Behavior Trees for Navigation](#behavior-trees-for-navigation)
- [Chapter Summary](#chapter-summary)
- [Knowledge Check](#knowledge-check)
- [Hands-on Exercise](#hands-on-exercise)

## Introduction to Navigation 2 (Nav2)

Navigation 2 (Nav2) is the next-generation navigation stack for ROS 2, designed to provide robust and flexible navigation capabilities for mobile robots. It offers a significant upgrade from the original ROS Navigation stack with improved performance, better modularity, and enhanced features.

### Nav2 Architecture

The Nav2 stack consists of several key components:

1. **Navigation Server**: Main service providing navigation capabilities
2. **Global Planner**: Computes global path from start to goal
3. **Local Planner**: Executes navigation while avoiding local obstacles
4. **Controller**: Translates navigation commands to robot motion
5. **Recovery Behaviors**: Handles navigation failures and stuck conditions
6. **Lifecycle Management**: Manages component states and transitions

*Figure: Nav2 Architecture for Humanoid Robots - Showing the components and their interactions for bipedal navigation*

### Core Navigation Components

- **Costmap 2D**: Represents obstacles and free space
- **TF2**: Handles coordinate transformations
- **Action Interface**: Asynchronous navigation commands
- **Plugin Architecture**: Extensible component system

### Nav2 vs Traditional Navigation

Key improvements in Nav2:
- **Lifecycle Management**: Better resource management
- **Improved Recovery**: More robust failure handling
- **Plugin System**: Flexible component replacement
- **Better Performance**: Optimized for real-time operation

## Bipedal Navigation Challenges

### Balance Considerations

Bipedal humanoid robots face unique navigation challenges:

- **Dynamic Balance**: Maintaining stability during motion
- **Zero-Moment Point (ZMP)**: Ensuring stable foot placement
- **Center of Mass (CoM)**: Managing CoM position during movement
- **Double Support Phase**: Managing transitions between feet

### Gait-Specific Navigation

Humanoid navigation must account for gait patterns:

- **Walking Gait**: Alternating single and double support phases
- **Step Size Limitations**: Maximum step length constraints
- **Step Timing**: Precise timing for stable locomotion
- **Turning Mechanics**: Different turning strategies than wheeled robots

### Terrain Adaptation

Bipedal robots require special terrain considerations:

- **Surface Stability**: Ensuring ground contact stability
- **Step Height**: Maximum obstacle height for stepping
- **Surface Friction**: Accounting for slip potential
- **Slope Limitations**: Maximum climbable inclines

## Customizing Nav2 for Humanoids

### Custom Costmap Layers

For bipedal navigation, custom costmap layers are essential:

1. **Footprint Layer**: Humanoid-specific collision checking
2. **Stability Layer**: Balance constraint visualization
3. **Terrain Analysis Layer**: Surface stability assessment

### Custom Planners

Humanoid-specific planners include:

- **Footstep Planner**: Generates stable foot placement sequences
- **Stability-Aware Planner**: Considers balance during path planning
- **Gait-Integrated Planner**: Plans paths considering gait constraints

### Configuration Parameters

Key Nav2 parameters for humanoid robots:

- **Footprint**: Robot's collision footprint in polygon format
- **Inflation Radius**: Safety margin around obstacles
- **Resolution**: Costmap resolution for detailed planning
- **Update Frequency**: How often costmaps update
- **Transform Tolerance**: TF transform tolerance

### Example: Humanoid Nav2 Configuration

```yaml
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: base_link
    odom_topic: /odom
    bt_loop_duration: 10
    default_server_timeout: 20
    enable_groot_monitoring: True
    groot_zmq_publisher_port: 1666
    groot_zmq_server_port: 1667
    goal_checker:
      plugin: "nav2_goal_checker::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: True
    behavior_tree_xml_filename: "navigate_w_replanning_and_recovery.xml"
    # Humanoid-specific parameters
    path_tolerance: 0.15  # Tighter tolerance for precise foot placement
    goal_tolerance: 0.10  # More precise goal approach for foot placement

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 10.0
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.5
    min_theta_velocity_threshold: 0.001
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    FollowPath:
      plugin: "nav2_mppi_controller::Controller"
      time_steps: 50
      control_freq: 10
      model_dt: 0.05
      # Humanoid-specific controller parameters
      vx_samples: 20
      vy_samples: 5
      wz_samples: 30
      vx_max: 0.2   # Reduced for stable walking
      vx_min: -0.1
      vy_max: 0.1
      wz_max: 0.3
      sim_time: 2.5
      xy_goal_tolerance: 0.15
      yaw_goal_tolerance: 0.25

local_costmap:
  local_costmap:
    ros__parameters:
      update_frequency: 5.0
      publish_frequency: 2.0
      global_frame: odom
      robot_base_frame: base_link
      use_sim_time: True
      rolling_window: true
      width: 6
      height: 6
      resolution: 0.05  # Higher resolution for precise foot placement
      robot_radius: 0.3  # Humanoid-specific radius
      plugins: ["voxel_layer", "inflation_layer"]
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
      voxel_layer:
        plugin: "nav2_costmap_2d::VoxelLayer"
        enabled: True
        publish_voxel_map: True
        origin_z: 0.0
        z_resolution: 0.2
        z_voxels: 10
        max_obstacle_height: 2.0
        mark_threshold: 0
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0

global_costmap:
  global_costmap:
    ros__parameters:
      update_frequency: 1.0
      publish_frequency: 0.5
      global_frame: map
      robot_base_frame: base_link
      use_sim_time: True
      robot_radius: 0.3
      resolution: 0.05
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        enabled: True
        observation_sources: scan
        scan:
          topic: /scan
          max_obstacle_height: 2.0
          clearing: True
          marking: True
          data_type: "LaserScan"
          raytrace_max_range: 3.0
          raytrace_min_range: 0.0
          obstacle_max_range: 2.5
          obstacle_min_range: 0.0
      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: True
      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        cost_scaling_factor: 3.0
        inflation_radius: 0.55
```

## Footstep Planning Integration

### What is Footstep Planning?

Footstep planning is the process of determining where and when to place each foot during humanoid locomotion to ensure stable and efficient walking.

### Integration with Nav2

Footstep planning integrates with Nav2 through:

1. **Path Smoothing**: Adapt global path for footstep constraints
2. **Step Generation**: Convert smooth path to discrete foot placements
3. **Stability Verification**: Ensure each step maintains balance
4. **Gait Generation**: Coordinate foot and body motion

### Footstep Planner Components

Key components of a footstep planner:

- **Step Generator**: Creates foot placement candidates
- **Stability Checker**: Verifies balance constraints
- **Path Follower**: Tracks planned footsteps
- **Recovery Handler**: Manages balance recovery

### Example: Footstep Planning Node

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration
import numpy as np
from scipy.spatial.transform import Rotation as R

class FootstepPlannerNode(Node):
    def __init__(self):
        super().__init__('footstep_planner_node')

        # Publishers and subscribers
        self.path_sub = self.create_subscription(
            Path,
            'global_plan',
            self.path_callback,
            10
        )

        self.footsteps_pub = self.create_publisher(
            Path,
            'footstep_plan',
            10
        )

        self.balance_pub = self.create_publisher(
            Bool,
            'balance_status',
            10
        )

        # Parameters
        self.max_step_length = 0.3  # meters
        self.max_step_width = 0.2   # meters
        self.min_step_height = 0.05 # meters
        self.zmp_margin = 0.05      # meters

        self.robot_pose = None
        self.current_foot = 'left'  # Start with left foot

    def path_callback(self, msg):
        """Convert global path to footstep plan"""
        footsteps = self.generate_footsteps(msg.poses)
        footstep_path = Path()
        footstep_path.header = msg.header
        footstep_path.poses = footsteps
        self.footsteps_pub.publish(footstep_path)

        # Check balance for the planned footsteps
        is_balanced = self.check_balance(footsteps)
        self.balance_pub.publish(Bool(data=is_balanced))

    def generate_footsteps(self, path_poses):
        """Generate footstep plan from global path"""
        footsteps = []

        if len(path_poses) < 2:
            return footsteps

        # Start with current robot position
        current_pos = path_poses[0].pose.position
        current_yaw = self.get_yaw_from_quaternion(path_poses[0].pose.orientation)

        footsteps.append(self.create_footstep_pose(current_pos, current_yaw, 'start'))

        # Generate footsteps along the path
        for i in range(1, len(path_poses)):
            target_pos = path_poses[i].pose.position
            target_yaw = self.get_yaw_from_quaternion(path_poses[i].pose.orientation)

            # Calculate step direction and distance
            dx = target_pos.x - current_pos.x
            dy = target_pos.y - current_pos.y
            dist = np.sqrt(dx*dx + dy*dy)

            # Generate intermediate footsteps if needed
            if dist > self.max_step_length:
                # Interpolate to create multiple steps
                num_steps = int(np.ceil(dist / self.max_step_length))
                for step in range(num_steps):
                    ratio = (step + 1) / num_steps
                    interp_x = current_pos.x + ratio * dx
                    interp_y = current_pos.y + ratio * dy
                    interp_yaw = current_yaw + ratio * (target_yaw - current_yaw)

                    foot_pose = self.create_footstep_pose(
                        Point(x=interp_x, y=interp_y, z=0.0),
                        interp_yaw,
                        self.current_foot
                    )
                    footsteps.append(foot_pose)

                    # Alternate feet
                    self.current_foot = 'right' if self.current_foot == 'left' else 'left'
            else:
                # Single step to target
                foot_pose = self.create_footstep_pose(target_pos, target_yaw, self.current_foot)
                footsteps.append(foot_pose)
                self.current_foot = 'right' if self.current_foot == 'left' else 'left'

            current_pos = target_pos
            current_yaw = target_yaw

        return footsteps

    def create_footstep_pose(self, position, yaw, foot_type):
        """Create a PoseStamped for a footstep"""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = 'map'

        pose_stamped.pose.position = position
        # Convert yaw to quaternion
        quat = R.from_euler('z', yaw).as_quat()
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]

        return pose_stamped

    def get_yaw_from_quaternion(self, quat):
        """Extract yaw from quaternion"""
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        return np.arctan2(siny_cosp, cosy_cosp)

    def check_balance(self, footsteps):
        """Check if footsteps maintain balance"""
        # Simple balance check - in practice this would be more complex
        # considering ZMP, CoM, and dynamic balance
        if len(footsteps) < 2:
            return True

        # Check that footsteps are within reasonable bounds
        for i in range(1, len(footsteps)):
            prev_pos = footsteps[i-1].pose.position
            curr_pos = footsteps[i].pose.position

            dx = curr_pos.x - prev_pos.x
            dy = curr_pos.y - prev_pos.y
            step_dist = np.sqrt(dx*dx + dy*dy)

            if step_dist > self.max_step_length + self.zmp_margin:
                return False

        return True

def main(args=None):
    rclpy.init(args=args)
    node = FootstepPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Balance-Aware Path Planning

### Zero-Moment Point (ZMP) Considerations

For stable humanoid walking, path planning must consider ZMP constraints:

- **ZMP Definition**: Point where the moment of the ground reaction force is zero
- **Stability Region**: Area where ZMP must remain for stable walking
- **Foot Support Region**: Convex hull of supporting feet

### Balance-Aware Planning Strategies

1. **Kinematic Constraints**: Limit path curvature based on balance
2. **Dynamic Stability**: Consider robot dynamics during planning
3. **Step Timing**: Coordinate path following with gait timing
4. **Recovery Planning**: Plan for balance recovery scenarios

### Center of Mass (CoM) Management

CoM management during navigation:

- **CoM Trajectory**: Plan CoM movement for stable transitions
- **Capture Point**: Ensure CoM remains within stable region
- **Walking Patterns**: Coordinate CoM with stepping patterns

## Behavior Trees for Navigation

### Behavior Tree Concepts

Behavior trees provide a flexible framework for navigation decision-making:

- **Nodes**: Actions, conditions, and control flow
- **Composites**: Sequences, selectors, and decorators
- **Blackboard**: Shared memory for node communication

### Nav2 Behavior Trees

Nav2 uses behavior trees for:

- **Action Sequencing**: Execute navigation actions in sequence
- **Recovery Behaviors**: Handle navigation failures
- **Condition Checking**: Verify preconditions before actions
- **Fallback Mechanisms**: Alternative strategies when primary fails

### Humanoid-Specific Behaviors

Custom behaviors for humanoid navigation:

- **Balance Check**: Verify stability before navigation
- **Footstep Validation**: Ensure footstep plan is executable
- **Gait Transition**: Switch between different walking patterns
- **Terrain Assessment**: Evaluate terrain for safe navigation

### Example: Humanoid Navigation Behavior Tree

```xml
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <ReactiveSequence name="NavigateWithRecovery">
      <!-- Check if robot is in stable state -->
      <CheckBalance name="CheckRobotBalance"/>

      <!-- Generate footstep plan -->
      <GenerateFootsteps name="PlanFootsteps"/>

      <!-- Validate footstep plan -->
      <ValidateFootsteps name="ValidateFootsteps"/>

      <!-- Execute navigation with recovery -->
      <ReactiveFallback name="NavigateWithRecovery">
        <PipelineSequence name="Navigate">
          <ComputePathToPose goal="{goal}" path="{path}"/>
          <FollowPath path="{path}" controller="{FollowPath}"/>
        </PipelineSequence>

        <ReactiveSequence name="Recovery">
          <!-- Clear costmap -->
          <ClearEntireCostmap name="ClearLocalCostmap" service_name="local_costmap/clear_entirely_local_costmap"/>
          <ClearEntireCostmap name="ClearGlobalCostmap" service_name="global_costmap/clear_entirely_global_costmap"/>

          <!-- Try alternative navigation -->
          <Spin name="Spin" angle="1.57"/>
          <Wait name="Wait" wait_duration="2"/>
        </ReactiveSequence>
      </ReactiveFallback>
    </ReactiveSequence>
  </BehaviorTree>

  <BehaviorTree ID="NavigateWristReplanning">
    <ReactiveSequence name="NavigateWithReplanning">
      <CheckBalance name="CheckRobotBalance"/>
      <GenerateFootsteps name="PlanFootsteps"/>
      <PipelineSequence name="MainNavigate">
        <ComputePathToPose goal="{goal}" path="{path}"/>
        <FollowPath path="{path}" controller="{FollowPath}"/>
        <IsPathValid path="{path}" error_angle_th="0.5" error_distance_th="2.0"/>
      </PipelineSequence>
    </ReactiveSequence>
  </BehaviorTree>
</root>
```

## Chapter Summary

In this chapter, we've covered Nav2 path planning for bipedal humanoid robots:
- The fundamentals of Navigation 2 and its architecture
- Unique challenges of bipedal navigation including balance considerations
- Customization techniques for humanoid-specific navigation
- Footstep planning integration with Nav2
- Balance-aware path planning strategies
- Behavior trees for navigation decision-making

## References and Citations

- [ROS 2 Navigation Documentation](https://navigation.ros.org/) - Official Nav2 documentation
- [NVIDIA Isaac ROS Navigation](https://isaac-ros.github.io/) - Isaac ROS navigation packages
- [Humanoid Robot Navigation Research](https://ieeexplore.ieee.org/) - Peer-reviewed research on humanoid navigation
- [NVIDIA Developer Resources](https://developer.nvidia.com/) - Additional resources for Isaac ecosystem

## Knowledge Check

1. What are the key challenges of bipedal navigation compared to wheeled robots?
2. How does footstep planning integrate with the Nav2 stack?
3. What are ZMP and CoM considerations in humanoid navigation?

## Hands-on Exercise

### Exercise: Nav2 Configuration for Humanoid Robot

**Objective**: Configure Nav2 for a humanoid robot simulation and test navigation with balance considerations.

**Prerequisites**:
- Isaac Sim with humanoid robot model
- Nav2 packages installed
- Basic understanding of ROS 2 navigation

**Steps**:
1. Set up a humanoid robot model in Isaac Sim
2. Configure Nav2 with humanoid-specific parameters (footprint, velocities, etc.)
3. Implement a simple footstep planning node
4. Test navigation in a simple environment
5. Verify that the robot maintains balance during navigation
6. Adjust parameters to improve navigation performance

**Expected Outcome**:
- Successfully configured Nav2 for humanoid navigation
- Implemented basic footstep planning
- Demonstrated stable navigation with balance considerations
- Tuned parameters for optimal performance

**Troubleshooting**:
- If navigation is unstable, check velocity limits and footstep planning
- If robot falls during navigation, verify balance constraints
- If path planning fails, ensure proper costmap configuration

**Next Steps**:
- Implement more advanced footstep planning algorithms
- Add terrain adaptation capabilities
- Integrate with perception for dynamic obstacle avoidance

---

**Learning Objective Achieved**: You now understand Nav2 customization for bipedal humanoid robots, footstep planning integration, balance-aware path planning, and behavior trees for navigation decision-making.