#!/bin/bash

# ROS 2 Workspace Setup Script for Humanoid Robotics
# This script creates a basic ROS 2 workspace structure and configuration

# Create the workspace directory structure
mkdir -p ros2_ws/src
cd ros2_ws

# Create the basic workspace files
touch src/COLCON_IGNORE
touch .rosinstall

# Create a basic setup file
cat > setup_ws.sh << 'EOF'
#!/bin/bash
# Setup script for ROS 2 Humble workspace

# Source ROS 2 installation
source /opt/ros/humble/setup.bash

# Source the workspace if it exists
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

# Set environment variables for humanoid robotics
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp
export ROS_DOMAIN_ID=42

echo "ROS 2 Humble workspace configured for humanoid robotics"
echo "Workspace directory: $(pwd)"
EOF

chmod +x setup_ws.sh

# Create a README for the workspace
cat > README.md << 'EOF'
# ROS 2 Workspace for Humanoid Robotics

This workspace contains packages for humanoid robotics development using ROS 2 Humble.

## Setup

1. Source the ROS 2 installation: `source /opt/ros/humble/setup.bash`
2. Navigate to this workspace directory
3. Build the workspace: `colcon build`
4. Source the workspace: `source install/setup.bash`

## Structure
- `src/` - Source packages for humanoid robotics
- `build/` - Build artifacts (created after building)
- `install/` - Installation directory (created after building)
- `log/` - Build logs (created after building)

## Packages
Add your humanoid robotics packages to the `src/` directory.
EOF

echo "ROS 2 workspace structure created in ros2_ws/"
echo "To use: cd ros2_ws && source setup_ws.sh"