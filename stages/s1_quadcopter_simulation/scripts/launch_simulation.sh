#!/bin/bash

# Stage 1: Quadcopter Simulation Launch Script
# This script launches the complete simulation environment

set -e

echo "🚁 Starting Quadcopter Simulation..."

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 environment not sourced. Please run:"
    echo "source /opt/ros/humble/setup.bash"
    exit 1
fi

# Check if Gazebo is available
if ! command -v gazebo &> /dev/null; then
    echo "Error: Gazebo not found. Please install Gazebo Fortress."
    exit 1
fi

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Set environment variables
export GAZEBO_MODEL_PATH="$PROJECT_DIR/models:$GAZEBO_MODEL_PATH"
export GAZEBO_RESOURCE_PATH="$PROJECT_DIR/worlds:$GAZEBO_RESOURCE_PATH"

echo "📁 Project directory: $PROJECT_DIR"
echo "🌍 Gazebo model path: $GAZEBO_MODEL_PATH"
echo "🌍 Gazebo resource path: $GAZEBO_RESOURCE_PATH"

# Build the workspace if needed
if [ ! -d "$PROJECT_DIR/build" ]; then
    echo "🔨 Building workspace..."
    cd "$PROJECT_DIR"
    colcon build --symlink-install
fi

# Source the workspace
source "$PROJECT_DIR/install/setup.bash"

echo "🚀 Launching simulation..."

# Launch the simulation
ros2 launch quadcopter_controller simulation.launch.py

echo "✅ Simulation launched successfully!"
echo ""
echo "🎮 Control the quadcopter using:"
echo "   - Keyboard: Use the teleop_twist_keyboard"
echo "   - Topics: Publish to /quadcopter/control/velocity"
echo "   - RViz: Visualize the quadcopter state"
echo ""
echo "📊 Monitor topics:"
echo "   - ros2 topic echo /quadcopter/sensors/imu"
echo "   - ros2 topic echo /quadcopter/sensors/gps"
echo "   - ros2 topic echo /quadcopter/state/odometry" 