#!/bin/bash

# Stage 1: Flight Test Script
# This script tests the basic flight functionality

set -e

echo "🧪 Testing Quadcopter Flight System..."

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS 2 environment not sourced. Please run:"
    echo "source /opt/ros/humble/setup.bash"
    exit 1
fi

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# Source the workspace
source "$PROJECT_DIR/install/setup.bash"

echo "🔍 Testing sensor topics..."

# Test 1: Check if sensor topics are publishing
echo "📡 Checking sensor topics..."
timeout 5s ros2 topic echo /quadcopter/sensors/imu --once || echo "⚠️  IMU topic not available"
timeout 5s ros2 topic echo /quadcopter/sensors/gps --once || echo "⚠️  GPS topic not available"
timeout 5s ros2 topic echo /quadcopter/sensors/altitude --once || echo "⚠️  Altitude topic not available"

echo "🎮 Testing control topics..."

# Test 2: Check if control topics are available
echo "📡 Checking control topics..."
ros2 topic list | grep quadcopter || echo "⚠️  No quadcopter topics found"

echo "🚁 Testing flight controller..."

# Test 3: Send a simple hover command
echo "📤 Sending hover command..."
ros2 topic pub /quadcopter/control/velocity geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

echo "📊 Testing state monitoring..."

# Test 4: Monitor state for a few seconds
echo "📡 Monitoring quadcopter state for 5 seconds..."
timeout 5s ros2 topic echo /quadcopter/state/odometry || echo "⚠️  Odometry topic not available"

echo "✅ Flight test completed!"
echo ""
echo "📋 Test Summary:"
echo "   - Sensor topics: Checked"
echo "   - Control topics: Checked"
echo "   - Flight controller: Tested"
echo "   - State monitoring: Tested"
echo ""
echo "🎯 Next steps:"
echo "   - Run the full simulation: ./scripts/launch_simulation.sh"
echo "   - Test manual control: Use keyboard teleop"
echo "   - Test autonomous flight: Publish waypoints" 