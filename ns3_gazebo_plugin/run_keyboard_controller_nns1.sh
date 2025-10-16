#!/bin/bash
#
# Run keyboard_controller node in network namespace nns1
#
# This script:
# 1. Sets up FastDDS static peers configuration for nns1
# 2. Sources ROS2 and workspace
# 3. Runs keyboard_controller node
#
# Network Configuration:
#   Direct Network (10.128.0.2): For cmd_vel and odometry with host
#   WiFi Network (10.0.0.1): For /robot1/state to other robots (via NS-3)
#
# Usage:
#   sudo ip netns exec nns1 bash /path/to/run_keyboard_controller_nns1.sh
#
#   Or enter namespace first:
#   sudo ip netns exec nns1 bash
#   bash /path/to/run_keyboard_controller_nns1.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo " Keyboard Controller (nns1)"
echo "=========================================="
echo ""

# Check if we're in nns1
CURRENT_IP=$(ip addr show | grep -oP '10\.128\.0\.\d+' | head -1)
if [ "$CURRENT_IP" != "10.128.0.2" ]; then
    echo "Error: Not running in nns1 namespace!"
    echo "Current Direct Network IP: $CURRENT_IP (expected: 10.128.0.2)"
    echo ""
    echo "Run this script with:"
    echo "  sudo ip netns exec nns1 bash $0"
    exit 1
fi

echo "✓ Running in nns1 (Direct IP: $CURRENT_IP)"

# Set FastDDS configuration for nns1
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_nns1.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "✓ FastDDS configuration: fastdds_nns1.xml"
echo "✓ RMW Implementation: $RMW_IMPLEMENTATION"
echo ""

# Source ROS2
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2 Jazzy..."
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace
ROS2_WS="/home/user/realgazebo/ns3_gazebo/ros2_ws"
if [ -f "$ROS2_WS/install/setup.bash" ]; then
    echo "Sourcing ros2_ws..."
    source "$ROS2_WS/install/setup.bash"
else
    echo "Error: ros2_ws not built!"
    echo "Please run: cd $ROS2_WS && colcon build"
    exit 1
fi

echo ""
echo "Network Configuration:"
echo "  Direct Network: 10.128.0.2 (cmd_vel, odometry)"
echo "  WiFi Network: 10.0.0.1 (/robot1/state via NS-3)"
echo ""
echo "Topics:"
echo "  Publishing: /robot1/cmd_vel (Direct → relay_bridge → Gazebo)"
echo "  Publishing: /robot1/state (WiFi → NS-3 → other robots)"
echo "  Subscribing: /robot1/odometry (relay_bridge → Direct)"
echo ""
echo "Starting keyboard_controller..."
echo "=========================================="
echo ""

# Run keyboard controller
ros2 run keyboard_controller keyboard_node
