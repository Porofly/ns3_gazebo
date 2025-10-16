#!/bin/bash
#
# Run forward_follower_robot node in network namespace nns2
#
# This node:
# - Moves forward at 1 m/s when communication is available
# - Follows leader's rotation (angular.z) when communication is available
# - STOPS COMPLETELY (linear.x=0, angular.z=0) on communication timeout
# Note: Differential drive cannot move in Y direction (lateral movement)
#
# Network Configuration:
#   Direct Network (10.128.0.10): For cmd_vel to Gazebo
#   WiFi Network (10.0.0.4): For receiving /robot1/state from leader (via NS-3)
#
# Usage:
#   sudo ip netns exec nns2 bash /path/to/run_forward_follower_nns2.sh
#
#   Or enter namespace first:
#   sudo ip netns exec nns2 bash
#   bash /path/to/run_forward_follower_nns2.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo " Forward Follower Robot (nns2)"
echo "=========================================="
echo ""

# Check if we're in nns2
CURRENT_IP=$(ip addr show | grep -oP '10\.128\.0\.\d+' | head -1)
if [ "$CURRENT_IP" != "10.128.0.10" ]; then
    echo "Error: Not running in nns2 namespace!"
    echo "Current Direct Network IP: $CURRENT_IP (expected: 10.128.0.10)"
    echo ""
    echo "Run this script with:"
    echo "  sudo ip netns exec nns2 bash $0"
    exit 1
fi

echo "✓ Running in nns2 (Direct IP: $CURRENT_IP)"

# Set FastDDS configuration for nns2
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_nns2.xml"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "✓ FastDDS configuration: fastdds_nns2.xml"
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
echo "  Direct Network: 10.128.0.10 (cmd_vel)"
echo "  WiFi Network: 10.0.0.4 (receiving /robot1/state via NS-3)"
echo ""
echo "Robot Behavior:"
echo "  When CONNECTED:"
echo "    - Linear X: 1.0 m/s (forward)"
echo "    - Angular Z: Follows leader's rotation"
echo "  When TIMEOUT (>1.0s):"
echo "    - Linear X: 0.0 m/s (STOPPED)"
echo "    - Angular Z: 0.0 rad/s (STOPPED)"
echo "  Note: DiffDrive cannot move in Y direction"
echo ""
echo "Topics:"
echo "  Subscribing: /robot1/state (WiFi ← NS-3 ← leader robot)"
echo "  Publishing: /robot2/cmd_vel (Direct → relay_bridge → Gazebo)"
echo ""
echo "Starting forward_follower_robot..."
echo "=========================================="
echo ""

# Run forward follower robot
ros2 run forward_follower_robot forward_follower_node
