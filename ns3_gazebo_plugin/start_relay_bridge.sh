#!/bin/bash

# Start ROS2-Gazebo Relay Bridge with FastDDS Static Peers
#
# This script configures FastDDS to use static peer discovery over
# the Direct Network, allowing the bridge to communicate with ROS2 nodes
# in network namespaces.
#
# Usage:
#   ./start_relay_bridge.sh [robot1] [robot2] [robot3] ...
#
# Example:
#   ./start_relay_bridge.sh robot1 robot2

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTDDS_CONFIG="$SCRIPT_DIR/fastdds_host.xml"

# Check if FastDDS config exists
if [ ! -f "$FASTDDS_CONFIG" ]; then
    echo "Error: FastDDS configuration file not found: $FASTDDS_CONFIG"
    exit 1
fi

# Default robots if not specified
if [ "$#" -eq 0 ]; then
    ROBOTS="robot1 robot2"
else
    ROBOTS="$@"
fi

echo "=========================================="
echo " ROS2-Gazebo Relay Bridge with FastDDS"
echo "=========================================="
echo ""
echo "Configuration:"
echo "  FastDDS Profile: $FASTDDS_CONFIG"
echo "  RMW Implementation: rmw_fastrtps_cpp"
echo "  Robots: $ROBOTS"
echo ""
echo "Static Peers (Direct Network):"
echo "  nns1: 10.128.0.2:7400"
echo "  nns2: 10.128.0.10:7400"
echo ""
echo "Starting bridge..."
echo ""

# Set FastDDS environment variables
export FASTRTPS_DEFAULT_PROFILES_FILE="$FASTDDS_CONFIG"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Run relay bridge
python3 "$SCRIPT_DIR/relay_bridge.py" $ROBOTS
