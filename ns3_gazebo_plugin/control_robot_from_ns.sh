#!/bin/bash

# Control Gazebo robot from network namespace using ROS2
# This script publishes ROS2 cmd_vel topics that are relayed to Gazebo
# via the relay_bridge.py running on the host.
#
# Network Path:
#   This script (in nns) → ROS2 /robotN/cmd_vel → Direct Network (10.128.0.x)
#                            ↓
#   relay_bridge.py (host)  → Gazebo Transport /model/robotN/cmd_vel
#                            ↓
#   Gazebo Simulation       → Robot moves
#
# Usage:
#   1. Run from within a network namespace:
#      sudo ip netns exec nns1 ./control_robot_from_ns.sh robot1 forward
#
#   2. Or enter the namespace first:
#      sudo ip netns exec nns1 bash
#      source /opt/ros/jazzy/setup.bash
#      ./control_robot_from_ns.sh robot1 forward

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <robot_name> <command> [speed]"
    echo ""
    echo "Commands:"
    echo "  forward [speed]     Move forward (default: 0.5 m/s)"
    echo "  backward [speed]    Move backward (default: 0.5 m/s)"
    echo "  left [speed]        Rotate left (default: 0.5 rad/s)"
    echo "  right [speed]       Rotate right (default: 0.5 rad/s)"
    echo "  stop                Stop the robot"
    echo "  custom <x> <z>      Custom velocity (linear_x, angular_z)"
    echo ""
    echo "Examples:"
    echo "  # From within namespace:"
    echo "  ./control_robot_from_ns.sh robot1 forward"
    echo "  ./control_robot_from_ns.sh robot1 left 0.3"
    echo "  ./control_robot_from_ns.sh robot2 custom 0.5 0.2"
    echo ""
    echo "  # From outside namespace:"
    echo "  sudo ip netns exec nns1 bash -c 'source /opt/ros/jazzy/setup.bash && ./control_robot_from_ns.sh robot1 forward'"
    exit 1
fi

ROBOT_NAME=$1
COMMAND=$2
SPEED=${3:-0.5}

# Detect which namespace we're in by checking IP addresses
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CURRENT_IP=$(ip addr show | grep -oP '10\.128\.0\.\d+' | head -1)

# Set FastDDS configuration based on namespace
if [ "$CURRENT_IP" == "10.128.0.2" ]; then
    # We're in nns1
    export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_nns1.xml"
    echo "[nns1] Using FastDDS config: fastdds_nns1.xml"
elif [ "$CURRENT_IP" == "10.128.0.10" ]; then
    # We're in nns2
    export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_nns2.xml"
    echo "[nns2] Using FastDDS config: fastdds_nns2.xml"
else
    echo "Warning: Could not detect namespace (IP: $CURRENT_IP)"
    echo "FastDDS static discovery may not work!"
fi

# Set RMW implementation to FastRTPS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source ROS2 if not already sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS2..."
    source /opt/ros/jazzy/setup.bash
fi

# Determine velocity values
case $COMMAND in
    forward)
        LINEAR_X=$SPEED
        ANGULAR_Z=0.0
        echo "[$ROBOT_NAME] Moving forward at $LINEAR_X m/s"
        ;;

    backward)
        LINEAR_X=$(echo "-1 * $SPEED" | bc -l)
        ANGULAR_Z=0.0
        echo "[$ROBOT_NAME] Moving backward at speed $SPEED m/s"
        ;;

    left)
        LINEAR_X=0.0
        ANGULAR_Z=$SPEED
        echo "[$ROBOT_NAME] Rotating left at $ANGULAR_Z rad/s"
        ;;

    right)
        LINEAR_X=0.0
        ANGULAR_Z=$(echo "-1 * $SPEED" | bc -l)
        echo "[$ROBOT_NAME] Rotating right at speed $SPEED rad/s"
        ;;

    stop)
        LINEAR_X=0.0
        ANGULAR_Z=0.0
        echo "[$ROBOT_NAME] Stopping"
        ;;

    custom)
        if [ "$#" -ne 4 ]; then
            echo "Error: custom command requires <linear_x> <angular_z>"
            exit 1
        fi
        LINEAR_X=$3
        ANGULAR_Z=$4
        echo "[$ROBOT_NAME] Custom velocity: linear.x=$LINEAR_X, angular.z=$ANGULAR_Z"
        ;;

    *)
        echo "Unknown command: $COMMAND"
        exit 1
        ;;
esac

# ROS2 topic name
ROS2_TOPIC="/${ROBOT_NAME}/cmd_vel"

# Construct Twist message
TWIST_MSG="{linear: {x: $LINEAR_X, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $ANGULAR_Z}}"

# Publish to ROS2 topic
echo "Publishing to $ROS2_TOPIC..."
echo "Message: $TWIST_MSG"

ros2 topic pub --once $ROS2_TOPIC geometry_msgs/msg/Twist "$TWIST_MSG"

if [ $? -eq 0 ]; then
    echo "✓ Command sent via ROS2 (Direct Network)"
    echo "  → Relay bridge on host will forward to Gazebo"
    echo "  → $ROBOT_NAME should move in Gazebo simulation"
else
    echo "✗ Failed to publish ROS2 message"
    echo ""
    echo "Troubleshooting:"
    echo "  1. Is relay_bridge.py running on the host?"
    echo "  2. Is direct network configured?"
    echo "     Check: ip addr show direct_vethn*"
    echo "  3. Can you reach the host?"
    echo "     Try: ping 10.128.0.1 (for nns1) or ping 10.128.0.9 (for nns2)"
    exit 1
fi
