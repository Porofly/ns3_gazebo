#!/bin/bash

# Start Dual Network Multi-Robot Simulation
#
# This script sets up and starts a complete simulation with:
# - Dual network architecture (WiFi + Direct)
# - Gazebo simulation with NS-3 plugin
# - Multiple robots
# - ROS2-Gazebo relay bridge
#
# Network Architecture:
#
#   ┌─────────────────────────────────────────────────────────┐
#   │  Namespace (nns1, nns2, ...)                             │
#   │                                                           │
#   │  WiFi (10.0.0.x/9)  ← NS-3 WiFi →  Robot-to-Robot comm  │
#   │  Direct (10.128.x.x/29) ← veth → Host (Robot control)    │
#   └─────────────────────────────────────────────────────────┘
#
# Usage:
#   ./start_dual_network.sh [num_robots]
#
# Example:
#   ./start_dual_network.sh 2

set -e  # Exit on error

NUM_ROBOTS=${1:-2}
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo " Dual Network Multi-Robot Simulation"
echo "=========================================="
echo ""
echo "Configuration:"
echo "  Number of robots: $NUM_ROBOTS"
echo "  WiFi Network: 10.0.0.0/9 (NS-3 simulated)"
echo "  Direct Network: 10.128.0.0/29+ (Host connection)"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run with sudo"
    echo "Usage: sudo ./start_dual_network.sh [$NUM_ROBOTS]"
    exit 1
fi

# Step 1: Setup network namespaces with direct connection
echo "Step 1: Setting up network namespaces..."
echo "  Creating $NUM_ROBOTS namespaces with WiFi + Direct networks"
cd "$SCRIPT_DIR/../scripts"

if python3 nns_setup.py setup -c $NUM_ROBOTS --include_direct; then
    echo "  ✓ Network namespaces configured"
else
    echo "  ✗ Failed to setup namespaces"
    exit 1
fi

cd "$SCRIPT_DIR"
echo ""

# Step 2: Check if Gazebo is already running
echo "Step 2: Checking Gazebo status..."
if pgrep -x "gz" > /dev/null; then
    echo "  ! Gazebo is already running"
    echo "  ! Using existing Gazebo instance"
    echo "  ! If you want a fresh start, please stop Gazebo manually first:"
    echo "    pkill -9 gz"
else
    echo "  Gazebo is not running"
    echo "  Please start Gazebo manually in a separate terminal:"
    echo ""
    echo "    cd $SCRIPT_DIR"
    echo "    export GZ_SIM_SYSTEM_PLUGIN_PATH=\"\$(pwd)/build\""
    echo "    gz sim ns3_gazebo_ros2.sdf"
    echo ""
    echo "  Press Enter when Gazebo is ready..."
    read
fi

echo ""

# Step 3: Spawn robots
echo "Step 3: Spawning robots in Gazebo..."
if [ -f "$SCRIPT_DIR/spawn_two_robots.sh" ] && [ $NUM_ROBOTS -eq 2 ]; then
    if bash "$SCRIPT_DIR/spawn_two_robots.sh"; then
        echo "  ✓ Robots spawned"
    else
        echo "  ✗ Failed to spawn robots"
        exit 1
    fi
else
    echo "  ! Manual robot spawning required for $NUM_ROBOTS robots"
    echo "  ! Please spawn robots manually"
    echo "  Press Enter when robots are spawned..."
    read
fi

echo ""

# Step 4: Start relay bridge with FastDDS
echo "Step 4: Starting ROS2-Gazebo relay bridge with FastDDS..."
echo "  This bridge uses static peer discovery to connect with namespaces"

# Generate robot names
ROBOT_NAMES=""
for i in $(seq 1 $NUM_ROBOTS); do
    ROBOT_NAMES="$ROBOT_NAMES robot$i"
done

echo "  Robots: $ROBOT_NAMES"
echo "  FastDDS Static Peers:"
echo "    nns1: 10.128.0.2:7400"
echo "    nns2: 10.128.0.10:7400"
echo "  Starting bridge in background..."

# Start relay bridge with FastDDS configuration
"$SCRIPT_DIR/start_relay_bridge.sh" $ROBOT_NAMES > /tmp/relay_bridge.log 2>&1 &
BRIDGE_PID=$!

# Wait a moment for bridge to start
sleep 2

if ps -p $BRIDGE_PID > /dev/null; then
    echo "  ✓ Relay bridge started (PID: $BRIDGE_PID)"
    echo "  Log file: /tmp/relay_bridge.log"
else
    echo "  ✗ Failed to start relay bridge"
    echo "  Check log: /tmp/relay_bridge.log"
    exit 1
fi

echo ""

# Step 5: Show system status
echo "=========================================="
echo " System Ready!"
echo "=========================================="
echo ""
echo "Network Status:"
echo "  Namespaces: $(ip netns list | wc -l) created"
echo "  WiFi Network: 10.0.0.0/9 (NS-3 simulated)"
echo "  Direct Network: Configured (10.128.0.x/29)"
echo ""
echo "Running Processes:"
echo "  Gazebo: $(pgrep -x gz > /dev/null && echo 'Running' || echo 'Not found')"
echo "  Relay Bridge: Running (PID: $BRIDGE_PID)"
echo ""
echo "How to control robots:"
echo "  From namespace nns1 (robot1):"
echo "    sudo ip netns exec nns1 bash -c 'source /opt/ros/jazzy/setup.bash && cd $SCRIPT_DIR && ./control_robot_from_ns.sh robot1 forward'"
echo ""
echo "  From namespace nns2 (robot2):"
echo "    sudo ip netns exec nns2 bash -c 'source /opt/ros/jazzy/setup.bash && cd $SCRIPT_DIR && ./control_robot_from_ns.sh robot2 left'"
echo ""
echo "To test robot-to-robot communication (via NS-3 WiFi):"
echo "  Terminal 1 (nns1 - talker):"
echo "    sudo ip netns exec nns1 bash"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    ros2 run demo_nodes_cpp talker"
echo ""
echo "  Terminal 2 (nns2 - listener):"
echo "    sudo ip netns exec nns2 bash"
echo "    source /opt/ros/jazzy/setup.bash"
echo "    ros2 run demo_nodes_cpp listener"
echo ""
echo "To stop:"
echo "  kill $BRIDGE_PID  # Stop relay bridge"
echo "  cd ../scripts && sudo python3 nns_setup.py teardown -c $NUM_ROBOTS --include_direct"
echo ""
echo "=========================================="
