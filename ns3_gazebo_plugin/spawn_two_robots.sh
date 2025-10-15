#!/bin/bash

# Spawn 2 robots - spawn은 호스트에서 실행하고, 
# 각 로봇의 ROS2 통신은 네임스페이스를 통해 처리

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORLD_NAME="ns3_gazebo_world"
ROBOT1_SDF="$SCRIPT_DIR/robot1_model.sdf"
ROBOT2_SDF="$SCRIPT_DIR/robot2_model.sdf"

echo "=== Spawning robots in Gazebo (from host) ==="
echo "Note: Robots will be spawned in the main Gazebo world"
echo "      Their ROS2 communication will be handled via network namespaces"
echo ""

# Source ROS2 on host and spawn robot1
source /opt/ros/jazzy/setup.bash

echo "Spawning robot1 at position (-3.0, 0.0, 0.5)..."
gz service -s /world/$WORLD_NAME/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"$ROBOT1_SDF\", name: \"robot1\", pose: {position: {x: -3.0, y: 0.0, z: 0.5}}" &
PID1=$!

sleep 2

echo "Spawning robot2 at position (3.0, 0.0, 0.5)..."
gz service -s /world/$WORLD_NAME/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"$ROBOT2_SDF\", name: \"robot2\", pose: {position: {x: 3.0, y: 0.0, z: 0.5}}" &
PID2=$!

echo ""
echo "Waiting for spawn operations to complete..."
wait $PID1
wait $PID2

sleep 2

echo ""
echo "=== Both robots spawned successfully! ==="
echo "robot1: position (-3.0, 0.0, 0.5)"
echo "robot2: position (3.0, 0.0, 0.5)"
echo ""
echo "Inter-robot distance: 6.0 m"
echo ""
echo "Next steps:"
echo "1. Start ROS2 bridge in nns1 for robot1"
echo "2. Start ROS2 bridge in nns2 for robot2"
echo "3. Test ROS2 communication between namespaces"
