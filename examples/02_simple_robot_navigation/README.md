# Simple Robot Navigation Example

## Overview

This example demonstrates the integration of ROS2 navigation, Gazebo physics simulation, and NS-3 network simulation. A differential drive robot navigates in a simple environment while communicating over a WiFi network.

## Learning Objectives

- Understand ROS2 navigation integration with Gazebo
- Learn how robots communicate over WiFi networks
- See the interaction between physical movement and network performance
- Observe real-time visualization of both robot and network status

## Scenario Description

- **Robot**: Differential drive robot with WiFi capability
- **Environment**: Simple indoor environment with obstacles
- **Navigation**: Autonomous navigation to goal positions
- **Network**: WiFi communication for status updates and control
- **Duration**: Continuous operation until goal reached

## Files Structure

```
02_simple_robot_navigation/
├── README.md                    # This file
├── robot_navigation.launch.py  # ROS2 launch file
├── robot_world.world           # Gazebo world with robot and environment
├── config/
│   ├── robot_config.yaml       # Robot parameters
│   ├── nav_params.yaml         # Navigation parameters
│   └── wifi_config.yaml        # WiFi network configuration
├── urdf/
│   └── diff_drive_robot.urdf   # Robot description
├── maps/
│   ├── simple_map.yaml         # Map configuration
│   └── simple_map.pgm          # Map image file
├── src/
│   ├── robot_controller.py     # Robot control node
│   ├── network_monitor.py      # Network monitoring node
│   └── goal_sender.py          # Goal publishing node
├── build.sh                    # Build script
├── run.sh                      # Run script
└── clean.sh                    # Cleanup script
```

## Prerequisites

- Complete NS3-Gazebo installation
- ROS2 navigation stack installed
- Basic understanding of ROS2 concepts
- Familiarity with robot navigation principles

## Quick Start

1. **Build the example**:
   ```bash
   cd examples/02_simple_robot_navigation
   ./build.sh
   ```

2. **Run the simulation**:
   ```bash
   ./run.sh
   ```

3. **Expected behavior**:
   - Robot spawns in Gazebo environment
   - Navigation system initializes
   - Robot autonomously navigates to goal
   - Network status displayed in real-time

## Detailed Components

### Robot Description (URDF)

The robot is a simple differential drive robot with:
- **Base**: Cylindrical chassis
- **Wheels**: Two drive wheels and one caster wheel
- **Sensors**: Lidar for navigation
- **WiFi**: Network interface for communication

### Navigation System

Uses ROS2 navigation stack with:
- **SLAM**: Simultaneous Localization and Mapping
- **Path Planning**: Global and local planners
- **Obstacle Avoidance**: Dynamic obstacle handling
- **Localization**: AMCL for position estimation

### Network Integration

WiFi network provides:
- **Status Updates**: Robot position and status
- **Control Commands**: Goal positions and navigation commands
- **Sensor Data**: Lidar and camera data transmission
- **Performance Monitoring**: Network latency and throughput

## Usage Instructions

### Basic Operation

1. **Start the system**:
   ```bash
   ./run.sh
   ```

2. **Set navigation goals**:
   - Use RViz2 to set navigation goals
   - Or use the goal_sender.py script for automated goals

3. **Monitor performance**:
   - Watch robot movement in Gazebo
   - Observe network metrics in terminal
   - Check navigation status in RViz2

### Advanced Usage

#### Custom Goal Sequences

```python
# goals.py - Define custom navigation goals
goals = [
    {"x": 2.0, "y": 1.0, "theta": 0.0},
    {"x": 4.0, "y": 3.0, "theta": 1.57},
    {"x": 1.0, "y": 4.0, "theta": 3.14}
]
```

#### Network Performance Testing

```bash
# Run with network analysis
./run.sh --network-analysis

# Test different WiFi configurations
./run.sh --wifi-config config/wifi_test.yaml
```

#### Multi-Robot Scenarios

```bash
# Launch multiple robots
./run.sh --robots 3
```

## Configuration Options

### Robot Parameters (config/robot_config.yaml)

```yaml
robot:
  # Physical parameters
  wheel_radius: 0.1        # meters
  wheel_separation: 0.3    # meters
  max_linear_velocity: 1.0 # m/s
  max_angular_velocity: 2.0 # rad/s

  # Sensor configuration
  lidar:
    range_min: 0.1
    range_max: 10.0
    angle_min: -3.14159
    angle_max: 3.14159
    angle_increment: 0.01745

  # WiFi interface
  wifi:
    interface: "wlan0"
    ssid: "robot_network"
    channel: 6
```

### Navigation Parameters (config/nav_params.yaml)

```yaml
navigation:
  # Global planner
  global_planner:
    plugin: "nav2_navfn_planner/NavfnPlanner"
    tolerance: 0.5
    use_astar: false

  # Local planner
  local_planner:
    plugin: "dwa_local_planner/DWAPlannerROS"
    max_vel_x: 0.5
    min_vel_x: 0.1
    max_vel_theta: 1.0

  # Costmap parameters
  costmap:
    inflation_radius: 0.6
    cost_scaling_factor: 10.0
```

## Monitoring and Debugging

### Network Performance

The network monitor displays:
- **Latency**: Round-trip time for packets
- **Throughput**: Data transmission rate
- **Packet Loss**: Percentage of lost packets
- **Signal Strength**: WiFi signal quality

### Navigation Status

Navigation monitor shows:
- **Current Position**: Robot's current pose
- **Goal Status**: Progress toward current goal
- **Path Quality**: Path planning effectiveness
- **Obstacle Detection**: Real-time obstacle information

### Debug Commands

```bash
# Check robot status
ros2 topic echo /robot_state

# Monitor navigation
ros2 topic echo /move_base/status

# View network metrics
ros2 topic echo /network_status

# Debug transforms
ros2 run tf2_tools view_frames.py
```

## Customization Examples

### Modifying Robot Behavior

```python
# In robot_controller.py
class RobotController:
    def __init__(self):
        # Adjust navigation parameters
        self.goal_tolerance = 0.2  # meters
        self.speed_factor = 0.8    # 80% of max speed

    def navigate_to_goal(self, goal):
        # Custom navigation logic
        pass
```

### Adding Sensors

```xml
<!-- In robot.urdf -->
<gazebo reference="base_link">
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.57</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
  </sensor>
</gazebo>
```

### Network Simulation

```cpp
// Custom NS-3 network topology
void SetupRobotNetwork() {
    // Create WiFi access point
    NodeContainer apNode;
    apNode.Create(1);

    // Create robot nodes
    NodeContainer robotNodes;
    robotNodes.Create(3);

    // Setup WiFi network
    WifiHelper wifi;
    // ... configuration
}
```

## Performance Metrics

### Expected Performance

- **Navigation Accuracy**: ±10cm position accuracy
- **Path Planning Time**: <1 second for 10m paths
- **Network Latency**: <50ms for local WiFi
- **Update Rate**: 10Hz for all sensors and status

### Optimization Tips

1. **Reduce Sensor Data**: Lower lidar resolution for better network performance
2. **Optimize Path Planning**: Adjust costmap parameters for faster planning
3. **Network Tuning**: Configure WiFi parameters for your environment
4. **Computational Load**: Balance simulation fidelity with real-time performance

## Troubleshooting

### Common Issues

1. **Robot doesn't move**:
   - Check navigation stack initialization
   - Verify goal publication
   - Ensure map is loaded correctly

2. **Poor navigation performance**:
   - Adjust costmap parameters
   - Check sensor data quality
   - Verify localization accuracy

3. **Network connectivity issues**:
   - Check WiFi configuration
   - Verify NS-3 network setup
   - Monitor signal strength

4. **Simulation runs slowly**:
   - Reduce Gazebo physics step size
   - Lower sensor update rates
   - Optimize computer resources

### Debug Procedures

```bash
# Check ROS2 nodes
ros2 node list

# Verify topics
ros2 topic list

# Check transforms
ros2 run tf2_tools view_frames.py

# Monitor CPU usage
htop

# Check network interfaces
ip link show
```

## Learning Progression

### Beginner Tasks

1. Run the basic example
2. Set manual navigation goals
3. Observe network performance changes
4. Modify robot speed parameters

### Intermediate Tasks

1. Add custom obstacles to the environment
2. Implement custom navigation behaviors
3. Create multi-goal navigation sequences
4. Analyze network performance data

### Advanced Tasks

1. Implement multi-robot coordination
2. Add advanced sensors (cameras, IMU)
3. Create dynamic environment changes
4. Optimize network protocols

## Next Steps

After completing this example:

1. **WiFi Performance Testing**: Progress to example 03
2. **Multi-Robot Systems**: Explore coordination algorithms
3. **Real Hardware**: Deploy to physical robots
4. **Advanced Mapping**: Implement SLAM algorithms

## Related Documentation

- [ROS2 Navigation Documentation](https://navigation.ros.org/)
- [Gazebo Robot Simulation](http://gazebosim.org/)
- [NS-3 WiFi Models](https://www.nsnam.org/docs/models/html/wifi.html)

## Support

For help with this example:
- Check ROS2 navigation troubleshooting guides
- Review Gazebo robot simulation tutorials
- Ask questions in GitHub Discussions