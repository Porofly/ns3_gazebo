# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

NS3-Gazebo is a real-time network simulation system that bridges NS-3 discrete-event network simulator with Gazebo Harmonic physics simulation. It enables realistic WiFi network modeling for multi-robot systems by synchronizing robot positions from Gazebo to NS-3's propagation models and measuring actual network performance metrics (RSSI, SNR, throughput, packet loss).

**Core Integration**: Gazebo System Plugin (ns3_gazebo_world.cpp) runs NS-3 in a separate thread, updates NS-3 mobility models from Gazebo entity poses every frame, and monitors WiFi PHY layer statistics.

## Build Commands

### Build NS-3 Network Simulator
```bash
cd ns-allinone-3.45/ns-3.45
./ns3 configure --enable-examples --enable-tests
./ns3 build

# Verify build
./ns3 run hello-simulator
```

### Build NS-3 Gazebo Plugin
```bash
cd ns3_gazebo_plugin
mkdir -p build && cd build
cmake ..
make

# Verify build - should show libns3_gazebo_world.so and libhello_world.so
ls -la *.so
```

### Build ROS2 Workspace (Optional)
```bash
cd ns3_gazebo_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Rebuild After Code Changes
```bash
# Quick rebuild of plugin only
cd ns3_gazebo_plugin/build
make

# Full rebuild if changing NS-3 integration
cd ns3_gazebo_plugin
rm -rf build && mkdir build && cd build
cmake .. && make
```

## Running Simulations

### Basic Simulation (No Network Namespaces)
```bash
cd ns3_gazebo_plugin
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/build"
gz sim ns3_gazebo.sdf

# Control robot with arrow keys: ↑↓←→
# Console shows RSSI/SNR every ~2 seconds after packets start flowing
```

### Distributed Testbed (With Network Namespaces)
```bash
# Setup network namespaces (number of nodes)
cd scripts
sudo python3 nns_setup.py setup -c 2

# Verify namespaces created
ip netns list  # Should show: nns1, nns2

# Run testbed with CSV configuration
cd ../ns3_testbed
sudo python3 testbed_runner.py -c 2 -s csv_setup/example1.csv -v

# Cleanup network namespaces
cd ../scripts
sudo python3 nns_setup.py teardown -c 2
```

### Distance Experiment (Automated)
```bash
cd experiments
./run_distance_experiment.sh

# Outputs:
# - CSV data: ns3_gazebo_plugin/distance_experiment.csv
# - Analysis: experiments/results_distance/distance_experiment_report.txt
# - Plots: experiments/results_distance/*.png
```

## Architecture

### System Components

**Gazebo Simulation Layer**:
- Robot physics (differential drive)
- Collision detection
- Entity pose tracking

**NS3-Gazebo Bridge Plugin** (`ns3_gazebo_plugin/ns3_gazebo_world.cpp`):
- **Configure()**: Initialize NS-3 network, create WiFi nodes, spawn NS-3 thread
- **PreUpdate()**: Find Gazebo "vehicle" model, bind to NS-3 Node 1
- **Update()**: Read Gazebo pose → update NS-3 ConstantPositionMobilityModel → print metrics

**NS-3 Network Layer** (runs in separate thread):
- WiFi 802.11a @ 54 Mbps (OFDM)
- Ad-hoc mode (IBSS)
- YansWifiChannel propagation model
- TAP bridge to Linux network namespaces
- UDP beacon traffic (Node 0 → Node 1, 1 packet/sec)
- MonitorSnifferRx callback captures RSSI/SNR

### Node Configuration
- **Node 0**: Fixed base station at origin (0, 0, 0)
- **Node 1**: Mobile robot (position synced from Gazebo "vehicle" model)
- **Node 2+**: Additional fixed or mobile nodes (requires code modification)

### Data Flow
```
Gazebo Entity Pose (x,y,z)
       ↓
Update() callback (~100 Hz)
       ↓
NS-3 MobilityModel::SetPosition()
       ↓
NS-3 WiFi PHY propagation calculation
       ↓
MonitorSignalCallback() captures RSSI/SNR
       ↓
Console output every 100 updates
```

## Configuration

### Adding More WiFi Nodes

Edit `ns3_gazebo_plugin/ns3_gazebo_world.cpp`:

```cpp
// Line 44: Increase node count
static const int COUNT = 4;  // 1 base + 1 robot + 2 relay stations

// Lines 94-96: Uncomment and add fixed positions
positionAlloc->Add(ns3::Vector(10.0, 0.0, 0.0));  // Node 2: 10m east
positionAlloc->Add(ns3::Vector(20.0, 0.0, 0.0));  // Node 3: 20m east
```

Then rebuild: `cd build && make`

### Changing WiFi Parameters

Edit `ns3_gazebo_plugin/ns3_gazebo_world.cpp`:

```cpp
// Line 88: Change WiFi standard
wifi.SetStandard(ns3::WIFI_STANDARD_80211n);  // Use 802.11n instead of 802.11a

// Line 90: Change data rate
wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                             "DataMode", ns3::StringValue("OfdmRate6Mbps"));

// Lines 107-108: Adjust transmission power
wifiPhy.Set("TxPowerStart", ns3::DoubleValue(20.0));  // 20 dBm
wifiPhy.Set("TxPowerEnd", ns3::DoubleValue(20.0));
```

### Modifying Update Frequency

Edit `ns3_gazebo_plugin/ns3_gazebo_world.cpp`:

```cpp
// Line 307: Change print frequency (default: every 100 updates)
if (++update_count % 50 == 0) {  // Print twice as often

// Line 183: Change UDP beacon rate (default: 1 packet/second)
echoClient.SetAttribute("Interval", ns3::TimeValue(ns3::Seconds(0.1)));  // 10 packets/sec
```

## Key Files

### Core Plugin Files
- **ns3_gazebo_plugin/ns3_gazebo_world.cpp**: Main integration plugin (520 lines)
- **ns3_gazebo_plugin/ns3_gazebo.sdf**: Basic world with robot model
- **ns3_gazebo_plugin/CMakeLists.txt**: Build configuration with NS-3 library linking
- **ns3_gazebo_plugin/hello_world.cpp**: Minimal example plugin

### Experiment Scripts
- **experiments/run_distance_experiment.sh**: Automated distance vs RSSI experiment runner
- **experiments/distance_controller.py**: Robot movement controller (moves forward at 0.5 m/s)
- **experiments/analyze_data.py**: Data analysis and visualization (generates reports/plots)
- **scenarios/distance_experiment.sdf**: Experiment-specific Gazebo world

### Network Namespace Tools
- **scripts/nns_setup.py**: Setup/teardown Linux network namespaces for distributed simulation
- **src/nns_ns3_wifi.cc**: Standalone NS-3 WiFi + TAP bridge example (copy to ns-3.45/scratch/)

### Testbed System
- **ns3_testbed/testbed_runner.py**: Multi-robot testbed launcher with network namespace support
- **ns3_testbed/csv_setup/example1.csv**: ROS2 pub/sub configuration (node, topic, QoS)
- **ns3_testbed/ns3_testbed_nodes/testbed_robot.py**: ROS2 node for testbed robots
- **ns3_testbed/ns3_testbed_gui/**: GUI for testbed visualization

### ROS2 Integration
- **ns3_gazebo_ws/src/diff_drive_ns3/**: Differential drive controller ROS2 package

## Important Implementation Details

### NS-3 API Migration (3.29 → 3.45)
The codebase was upgraded from NS-3 3.29 to 3.45. Key changes:
- `WIFI_PHY_STANDARD_*` → `WIFI_STANDARD_*`
- Helper initialization: `YansWifiChannelHelper::Default()` now required
- WiFi MAC queue scheduler patched for TAP bridge compatibility

### Thread Safety
NS-3 runs in a separate thread (line 211-216 of ns3_gazebo_world.cpp) for 1 year of simulation time. The `g_signalQuality` global map is accessed from both NS-3 callback thread and Gazebo update thread without mutex protection. This works because:
- NS-3 thread only writes to the map (MonitorSignalCallback)
- Gazebo thread only reads from the map (Update method)
- No concurrent writes to same node ID

### TAP Bridge vs Internet Stack Conflict
Lines 194-206 setup TAP bridges, but this conflicts with UDP echo client/server which requires Internet stack. For pure simulation, UDP beacons work fine. For real network connectivity via TAP bridge, comment out UDP application code (lines 173-190).

### Model Binding Limitation
PreUpdate() method (line 267) hardcodes search for "vehicle" model name. To track multiple robots, modify to iterate all models or search by model prefix.

### Signal Quality Requires Traffic
RSSI/SNR metrics only update when packets are received. UDP beacon traffic starts at 1 second (line 189), so first ~2 seconds show "No packets received yet".

## Testing Network Namespaces

```bash
# After setting up namespaces with nns_setup.py

# Terminal 1: Run talker in nns1
sudo ip netns exec nns1 bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2: Run listener in nns2
sudo ip netns exec nns2 bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp listener

# Check connectivity
sudo ip netns exec nns1 ping 10.0.0.2  # Ping from nns1 to nns2
```

## Dependencies

- **NS-3 3.45**: Network simulator (included in ns-allinone-3.45/)
- **Gazebo Harmonic**: gz-sim8, gz-math7, gz-plugin2, sdformat14
- **ROS2 Jazzy**: For ROS2 workspace and testbed (optional)
- **Linux Network Namespaces**: For distributed testbed (optional)
- **Python 3.12+**: For scripts
- **CMake 3.22+**: For plugin build
- **GCC 13+**: C++20 support required

## Troubleshooting

### Plugin Not Loading
```bash
# Verify plugin path is set correctly
echo $GZ_SIM_SYSTEM_PLUGIN_PATH

# Check plugin exists
ls -la $GZ_SIM_SYSTEM_PLUGIN_PATH/libns3_gazebo_world.so

# Run with verbose logging
gz sim -v 4 ns3_gazebo.sdf
```

### NS-3 Build Failures
```bash
# Rebuild NS-3 from scratch
cd ns-allinone-3.45/ns-3.45
./ns3 clean
./ns3 configure --enable-examples --enable-tests
./ns3 build

# Check for missing dependencies
./ns3 configure --enable-examples --enable-tests 2>&1 | grep "not found"
```

### Network Namespace Issues
```bash
# Check if namespaces exist
ip netns list

# Verify TAP devices
sudo ip netns exec nns1 ip addr show

# Cleanup stuck namespaces
sudo ip -all netns delete
cd scripts && sudo python3 nns_setup.py setup -c 2
```

### No RSSI Data Showing
- Wait 2-3 seconds after simulation starts (UDP beacons start at t=1s)
- Check console for "UDP echo client" messages indicating traffic
- Verify Node 0 and Node 1 are configured (COUNT >= 2)
- Move robot to ensure position changes are detected
