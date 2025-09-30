# NS3-Gazebo Integration System

**Real-time Network Simulation for Robotic Systems**

[![NS-3](https://img.shields.io/badge/NS--3-3.45-blue)](https://www.nsnam.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic%208-orange)](https://gazebosim.org/)
[![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E)](https://docs.ros.org/en/jazzy/)

---

## Overview

NS3-Gazebo is an integrated simulation platform that combines:
- **NS-3 3.45**: Discrete-event network simulator with WiFi 802.11a/b/g/n/ac support
- **Gazebo Harmonic 8**: High-fidelity physics simulation for robotics
- **ROS2 Jazzy**: Robot Operating System for control and communication

This system enables **realistic network simulation** for multi-robot systems by:
1. Simulating robot physics and mobility in Gazebo
2. Modeling WiFi propagation and network behavior in NS-3
3. Synchronizing robot positions between simulators in real-time
4. Measuring actual network performance (RSSI, SNR, throughput, packet loss)

### Key Features

- **Real-time Position Synchronization**: Gazebo robot positions automatically update NS-3 mobility models
- **Actual WiFi Metrics**: Live RSSI, SNR, and data rate measurements from NS-3 PHY layer
- **TAP Bridge Support**: Real network connectivity via Linux network namespaces
- **Distributed Testbed**: Multi-robot simulation across network namespaces
- **Modular Architecture**: Easy to extend with additional robots or network nodes

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    Gazebo Harmonic 8                            │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  Physics Simulation                                      │   │
│  │  - Robot dynamics (differential drive)                   │   │
│  │  - Collision detection                                   │   │
│  │  - Sensor simulation                                     │   │
│  └────────────────┬─────────────────────────────────────────┘   │
└───────────────────┼─────────────────────────────────────────────┘
                    │ Robot Position (x, y, z)
                    ▼
┌─────────────────────────────────────────────────────────────────┐
│            NS3-Gazebo Plugin (System Plugin)                    │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  • Reads Gazebo entity poses every frame                 │   │
│  │  • Updates NS-3 mobility models                          │   │
│  │  • Monitors WiFi PHY layer (MonitorSnifferRx)            │   │
│  │  • Outputs RSSI, SNR, expected data rate                 │   │
│  └────────────────┬─────────────────────────────────────────┘   │
└───────────────────┼─────────────────────────────────────────────┘
                    │ Position Update
                    ▼
┌─────────────────────────────────────────────────────────────────┐
│                      NS-3 3.45                                  │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │  WiFi Network Simulation                                 │   │
│  │  • 802.11a Ad-hoc network                                │   │
│  │  • Path loss calculation (log-distance model)            │   │
│  │  • Signal propagation (YansWifiChannel)                  │   │
│  │  • TAP bridge to real network                            │   │
│  │  • UDP beacon traffic (Node 0 → Node 1)                  │   │
│  └──────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### Node Configuration

- **Node 0**: Fixed base station/AP at world origin (0, 0, 0)
- **Node 1**: Mobile robot (position synchronized from Gazebo)
- **Nodes 2+**: Additional fixed nodes or robots (configurable)

---

## Directory Structure

```
ns3_gazebo/
│
├── README.md                     # This file
│
├── ns-allinone-3.45/             # NS-3 network simulator
│   └── ns-3.45/                  # NS-3 3.45 source code
│       └── src/                  # NS-3 modules (wifi, mobility, etc.)
│
├── ns3_gazebo_plugin/            # Main Gazebo-NS3 integration
│   ├── ns3_gazebo_world.cpp      # System plugin (Gazebo-NS3 bridge)
│   ├── ns3_gazebo.sdf            # Gazebo world with robot model
│   ├── CMakeLists.txt            # Build configuration
│   ├── build/                    # Compiled plugin (.so files)
│   │   ├── libns3_gazebo_world.so
│   │   └── libhello_world.so
│   └── hello_world.cpp           # Example hello world plugin
│
├── scripts/                      # Utility scripts
│   └── nns_setup.py              # Network namespace setup
│
├── ns3_wifi_tap_test/            # WiFi TAP bridge testing (NS-3 Only)
│
├── ns3_gazebo_ws/                # ROS2 workspace              (Optional, Not Verified)
├── ns3_testbed/                  # Distributed testbed         (Optional, Not Verified)
├── ns3_testbed_simtime/          # Simulation time management  (Optional, Not Verified)
├── example_policy/               # Example policies            (Optional, Not Verified)
└── src/                          # Additional source files     (Copied to ns-3 scratch directory)
```

---

## Prerequisites

### System Requirements

- **OS**: Ubuntu 24.04 LTS (recommended) or Ubuntu 22.04

### Software Dependencies

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install Gazebo Harmonic
sudo apt install -y gz-harmonic

# Install ROS2 Jazzy (if not already installed)
sudo apt install -y ros-jazzy-desktop

# Install NS-3 dependencies
sudo apt install -y g++ python3 cmake ninja-build ccache

# Install network tools
sudo apt install -y iproute2 net-tools iputils-ping bridge-utils

# Install development tools
sudo apt install -y git pkg-config
```

### Verify Installation

```bash
# Check Gazebo version
gz sim --version
# Expected: Gazebo Sim, version 8.x.x

# Check ROS2 version
ros2 --version
# Expected: ros2 doctor version 0.x.x using Jazzy

# Check NS-3 (after building)
./ns-allinone-3.45/ns-3.45/ns3 --version
# Expected: ns-3.45
```

---

## Installation & Build

### 1. Clone Repository

```bash
git clone <repository-url>
```

### 2. Build NS-3

```bash
cd ns3_gazebo/ns-allinone-3.45/ns-3.45

# Configure NS-3
./ns3 configure --enable-examples --enable-tests

# Build NS-3
./ns3 build

# Verify build
./ns3 run hello-simulator
# Should output: "Hello Simulator"
```

### 3. Build NS3-Gazebo Plugin

```bash
cd ns3_gazebo/ns3_gazebo_plugin

# Create build directory
mkdir -p build && cd build

# Configure and build
cmake ..
make

# Verify build
ls -la *.so
# Expected: libns3_gazebo_world.so, libhello_world.so
```

### 4. Build ROS2 Workspace (Optional, Not Verified)

```bash
cd ns3_gazebo/ns3_gazebo_ws

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```

---

## Quick Start

### Network Namespace Setup (Distributed Simulation)

```bash
cd ns3_gazebo/scripts

# Setup 2 network namespaces (number of nodes)
sudo ./nns_setup.py setup -c 2

# Verify namespaces
ip netns list
# Expected: nns1, nns2

# Cleanup namespaces
sudo ./nns_setup.py teardown -c 2
```

### Running Basic Simulation

```bash
cd ~/realgazebo/ns3_gazebo/ns3_gazebo_plugin

# Set plugin path
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd)/build"

# Launch simulation
gz sim ns3_gazebo.sdf
```

**What happens:**
1. Gazebo window opens with a blue robot vehicle
2. NS-3 network simulator starts in background thread
3. Console shows:
   - Node configuration (Node 0: Base Station, Node 1: Robot)
   - Signal monitoring enabled
   - TAP bridge installation
   - UDP beacon traffic status

### Controlling the Robot

[Gazebo Harmonic Reference: Moving Robot](https://gazebosim.org/docs/harmonic/moving_robot/)

Use keyboard arrow keys to move the robot:
- **↑**: Forward
- **↓**: Backward
- **←**: Turn left
- **→**: Turn right

### Monitoring Network Quality

Every ~2 seconds (after UDP packets start flowing), console shows:

```
=== NS-3 Network Status ===
Robot (Node 1) position: (5.20, 1.30, 0.40)

[Robot ↔ Base Station]
  Distance: 5.41 m
  NS-3 WiFi Metrics:
    RSSI: -68.3 dBm [Fair - Reduced Speed]
    SNR: 18.5 dB
    Expected Rate: 48 Mbps
===========================
```

**Signal Quality Levels:**
- **Excellent** (RSSI > -50 dBm): Max speed, very close range
- **Good** (RSSI > -60 dBm): 54 Mbps, normal operation
- **Fair** (RSSI > -70 dBm): Reduced speed, longer range
- **Weak** (RSSI > -80 dBm): Unstable connection
- **Very Weak** (RSSI < -80 dBm): Packet loss, near disconnection

### Testing ROS2 System
```
# Working within the Namespaces

# Open new terminal
sudo ip netns exec nns1 bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker

# Open new terminal
sudo ip netns exec nns2 bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp listener

```

## Configuration

### Adding More Nodes

Edit `ns3_gazebo_plugin/ns3_gazebo_world.cpp`:

```cpp
// Change node count (line 44)
static const int COUNT=4;  // 1 base + 1 robot + 2 fixed nodes

// Add fixed node positions (lines 94-96, uncomment)
positionAlloc->Add(ns3::Vector(10.0, 0.0, 0.0));  // Node 2: 10m
positionAlloc->Add(ns3::Vector(20.0, 0.0, 0.0));  // Node 3: 20m
```

Rebuild:
```bash
cd build && make
```

### Adjusting WiFi Parameters

Edit `ns3_gazebo_plugin/ns3_gazebo_world.cpp` (lines 88-95):

```cpp
// Change WiFi standard
wifi.SetStandard(ns3::WIFI_STANDARD_80211n);  // 802.11n instead of 802.11a

// Change data rate
wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                             "DataMode", ns3::StringValue("OfdmRate6Mbps"));

// Adjust transmission power
wifiPhy.Set("TxPowerStart", ns3::DoubleValue(20.0));  // 20 dBm
wifiPhy.Set("TxPowerEnd", ns3::DoubleValue(20.0));
```

### Changing Beacon Frequency

Edit `ns3_gazebo_plugin/ns3_gazebo_world.cpp` (line 183):

```cpp
// Change from 1 packet/second to 10 packets/second
echoClient.SetAttribute("Interval", ns3::TimeValue(ns3::Seconds(0.1)));
```

### Modifying Update Rate

Edit `ns3_gazebo_plugin/ns3_gazebo_world.cpp` (line 307):

```cpp
// Print every 50 updates instead of 100
if (++update_count % 50 == 0) {
```

## Technical Details

### NS-3 WiFi Configuration

- **Standard**: 802.11a (5 GHz)
- **Channel Model**: YansWifiChannel (default propagation loss)
- **MAC Type**: Ad-hoc (IBSS)
- **Data Rate**: 54 Mbps (OFDM, adjustable based on SNR)
- **TX Power**: ~16 dBm (configurable)
- **Frequency**: 5180 MHz (Channel 36)

### Gazebo Plugin Callbacks

- **Configure**: Initializes NS-3, sets up devices, starts simulator thread
- **PreUpdate**: Finds and matches Gazebo models with NS-3 nodes (first frame only)
- **Update**: Reads robot poses, updates NS-3 mobility, prints metrics (every frame)

### Performance Metrics

**Update Frequency:**
- Gazebo simulation: ~1000 Hz (depends on physics step size)
- NS-3 position sync: Every Gazebo frame
- Metric output: Every 100 frames (~10 Hz with default physics)
- UDP beacons: 1 packet/second

**Computational Cost:**
- NS-3 in separate thread (non-blocking)
- Position update: O(1) per robot
- Signal monitoring: Triggered by packet reception (minimal overhead)

---

## API Reference

### Key Classes

#### `NS3GazeboWorld` (ns3_gazebo_world.cpp)

Main plugin class implementing Gazebo System interface.

**Methods:**
- `Configure()`: Initialize NS-3 network, create nodes, setup callbacks
- `PreUpdate()`: Find and bind Gazebo models to NS-3 nodes
- `Update()`: Synchronize positions, read signal quality, output metrics

#### `MonitorSignalCallback()`

Callback function to capture WiFi PHY layer statistics.

**Parameters:**
- `nodeId`: NS-3 node identifier
- `signalNoise`: Structure containing RSSI and noise level

**Updates:**
- `g_signalQuality[nodeId].rssi`: Received Signal Strength Indicator (dBm)
- `g_signalQuality[nodeId].snr`: Signal-to-Noise Ratio (dB)

### Configuration Constants

```cpp
// ns3_gazebo_world.cpp

static const int COUNT = 2;  // Number of NS-3 nodes

struct SignalQuality {
  double rssi;              // RSSI in dBm
  double snr;               // SNR in dB
  double rxPower;           // Received power in dBm
  uint64_t lastUpdateTime;  // Timestamp of last update
  bool hasData;             // Whether metrics are available
};
```

## Known Limitations

1. **TAP Bridge Compatibility**: TAP bridge and Internet stack both try to assign IP addresses, which may conflict. For pure simulation, UDP beacons work fine. For real network connectivity, disable UDP beacon traffic.

2. **Model Matching**: Currently hardcoded to match model name "vehicle". To track multiple robots, modify `PreUpdate()` to iterate all models.

3. **One-way Synchronization**: Only Gazebo → NS-3 position sync. NS-3 network conditions don't affect Gazebo physics (by design).

4. **Signal Quality Requires Traffic**: RSSI/SNR metrics require packet transmission. Initial ~2 seconds show "No packets received yet".


## References

- [NS-3 3.45 Documentation](https://www.nsnam.org/docs/release/3.45/)
- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic)
- [ROS2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)

---

**Last Updated**: 2024-09-30
