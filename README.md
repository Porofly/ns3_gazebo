# NS-3 Network Simulator + Gazebo Harmonic + ROS2 Integration

A powerful integrated simulation platform for testing multi-robot systems with realistic WiFi network simulation. This project combines NS-3's discrete event network simulator with Gazebo Harmonic's physics engine and ROS2 middleware for end-to-end robotics communication testing.

## Overview

This project enables realistic simulation of multi-robot scenarios where robots communicate over WiFi networks with accurate network performance modeling. By integrating NS-3's WiFi simulation with Gazebo's robot physics and ROS2's middleware, you can test communication-aware robotic algorithms in a unified environment before physical deployment.

## Key Features

- **IEEE 802.11a WiFi Ad-hoc Network Simulation** - Realistic 54Mbps WiFi with PHY/MAC layer modeling
- **Real-time Robot Physics** - Full 3D rigid body dynamics via Gazebo Harmonic
- **ROS2 Jazzy Integration** - Complete robot control and sensor data publishing
- **Multi-Robot Support** - Dynamic spawning and removal of network-connected robots
- **Network Performance Metrics** - Real-time RSSI, SNR, and packet loss monitoring
- **Configurable Test Scenarios** - CSV-based scenario configuration
- **GUI Network Monitoring** - PyQt5-based visualization for network metrics
- **Network Namespace Isolation** - Linux network namespaces for isolated robot networks
- **FastDDS Multi-Network** - Cross-namespace DDS communication support
- **Simulation Time Sync** - Deterministic, reproducible experiments with synchronized timing

## Architecture

The project uses a modular, layered architecture:

```
┌─────────────────────────────────────────────────────────┐
│               GAZEBO HARMONIC                           │
│    (3D Physics, Robot Models, Visualization)            │
└──────────┬──────────────────────────────────────────────┘
           │ Robot Position Updates
           ▼
┌─────────────────────────────────────────────────────────┐
│      NS3 GAZEBO PLUGIN (Gazebo System Plugin)           │
│  - Tracks robot entities                                │
│  - Updates NS3 node positions                           │
│  - Monitors network statistics (RSSI/SNR/packets)       │
└──────────┬──────────────────────────────────────────────┘
           │
    ┌──────┴──────────────────────┐
    │                             │
    ▼                             ▼
┌──────────────┐      ┌──────────────────────┐
│  NS-3 THREAD │      │  ROS2 NODE THREAD    │
│              │      │                      │
│ - WiFi Sim   │◄────►│ - Odometry Sub       │
│ - TAP Bridge │      │ - Position Update    │
│ - PHY/MAC    │      │ - Robot Control      │
│ - Packet Log │      │                      │
└────────┬─────┘      └──────────────────────┘
         │
         ▼
┌──────────────────────────────────────────┐
│  LINUX NETWORK NAMESPACES (nns1, nns2)   │
│  - Virtual network interfaces            │
│  - IP stacks (192.168.x.x)               │
│  - TAP device connections                │
└──────────────────────────────────────────┘
```

## Technologies Used

| Technology | Version | Purpose |
|-----------|---------|---------|
| **NS-3** | 3.45 | Discrete event network simulator |
| **Gazebo** | Harmonic | Robot physics & 3D visualization |
| **ROS 2** | Jazzy | Robot middleware & communication |
| **C++** | 20 | Core plugin implementation |
| **Python** | 3.x | Testing, GUI, and utilities |
| **CMake** | 3.22.1+ | Build system |
| **FastDDS** | Latest | Inter-process communication middleware |
| **PyQt5** | 5.x | GUI framework |

## Project Structure

```
ns3_gazebo/
├── ns3_gazebo_plugin/          # Core Gazebo-NS3 integration plugin
│   ├── ns3_gazebo_world.cpp    # Main plugin implementation
│   ├── ns3_gazebo_ros2.sdf     # Gazebo world description
│   └── fastdds_*.xml           # DDS middleware configuration
│
├── etc/
│   ├── ns3_gazebo_ws/          # ROS2 workspace with robot control packages
│   │   └── src/diff_drive_ns3/ # Differential drive robot with NS3 integration
│   ├── ns3_testbed/            # Testing framework with GUI
│   │   ├── testbed_runner.py   # Multi-robot orchestration
│   │   ├── ns3_testbed_gui/    # PyQt5 network metrics visualization
│   │   └── ns3_testbed_nodes/  # ROS2 test nodes
│   ├── ns3_testbed_simtime/    # Simulation time synchronized version
│   └── ns3_wifi_tap_test/      # WiFi TAP device testing utilities
│
├── ns-allinone-3.45/           # NS-3 3.45 network simulator (submodule)
└── scripts/                    # Utility scripts
    └── nns_setup.py            # Network namespace setup
```

## Key Components

### NS3 Gazebo Plugin (`ns3_gazebo_plugin/`)

Core integration between Gazebo and NS-3 that:
- Creates NS-3 nodes representing robots in the simulated WiFi network
- Configures WiFi 802.11a ad-hoc network with 54Mbps data rate
- Implements PHY-layer monitoring callbacks to capture RSSI, SNR, and packet statistics
- Uses TAP bridges to connect NS-3 virtual WiFi devices to Linux network namespaces
- Synchronizes robot positions between Gazebo and NS-3 mobility models
- Runs NS-3 simulator in a separate thread for real-time execution

**Main file:** `ns3_gazebo_world.cpp`

### ROS2 Differential Drive Robot (`etc/ns3_gazebo_ws/src/diff_drive_ns3/`)

Bridges ROS2 odometry data to NS3 network simulation:
- Subscribes to odometry messages from Gazebo
- Updates NS-3 node position based on robot's actual Gazebo position
- Runs both NS-3 simulator and ROS2 node in separate threads
- Synchronizes robot movement with network simulation

**Main files:** `diff_drive_ns3_ros2.cpp`, `diff_drive_robot.cpp/.hpp`

### NS3 Testbed Framework (`etc/ns3_testbed/`)

Comprehensive multi-robot testing framework with:
- **Testbed Runner** - Orchestrates multi-robot simulations using network namespaces
- **Testbed GUI** - PyQt5-based visualization for network metrics
- **Testbed Nodes** - Python ROS2 publishers/subscribers framework
- **Network Setup** - Creates Linux network namespaces and virtual network interfaces

### FastDDS Multi-Network Configuration

Enables DDS communication across separate network namespaces:
- Static peer discovery configuration for ROS2 DDS middleware
- Supports dual-network topology (direct connections + WiFi simulation)
- Configures separate participant profiles for host and network namespace nodes

## Usage Scenarios

### Scenario 1: Single Robot Network Testing

```bash
# Launch Gazebo with NS3 plugin
gz sim -f ns3_gazebo_ros2.sdf

# Run ROS2 robot controller
ros2 run diff_drive_ns3 diff_drive_ns3_ros2

# Observe network metrics in console output
```

### Scenario 2: Multi-Robot Testbed

```bash
# Setup network namespaces
sudo python3 scripts/nns_setup.py --setup example_wifi

# Run testbed orchestration
python3 etc/ns3_testbed/testbed_runner.py

# Launch GUI for visualization
python3 etc/ns3_testbed/ns3_testbed_gui/tg.py
```

### Scenario 3: Simulation Time Synchronized Testing

```bash
# Use simtime version for deterministic results
cd etc/ns3_testbed_simtime
python3 testbed_runner.py --config scenario.csv
```

## Build Instructions

### Prerequisites

- CMake 3.22.1+
- C++20 compiler (g++ or clang)
- Python 3.x
- Gazebo Harmonic libraries (gz-sim8, gz-math7, gz-plugin2)
- ROS2 Jazzy
- PyQt5
- FastDDS

### Build Process

```bash
# 1. Build NS-3
cd ns-allinone-3.45/ns-3.45
./ns3 configure --build-profile=optimized
./ns3 build

# 2. Build Gazebo plugin
cd ns3_gazebo_plugin
cmake ..
make

# 3. Build ROS2 packages
cd etc/ns3_gazebo_ws
colcon build
```

## Network Simulation Details

**WiFi Configuration:**
- Standard: IEEE 802.11a (OFDM)
- Data Rate: 54 Mbps
- Mode: Ad-hoc (no access point required)
- Propagation Model: YansWifiChannelHelper (default path loss)
- Layers Simulated: PHY + MAC

**Monitored Metrics:**
- RSSI (Received Signal Strength Indicator)
- SNR (Signal-to-Noise Ratio)
- Packet transmission/reception counts
- Network throughput
- Packet loss rates

## Target Use Cases

- Testing multi-robot communication systems with realistic WiFi network conditions
- Analyzing network performance during robot coordination tasks
- Developing algorithms for communication-aware robot navigation
- Prototyping networked robot swarms before physical deployment
- Research in mobile ad-hoc networks (MANETs)
- Network-dependent robot behavior testing

## Recent Development

Recent work has focused on:
- Multi-robot support with follower robot implementation and communication handling
- Statistics logging with CSV output for network performance metrics
- FastDDS integration for dual-network configuration and namespace isolation
- WiFi relay bridge for communication relay between isolated networks
- Code cleanup and improved organization
- Migration from NS-3 3.29 to 3.45 with API updates
- Migration from Gazebo Classic to Harmonic with new plugin API

## License

See individual component licenses:
- NS-3: GPLv2
- Gazebo: Apache 2.0
- ROS2: Apache 2.0

## Contributing

This project integrates multiple open-source robotics and networking tools. Contributions are welcome for improvements to the integration layer, additional robot models, or testing scenarios.
