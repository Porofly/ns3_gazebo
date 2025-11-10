# NS3-Gazebo Integration

A real-time network simulation platform for multi-robot systems that combines NS-3 network simulator, Gazebo Harmonic physics engine, and ROS2 middleware.

## Overview

This project enables realistic WiFi network simulation in robotic systems by synchronizing robot positions between Gazebo and NS-3 in real-time. It allows researchers to measure actual network performance (RSSI, SNR, throughput, packet loss) as robots move in a simulated environment.

## Key Features

- **Real-time Position Synchronization**: Robot positions from Gazebo are continuously updated in NS-3
- **Live Network Metrics**: WiFi signal quality (RSSI, SNR, data rate) measured from NS-3 PHY layer
- **Multi-Robot Support**: TAP bridge and network namespaces for distributed simulation
- **Modular Architecture**: Easy to extend and customize for specific use cases

## System Components

### Core Integration Plugin
- **ns3_gazebo_plugin/**: Gazebo system plugin (C++) that bridges Gazebo and NS-3
  - Reads robot poses from Gazebo each simulation frame
  - Updates NS-3 mobility models for WiFi propagation calculation
  - Monitors WiFi PHY layer and outputs network metrics

### Network Simulator
- **ns-allinone-3.45/ns-3.45/**: NS-3 discrete-event network simulator with WiFi 802.11a/b/g/n/ac support

### Utility Scripts
- **scripts/nns_setup.py**: Network namespace configuration for distributed multi-robot testbeds

## Technologies Used

- **NS-3 3.45**: Network simulation and WiFi modeling
- **Gazebo Harmonic 8**: Physics simulation and robot visualization
- **ROS2 Jazzy**: Robot control and inter-process communication

## How It Works

1. Gazebo simulates robot physics and movement
2. NS3-Gazebo plugin extracts robot positions each frame
3. Plugin updates NS-3 mobility models for network propagation
4. WiFi signal quality is calculated based on robot positions
5. Network metrics (RSSI, SNR, packet loss) are output in real-time

## Use Case

Ideal for testing distributed multi-robot algorithms where network reliability affects coordination and task success. Researchers can measure exactly how WiFi signal degradation impacts robot swarm behavior.

## Building

```bash
cd ns3_gazebo_plugin
mkdir -p build && cd build
cmake ..
make
```

## Running

Load the Gazebo world file with the plugin:
```bash
gz sim ns3_gazebo_ros2.sdf
```

## License

See [COPYING.md](COPYING.md) for license information.
