# Hello World Network Example

## Overview

This example demonstrates the basic integration between NS-3 network simulation and Gazebo physics simulation. It creates a simple two-node WiFi network where one node sends "Hello World" messages to another node.

## Learning Objectives

- Understand NS-3 and Gazebo integration basics
- Learn how to create a simple WiFi network
- See packet transmission and reception in action
- Observe network simulation in 3D environment

## Scenario Description

- **Nodes**: 2 WiFi-enabled nodes in Gazebo
- **Network**: IEEE 802.11g WiFi network
- **Traffic**: UDP packets with "Hello World" messages
- **Duration**: 10 seconds simulation
- **Visualization**: Real-time packet flow in Gazebo

## Files Structure

```
01_hello_world_network/
├── README.md              # This file
├── hello_world.cc         # NS-3 simulation script
├── hello_world.world      # Gazebo world file
├── CMakeLists.txt         # Build configuration
├── build.sh              # Build script
├── run.sh                # Run script
├── clean.sh              # Cleanup script
└── config/
    ├── wifi_config.yaml   # WiFi configuration
    └── node_positions.txt # Node position data
```

## Prerequisites

- NS3-Gazebo system installed and working
- Basic understanding of NS-3 concepts
- Familiarity with Gazebo simulation

## Quick Start

1. **Build the example**:
   ```bash
   cd examples/01_hello_world_network
   ./build.sh
   ```

2. **Run the simulation**:
   ```bash
   ./run.sh
   ```

3. **Expected output**:
   - Gazebo window opens with two nodes
   - Console shows packet transmission logs
   - Simulation runs for 10 seconds
   - Final statistics displayed

## Detailed Steps

### Step 1: Build Process

The build script compiles the NS-3 simulation code:

```bash
#!/bin/bash
# Build the hello world network example
cd "$(dirname "$0")"
mkdir -p build
cd build
cmake ..
make
```

### Step 2: Simulation Execution

The run script starts both Gazebo and NS-3:

```bash
#!/bin/bash
# Run the hello world network simulation
cd "$(dirname "$0")"

# Start Gazebo with the world file
gz sim hello_world.world &
GAZEBO_PID=$!

# Wait for Gazebo to initialize
sleep 3

# Run NS-3 simulation
cd build
./hello_world_network

# Clean up
kill $GAZEBO_PID
```

### Step 3: Understanding the Output

**Console Output**:
```
NS-3 Hello World Network Example
================================
Creating 2 WiFi nodes...
Setting up WiFi network...
Installing applications...
Starting simulation...

Time: 1.0s - Node 0 sending: Hello World packet 1
Time: 1.0s - Node 1 received: Hello World packet 1 (32 bytes)
Time: 2.0s - Node 0 sending: Hello World packet 2
Time: 2.0s - Node 1 received: Hello World packet 2 (32 bytes)
...
Time: 10.0s - Simulation complete

Statistics:
- Packets sent: 10
- Packets received: 10
- Packet loss: 0%
- Average delay: 1.2ms
```

**Gazebo Visualization**:
- Two sphere nodes in 3D space
- WiFi signal visualization (if enabled)
- Node movement animations (if configured)

## Code Explanation

### NS-3 Simulation (hello_world.cc)

```cpp
// Key components of the simulation:

// 1. Create WiFi nodes
NodeContainer nodes;
nodes.Create(2);

// 2. Setup WiFi network
WifiHelper wifi;
YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
phy.SetChannel(channel.Create());

// 3. Install network stack
WifiMacHelper mac;
NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

// 4. Setup applications
UdpEchoServerHelper server(9);
UdpEchoClientHelper client(serverAddress, 9);
client.SetAttribute("MaxPackets", UintegerValue(10));
client.SetAttribute("Interval", TimeValue(Seconds(1.0)));
client.SetAttribute("PacketSize", UintegerValue(1024));
```

### Gazebo World (hello_world.world)

```xml
<!-- Simple world with two nodes -->
<sdf version="1.8">
  <world name="hello_world">
    <!-- Physics settings -->
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Node models -->
    <model name="node_0">
      <pose>0 0 0.5 0 0 0</pose>
      <!-- Sphere visual -->
      <link name="base_link">
        <visual name="visual">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </visual>
      </link>
    </model>

    <model name="node_1">
      <pose>5 0 0.5 0 0 0</pose>
      <!-- Sphere visual -->
      <link name="base_link">
        <visual name="visual">
          <geometry>
            <sphere><radius>0.1</radius></sphere>
          </geometry>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

## Configuration Options

### WiFi Settings (config/wifi_config.yaml)

```yaml
wifi:
  standard: "802.11g"
  data_rate: "54Mbps"
  tx_power: 20.0  # dBm
  frequency: 2.4  # GHz
  channel_width: 20  # MHz
```

### Node Positions (config/node_positions.txt)

```
# node_id x y z
0 0.0 0.0 0.5
1 5.0 0.0 0.5
```

## Customization

### Modifying Network Parameters

1. **Change WiFi standard**:
   ```cpp
   wifi.SetStandard(WIFI_STANDARD_80211n);
   ```

2. **Adjust transmission power**:
   ```cpp
   phy.Set("TxPowerStart", DoubleValue(20.0));
   phy.Set("TxPowerEnd", DoubleValue(20.0));
   ```

3. **Modify packet size and rate**:
   ```cpp
   client.SetAttribute("PacketSize", UintegerValue(2048));
   client.SetAttribute("Interval", TimeValue(Seconds(0.5)));
   ```

### Adding More Nodes

```cpp
// Increase node count
nodes.Create(4);  // Creates 4 nodes instead of 2

// Setup mesh topology
for (uint32_t i = 0; i < nodes.GetN(); ++i) {
    // Configure each node...
}
```

### Enabling Packet Capture

```cpp
// Enable pcap tracing
phy.EnablePcapAll("hello_world");

// Enable ASCII tracing
AsciiTraceHelper ascii;
phy.EnableAsciiAll(ascii.CreateFileStream("hello_world.tr"));
```

## Troubleshooting

### Common Issues

1. **Build fails**:
   - Check NS-3 installation
   - Verify CMAKE_PREFIX_PATH includes NS-3
   - Ensure C++20 compiler support

2. **Gazebo doesn't start**:
   - Check GAZEBO_MODEL_PATH
   - Verify Gazebo Harmonic installation
   - Try running Gazebo manually: `gz sim hello_world.world`

3. **No network communication**:
   - Verify NS-3 WiFi module compilation
   - Check node positions (not too far apart)
   - Review WiFi configuration parameters

4. **Simulation runs too fast/slow**:
   - Adjust real-time factor in world file
   - Modify NS-3 simulation time step

### Debug Commands

```bash
# Check NS-3 modules
./ns3 show modules

# Verify Gazebo plugins
gz plugin --info

# Test WiFi functionality
./build/hello_world_network --verbose

# Check network interfaces
ip link show
```

## Performance Notes

- **Simulation time**: ~10 seconds real-time
- **Memory usage**: ~50MB
- **CPU usage**: Low (single core)
- **Network overhead**: Minimal

## Next Steps

After completing this example:

1. Try modifying node positions
2. Experiment with different WiFi standards
3. Add packet loss or delay models
4. Progress to the robot navigation example

## Related Examples

- **02_simple_robot_navigation**: Adds mobility to nodes
- **03_wifi_performance_test**: Performance analysis tools
- **04_tap_bridge_connection**: Real network integration

## Support

For help with this example:
- Check the main project documentation
- Review NS-3 tutorial documentation
- Ask questions in GitHub Discussions