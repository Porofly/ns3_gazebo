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



## Quick Start



For those who want to get running quickly:



```bash

# 1. Build NS-3

cd ns-allinone-3.45/ns-3.45

./ns3 configure --enable-examples --enable-tests

./ns3 build



# 2. Build plugin

cd ../../ns3_gazebo_plugin

mkdir -p build && cd build

cmake .. && make



# 3. Set environment (adjust path as needed)

export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd):$GZ_SIM_SYSTEM_PLUGIN_PATH



# 4. Setup network namespaces

cd ../scripts

sudo ./nns_setup.py setup --count 5



# 5. Run simulation

cd ../ns3_gazebo_plugin

gz sim ns3_gazebo_ros2.sdf

```



Then spawn robots and watch the network metrics in the terminal!



## Prerequisites



- **Ubuntu 24.04** (recommended)

- **ROS2 Jazzy** - Robot Operating System 2

- **Gazebo Harmonic 8** - Modern physics simulator

- **NS-3 3.45** - Network simulator (included in this repo)

- **Build Tools**: cmake, g++, python3

- **Root privileges** - Required for network namespace setup



## Installation & Setup



### 1. Build NS-3



First, build the NS-3 network simulator:



```bash

cd ns-allinone-3.45/ns-3.45

./ns3 configure --enable-examples --enable-tests

./ns3 build

```



### 2. Build the Gazebo Plugin



Build the ns3_gazebo plugin that bridges Gazebo and NS-3:



```bash

cd ../../ns3_gazebo_plugin

mkdir -p build && cd build

cmake ..

make

```



The plugin will be compiled as `libns3_gazebo_world.so`.



### 3. Set Environment Variables



Add the plugin to Gazebo's search path:



```bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=/path/to/ns3_gazebo/ns3_gazebo_plugin/build:$GZ_SIM_SYSTEM_PLUGIN_PATH

```



For convenience, add this to your `~/.bashrc`:



```bash

echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=/path/to/ns3_gazebo/ns3_gazebo_plugin/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc

source ~/.bashrc

```



### 4. Setup Network Namespaces



Create network namespaces and TAP devices for multi-robot networking. This requires root privileges:



```bash

cd ../scripts

sudo ./nns_setup.py setup --count 5

```



This creates 5 network namespaces (nns1 through nns5) with:

- TAP devices: `wifi_tap1`, `wifi_tap2`, ..., `wifi_tap5`

- Virtual ethernet pairs for isolation

- Bridges connecting TAP devices to namespaces



**Note**: You need to run this after each system reboot.



To teardown the network configuration:



```bash

sudo ./nns_setup.py teardown --count 5

```



## Running the Simulation



### Basic Usage



1. **Start Gazebo with the NS-3 plugin:**



```bash

cd ns3_gazebo_plugin

gz sim ns3_gazebo_ros2.sdf

```



This loads an empty world with:

- NS-3 WiFi simulator running in background

- Ground plane and lighting

- Ready to accept dynamically spawned robots



2. **Spawn robots dynamically:**



You can spawn robots using Gazebo's service interface. Each robot model name should start with "robot" (e.g., robot1, robot2):



```bash

# Example: Spawn a simple box robot

gz service -s /world/ns3_gazebo_world/create \

  --reqtype gz.msgs.EntityFactory \

  --reptype gz.msgs.Boolean \

  --timeout 1000 \

  --req 'sdf: "<?xml version=\"1.0\"?><sdf version=\"1.8\"><model name=\"robot1\"><pose>2 0 0.5 0 0 0</pose><link name=\"link\"><collision name=\"collision\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision><visual name=\"visual\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></visual></link></model></sdf>"'

```



3. **Monitor network metrics:**



The plugin prints network status every 100 simulation frames (~1 second):

- Robot positions in Gazebo

- NS-3 node positions

- RSSI (Received Signal Strength Indicator) in dBm

- SNR (Signal-to-Noise Ratio) in dB

- Packet reception statistics

- Inter-robot distances



### Running Applications in Network Namespaces



To test real network communication through the simulated WiFi:



```bash

# In namespace nns1 (robot1)

sudo ip netns exec nns1 iperf3 -s



# In namespace nns2 (robot2)

sudo ip netns exec nns2 iperf3 -c 10.0.0.1

```



The network performance will reflect the simulated WiFi conditions based on robot positions!



## Configuration



### Adding More Robots



By default, the plugin creates 2 NS-3 nodes (1 base station + 1 robot). To add more:



1. Edit `ns3_gazebo_plugin/ns3_gazebo_world.cpp`:

   - Change `COUNT` variable (line 47) to desired number

   - Add initial positions in the position allocator (around line 157)



2. Rebuild the plugin:

   ```bash

   cd ns3_gazebo_plugin/build

   make

   ```



3. Setup corresponding network namespaces:

   ```bash

   sudo ./scripts/nns_setup.py setup --count <COUNT>

   ```



### WiFi Configuration



The default configuration uses:

- **Standard**: 802.11a (5 GHz)

- **Data Rate**: 54 Mbps (OFDM)

- **MAC**: Ad-hoc mode

- **Propagation**: Friis model (default)



To modify WiFi settings, edit `ns3_gazebo_world.cpp` in the `ns3_setup()` function (lines 89-214).



### Network Namespace Configuration



The `nns_setup.py` script supports additional options:



```bash

# Setup with direct network connection (for external internet access)

sudo ./scripts/nns_setup.py setup --count 5 --include_direct



# Custom number of namespaces

sudo ./scripts/nns_setup.py setup --count 10

```



## Troubleshooting



### Plugin not found

**Error**: `Failed to load plugin: libns3_gazebo_world.so`



**Solution**: Ensure `GZ_SIM_SYSTEM_PLUGIN_PATH` is set correctly:

```bash

export GZ_SIM_SYSTEM_PLUGIN_PATH=/path/to/ns3_gazebo/ns3_gazebo_plugin/build:$GZ_SIM_SYSTEM_PLUGIN_PATH

```



### TAP device not found

**Error**: `TapBridge: Failed to open tap device`



**Solution**: Run the network namespace setup script:

```bash

sudo ./scripts/nns_setup.py setup --count 5

```



### NS-3 build errors

**Error**: NS-3 libraries not found during plugin build



**Solution**:

1. Ensure NS-3 is built: `cd ns-allinone-3.45/ns-3.45 && ./ns3 build`

2. Check that `build/lib/` contains `libns3.45-*.so` files



### No network metrics shown

**Issue**: Plugin loads but no RSSI/SNR data appears



**Solution**: Ensure robots are spawned with names starting with "robot" (robot1, robot2, etc.). The plugin only tracks models matching this pattern.



## Verifying Installation



Check that everything is set up correctly:



```bash

# 1. Verify NS-3 libraries

ls ns-allinone-3.45/ns-3.45/build/lib/libns3*.so



# 2. Verify plugin built

ls ns3_gazebo_plugin/build/libns3_gazebo_world.so



# 3. Verify network namespaces

sudo ip netns list  # Should show nns1, nns2, etc.



# 4. Verify TAP devices

ip link show | grep wifi_tap  # Should show wifi_tap1, wifi_tap2, etc.

```



## Example Use Cases



### Multi-Robot Coordination Study



Simulate a team of robots moving in formation and measure how network quality affects coordination:



1. Spawn 4 robots at different positions

2. Run a coordination algorithm in each namespace

3. Monitor RSSI/SNR as robots move apart

4. Measure packet loss and communication latency

5. Analyze how distance affects task completion



### WiFi Coverage Mapping



Create a heatmap of WiFi signal strength in your environment:



1. Spawn a base station (robot1) at a fixed position

2. Spawn a mobile robot (robot2)

3. Write a script to move robot2 through the space

4. Log RSSI values at each position

5. Visualize coverage map



### Network-Aware Path Planning



Test path planning algorithms that consider network connectivity:



1. Implement a path planner that avoids low-signal areas

2. Compare with traditional path planning

3. Measure task success rate vs network reliability

4. Optimize robot routes for both distance and connectivity



## Advanced Topics



### Custom WiFi Models



You can modify the WiFi propagation model in ns3_gazebo_world.cpp:108:



```cpp

// Default uses Friis propagation model

// Change to log-distance path loss model:

wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",

                                "Exponent", ns3::DoubleValue(3.0));

```



### ROS2 Integration



The plugin supports ROS2 Jazzy. FastDDS configuration files are included for multi-namespace ROS2 communication:

- `fastdds_host.xml` - Host machine configuration

- `fastdds_nns1.xml` - Robot 1 namespace configuration

- `fastdds_nns2.xml` - Robot 2 namespace configuration



Use these to enable ROS2 discovery across network namespaces.



### Data Logging



To log WiFi metrics to file, you can enable PCAP tracing in ns3_gazebo_world.cpp:138:



```cpp

// Uncomment to capture all WiFi packets

wifiPhy.EnablePcapAll("ns3_gazebo_wifi");

```



This creates .pcap files that can be analyzed with Wireshark.



## References



- [NS-3 Documentation](https://www.nsnam.org/documentation/)

- [Gazebo Harmonic Documentation](https://gazebosim.org/docs/harmonic/getstarted)

- [TAP Bridge Guide](https://www.nsnam.org/docs/release/3.45/models/html/tap.html)

- [Linux Network Namespaces](https://man7.org/linux/man-pages/man8/ip-netns.8.html)



## License



See [COPYING.md](COPYING.md) for license information.

