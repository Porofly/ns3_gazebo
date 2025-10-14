# NS3 Gazebo World Plugin

## Overview
This plugin integrates NS-3 WiFi network simulation with Gazebo Harmonic for realistic wireless network modeling in robot simulations. It enables real-time network performance measurements (RSSI, SNR, signal strength) by synchronizing robot positions from Gazebo to NS-3's propagation models.

## Architecture

### Network Configuration
- **Node Count**: 2 nodes (configurable via `COUNT` constant at line 46)
  - Node 0: Fixed base station at origin (0,0,0)
  - Node 1: Mobile robot (position synced from Gazebo)
- **WiFi Standard**: 802.11a @ 54 Mbps (OFDM)
- **Network Mode**: Ad-hoc (peer-to-peer)
- **IP Range**: 10.1.1.0/24
- **Interface**: TAP bridge devices (`wifi_tap1`, `wifi_tap2`)

### Signal Quality Monitoring
**Metrics Tracked** (lines 49-57):
- RSSI (Received Signal Strength Indicator)
- SNR (Signal-to-Noise Ratio)
- Rx Power

**Implementation**:
- `MonitorSignalCallback` (lines 60-69) captures PHY layer statistics via `MonitorSnifferRx` trace
- Global `g_signalQuality` map stores metrics indexed by node ID

### Traffic Generation
**UDP Echo Server/Client** (lines 173-190):
- Base station (Node 0) → Robot (Node 1)
- 1 packet/second, 64 bytes payload
- Generates traffic for signal quality measurements

### Gazebo Integration
Implements 3 Gazebo system interfaces:
- **`ISystemConfigure`** (line 241): Initialize NS-3 network, spawn NS-3 simulation thread
- **`ISystemPreUpdate`** (line 257): Find "vehicle" model entity in Gazebo scene
- **`ISystemUpdate`** (line 281): Synchronize vehicle pose to NS-3 Node 1 position

### Position Synchronization
- Lines 284-302: Extracts Gazebo vehicle pose (x, y, z coordinates)
- Updates NS-3 `ConstantPositionMobilityModel` for Node 1
- Every 100 updates (lines 305-394):
  - Prints robot position
  - Calculates distance to base station
  - Displays WiFi metrics (RSSI, SNR, expected data rate)

## Technical Details

### Migration History (Lines 5-11)
Upgraded from NS-3 3.29 → 3.45 and Gazebo Classic → Harmonic:
- **API Changes**:
  - `WIFI_PHY_STANDARD_*` → `WIFI_STANDARD_*`
  - Namespace: `gazebo::` → `gz::sim::`
  - Helper initialization: `YansWifiChannelHelper::Default()`

### Real-time Simulation
- Lines 74-76: NS-3 runs in real-time mode with checksums enabled
- Lines 211-216: NS-3 runs in separate thread for 1 year simulation time
- Enables synchronized operation with Gazebo's physics engine

### Network Namespaces Integration
- Lines 194-206: TAP bridges connect NS-3 to Linux network namespaces
- Enables real network connectivity for distributed simulation scenarios
- Allows actual network traffic between simulated nodes

## Data Flow

```
Gazebo Vehicle Pose
       ↓
Update() callback (~100Hz)
       ↓
NS-3 Node 1 Position Update
       ↓
NS-3 WiFi PHY Propagation Model
       ↓
MonitorSignalCallback captures RSSI/SNR
       ↓
Console output every 100 updates
```

## Extending the Plugin

### Adding More Nodes
Lines 131-143 demonstrate the pattern:
1. Increase `COUNT` constant
2. Add positions via `positionAlloc->Add(Vector(x, y, z))`
3. Nodes 2+ can be fixed relay stations or additional robots
4. Modify Update() to sync multiple robot positions

### Example: 3-Node Network
```cpp
const int COUNT = 3;  // Line 46

// In Configure() method (around line 131)
positionAlloc->Add(Vector(0.0, 0.0, 0.0));   // Base station
positionAlloc->Add(Vector(0.0, 0.0, 0.0));   // Robot 1
positionAlloc->Add(Vector(10.0, 0.0, 0.0));  // Relay station
```

## Known Limitations

1. **Hard-coded model name**: "vehicle" string at line 267 - limits flexibility
2. **Single robot support**: Only Node 1 syncs with Gazebo
3. **Console-only output**: No ROS2 topics for signal metrics
4. **Fixed update rate**: 100-update interval for logging (not configurable)
5. **No configuration file**: All parameters hard-coded in source
6. **Thread safety**: No mutex protecting `g_signalQuality` map access

## Build Instructions

```bash
cd ns3_gazebo/ns3_gazebo_plugin
mkdir -p build && cd build
cmake ..
make
```

## Running the Plugin

```bash
# Set plugin path
export GZ_SIM_SYSTEM_PLUGIN_PATH="/home/user/realgazebo/ns3_gazebo/ns3_gazebo_plugin/build"

# Run Gazebo with example world
gz sim ns3_gazebo.sdf
```

## Output Example

The plugin prints signal quality metrics every 100 simulation updates:

```
Robot position: x=5.23 y=2.14 z=0.10
Distance to base station: 5.66 m
RSSI: -45.2 dBm
SNR: 28.3 dB
Expected data rate: 54 Mbps
```

## Future Improvements

1. **Multi-robot support**: Sync multiple Gazebo models to NS-3 nodes
2. **ROS2 integration**: Publish signal metrics as ROS2 topics
3. **Configuration file**: YAML/XML for network parameters
4. **Dynamic node discovery**: Auto-detect Gazebo models instead of hard-coded names
5. **Thread-safe metrics**: Add mutex for `g_signalQuality` access
6. **Configurable logging**: Adjustable update interval and output format
7. **Bi-directional sync**: Update Gazebo based on NS-3 network events (packet loss, delays)

## Files

- **ns3_gazebo_world.cpp**: Main plugin implementation
- **hello_world.cpp**: Simple example plugin
- **ns3_gazebo.sdf**: Example Gazebo world file
- **CMakeLists.txt**: Build configuration

## Dependencies

- NS-3 3.45
- Gazebo Harmonic (gz-sim8)
- Linux network namespaces (optional, for TAP bridge functionality)

## References

- [NS-3 Documentation](https://www.nsnam.org/documentation/)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- Original implementation based on NS-3 3.29 and Gazebo Classic
