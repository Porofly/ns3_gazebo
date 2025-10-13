# NS3-Gazebo Experiments TODO

## Project Overview

This document outlines the tasks required to implement three network experiments using the NS3-Gazebo integration:

1. **Distance-based Network Quality**: Measure RSSI degradation and packet loss as UAV moves away from base station
2. **Obstacle-induced Communication Shadowing**: Analyze RSSI changes and packet loss when LOS is blocked
3. **Multi-robot Throughput Analysis**: Measure throughput degradation as number of nodes increases (1-10 UAVs)

---

## Phase 1: Foundation (Required for All Experiments) ✅ COMPLETED

### 1.1 Data Logging System ✅
- [x] Add CSV logging capability to `ns3_gazebo_plugin/ns3_gazebo_world.cpp`
  - [x] Create `std::ofstream` member variable for log file
  - [x] Add CSV header: `timestamp,distance,rssi,snr,packets_sent,packets_received,loss_rate`
  - [x] Implement logging method called every update
  - [x] Add flush mechanism for real-time data writing
- [x] Add configuration parameter for log file path via SDF

### 1.2 Packet Statistics Collection ✅
- [x] Modify UDP Echo setup (lines 173-190 in `ns3_gazebo_world.cpp`)
  - [x] Add PHY transmission callback for sent packet counting
  - [x] Add PHY reception callback for received packet counting
  - [x] Implement packet loss rate calculation with overflow protection
  - [x] Store statistics per node in global data structure
- [x] Fixed callback signatures (PhyTxBegin requires 2 parameters)
- [x] Fixed packet loss calculation (handle received > sent cases)

### 1.3 Analysis Scripts ✅
- [x] Create `ns3_gazebo/experiments/` directory
- [x] Write `analyze_data.py` Python script
  - [x] CSV parsing functionality
  - [x] Matplotlib integration for graphs
  - [x] Export to PNG/PDF formats
  - [x] Filter unstable initial data
- [x] Write `plot_distance_vs_rssi.py` for Experiment 1
- [x] All analysis scripts working with data filtering

### 1.4 Build System Updates ✅
- [x] CMakeLists.txt works without changes
- [x] Test compilation successful
- [x] Plugin loads correctly in Gazebo
- [x] TAP bridge disabled (not needed for experiments)

### 1.5 Verification ✅
- [x] 92,395 data points collected over 92.8 seconds
- [x] Robot moved 42m, RSSI ranged from -78.5 to -30.7 dBm
- [x] Distance-RSSI correlation: -0.908 (excellent)
- [x] Packet loss: 8.9% at 42m distance
- [x] All graphs generated successfully

**Actual Time**: ~6 hours (including debugging)

---

## Phase 2: Experiment 1 - Distance-based Network Quality

### 2.1 Gazebo Simulation Scenario
- [ ] Create `ns3_gazebo/scenarios/distance_test.sdf`
  - [ ] Place base station model at origin (0, 0, 0)
  - [ ] Place UAV model at starting position (5, 0, 5)
  - [ ] Add ground plane for visualization
  - [ ] Configure camera views
- [ ] Test SDF loads correctly in Gazebo

### 2.2 UAV Movement Controller
- [ ] Option A: Create ROS2 controller node
  - [ ] Write `experiments/distance_controller.py`
  - [ ] Subscribe to UAV pose
  - [ ] Publish velocity commands to move UAV in straight line (0→100m)
  - [ ] Movement speed: 1 m/s for sufficient sampling
- [ ] Option B: Use Gazebo scripted trajectory
  - [ ] Add `<script>` tag to vehicle model in SDF
  - [ ] Define waypoints along straight path

### 2.3 Experiment Automation
- [ ] Create `experiments/run_distance_experiment.sh`
  - [ ] Launch Gazebo with distance_test.sdf
  - [ ] Start NS3 plugin with logging enabled
  - [ ] Launch UAV controller (if ROS2)
  - [ ] Wait for completion (100 seconds)
  - [ ] Kill processes and collect logs
- [ ] Create `experiments/distance_experiment.py` wrapper
  - [ ] Execute bash script
  - [ ] Call analysis script
  - [ ] Generate report with graphs

### 2.4 Validation
- [ ] Run experiment and verify:
  - [ ] CSV log contains all expected columns
  - [ ] RSSI decreases with distance
  - [ ] Packet loss increases at far distances
  - [ ] Graphs are generated correctly

**Estimated Time**: 2-3 hours

---

## Phase 3: Experiment 2 - Obstacle-induced Shadowing

### 3.1 Ray-casting Implementation
- [ ] Add Gazebo Physics ray-casting to `ns3_gazebo_world.cpp`
  - [ ] Include `gz::physics` headers
  - [ ] Implement `CheckLOS()` method using `World::RayCast()`
  - [ ] Call ray-cast between base station and UAV each update
  - [ ] Store LOS status in member variable
- [ ] Add LOS status to CSV log output

### 3.2 Propagation Loss Adjustment
- [ ] Option A: Custom NS-3 Propagation Model (Complex)
  - [ ] Create `GazeboObstaclePropagationLoss` class extending `PropagationLossModel`
  - [ ] Query Gazebo LOS status from plugin
  - [ ] Apply additional attenuation (20-40 dB) when NLOS
  - [ ] Integrate with `YansWifiChannelHelper`
- [ ] Option B: Simple Attenuation Injection (Recommended)
  - [ ] When LOS blocked, modify NS-3 node's PHY attributes
  - [ ] Add `RxGain` penalty or similar mechanism
  - [ ] Log "LOS" vs "NLOS" state explicitly
- [ ] Test both LOS and NLOS scenarios manually

### 3.3 Obstacle Scenario SDF
- [ ] Create `ns3_gazebo/scenarios/obstacle_test.sdf`
  - [ ] Base station at (0, 0, 0)
  - [ ] Large wall obstacle at (50, 0, 0): 0.5m × 20m × 10m
  - [ ] UAV starting position: (0, 0, 5)
  - [ ] UAV end position: (100, 0, 5)
- [ ] Verify collision geometry renders correctly
- [ ] Test ray-cast detects wall properly

### 3.4 Experiment Automation
- [ ] Create `experiments/run_obstacle_experiment.sh`
  - [ ] Launch Gazebo with obstacle_test.sdf
  - [ ] Start NS3 plugin with LOS tracking
  - [ ] Move UAV through obstacle shadow region
  - [ ] Collect logs with LOS/NLOS annotations
- [ ] Create `experiments/obstacle_experiment.py` wrapper
  - [ ] Execute experiment
  - [ ] Analyze LOS vs NLOS regions
  - [ ] Generate comparison graphs (RSSI, packet loss)

### 3.5 Validation
- [ ] Verify RSSI drops significantly when UAV enters NLOS region (~50m mark)
- [ ] Confirm packet loss increases behind obstacle
- [ ] Check ray-casting accuracy with Gazebo GUI visualization

**Estimated Time**: 6-8 hours

---

## Phase 4: Experiment 3 - Multi-robot Throughput Analysis

### 4.1 Dynamic Node Count Support
- [ ] Modify `ns3_gazebo_world.cpp` for scalable nodes
  - [ ] Replace `COUNT` constant with SDF parameter
  - [ ] Read `<node_count>` from SDF in `Configure()`
  - [ ] Dynamically allocate NS-3 nodes based on parameter
  - [ ] Update IP addressing for variable node count
- [ ] Test with 2, 5, and 10 nodes

### 4.2 Multi-robot Tracking System
- [ ] Modify `PreUpdate()` method
  - [ ] Replace single "vehicle" entity with vector
  - [ ] Search for "vehicle1", "vehicle2", ..., "vehicleN"
  - [ ] Store entity IDs in `std::vector<Entity> vehicleEntities`
- [ ] Modify `Update()` method
  - [ ] Loop through all vehicle entities
  - [ ] Update corresponding NS-3 node positions (Node 1→N)
  - [ ] Maintain synchronized pose for each robot
- [ ] Add per-robot statistics tracking

### 4.3 Throughput Measurement
- [ ] Replace UDP Echo with bulk transfer applications
  - [ ] Install `BulkSendApplication` on base station (Node 0)
  - [ ] Create separate flow to each UAV (Node 1→N)
  - [ ] Install `PacketSink` on each UAV to measure reception
- [ ] Implement periodic throughput calculation
  - [ ] Sample `PacketSink::GetTotalRx()` every 1 second
  - [ ] Calculate instantaneous throughput: `(currentRx - lastRx) / interval`
  - [ ] Store per-node throughput history
- [ ] Log to CSV: `timestamp,node_id,throughput_mbps,packet_loss`

### 4.4 Network Namespace Scaling
- [ ] Verify `nns_setup.py` supports `-c 10`
  - [ ] Test namespace creation for 10 nodes
  - [ ] Verify TAP bridge naming scheme scales
- [ ] Update TAP bridge creation loop in plugin (lines 194-206)
  - [ ] Use dynamic loop based on `node_count`
  - [ ] Generate TAP device names: `wifi_tap1` → `wifi_tap10`

### 4.5 Multi-robot SDF Generation
- [ ] Create `ns3_gazebo/scripts/generate_multi_robot_sdf.py`
  - [ ] Accept command-line argument: `--count N`
  - [ ] Generate SDF with N vehicle models
  - [ ] Position robots in grid or circular formation
  - [ ] Output to `scenarios/multi_robot_N.sdf`
- [ ] Test generation for N=1,2,3,5,10
- [ ] Manually verify generated SDF loads in Gazebo

### 4.6 Experiment Automation
- [ ] Create `experiments/run_multi_robot_experiment.sh`
  - [ ] Accept node count parameter
  - [ ] Setup network namespaces
  - [ ] Generate appropriate SDF
  - [ ] Launch Gazebo simulation
  - [ ] Run for 60 seconds (sufficient for throughput measurement)
  - [ ] Collect and store logs with node count label
- [ ] Create `experiments/multi_robot_experiment.py`
  - [ ] Loop over node counts: [1, 2, 3, 5, 10]
  - [ ] Execute experiment for each configuration
  - [ ] Aggregate results into summary CSV
  - [ ] Generate throughput vs node count graph

### 4.7 Validation
- [ ] Run experiment with 1 robot baseline
- [ ] Verify throughput decreases as node count increases
- [ ] Check for channel contention effects (80211 DCF)
- [ ] Ensure all nodes are tracked correctly in logs

**Estimated Time**: 10-15 hours

---

## Phase 5: Documentation and Reporting

### 5.1 Experiment Documentation
- [ ] Create `ns3_gazebo/experiments/README.md`
  - [ ] Overview of three experiments
  - [ ] Step-by-step instructions to reproduce
  - [ ] Expected results and graphs
  - [ ] Troubleshooting section
- [ ] Add inline code comments to modified plugin code
- [ ] Update main `ns3_gazebo/ns3_gazebo_plugin/README.md`

### 5.2 Results Analysis
- [ ] Create `experiments/results/` directory
- [ ] Generate final graphs for all experiments
- [ ] Write `experiments/RESULTS.md` with findings
  - [ ] Experiment 1: Distance vs RSSI/Loss curves
  - [ ] Experiment 2: LOS vs NLOS comparison table
  - [ ] Experiment 3: Throughput scaling analysis
- [ ] Include raw CSV data for reproducibility

### 5.3 Presentation Materials
- [ ] Create slides summarizing experiments (optional)
- [ ] Record demo videos of simulations (optional)
- [ ] Prepare sample configuration files

**Estimated Time**: 2-3 hours

---

## Testing and Validation

### General Testing Checklist
- [ ] Plugin compiles without warnings
- [ ] Plugin loads in Gazebo without errors
- [ ] NS-3 simulation thread starts correctly
- [ ] CSV logs are created and populated
- [ ] No memory leaks (valgrind check)
- [ ] Clean shutdown of all processes

### Per-Experiment Validation
- [ ] **Experiment 1**: RSSI decreases monotonically with distance
- [ ] **Experiment 2**: RSSI drops sharply at obstacle boundary
- [ ] **Experiment 3**: Average throughput decreases with node count

---

## Known Issues and Workarounds

### Current Limitations
1. **Single robot tracking**: Current code hardcodes "vehicle" model name
   - **Workaround**: Phase 4.2 addresses this
2. **No obstacle awareness**: NS-3 propagation doesn't see Gazebo geometry
   - **Workaround**: Phase 3.1-3.2 implements ray-casting
3. **Console-only output**: No persistent logging
   - **Workaround**: Phase 1.1 adds CSV logging
4. **Thread safety**: `g_signalQuality` map not protected
   - **Future work**: Add mutex if concurrent access detected

### Potential Issues
- **TAP bridge permissions**: May require `sudo` or capabilities
- **Network namespace conflicts**: Clean up with `teardown` script between runs
- **Gazebo performance**: 10 robots may require physics tuning
- **NS-3 real-time sync**: May lag with complex scenarios

---

## Priority Recommendations

### Minimum Viable Product (MVP)
**Goal**: Demonstrate distance-based experiment
- Phase 1.1-1.2: Logging and statistics
- Phase 2: Complete Experiment 1
- **Timeline**: 1 week

### Research Paper Ready
**Goal**: Two solid experiments with publishable results
- MVP + Phase 3: Add obstacle experiment
- Phase 1.3: Professional graphs
- Phase 5: Documentation
- **Timeline**: 2-3 weeks

### Full Implementation
**Goal**: All three experiments operational
- All phases
- Comprehensive validation
- **Timeline**: 4-6 weeks

---

## Dependencies and Prerequisites

### Software Requirements
- [x] Gazebo Harmonic (gz-sim8)
- [x] NS-3 3.45
- [x] ROS2 Jazzy (optional, for advanced control)
- [ ] Python packages: `matplotlib`, `pandas`, `numpy`

### System Configuration
- [ ] Network namespaces enabled in kernel
- [ ] TAP device support configured
- [ ] Sufficient system resources (10 robots = high CPU/RAM)

### Development Tools
- [x] GCC 13.3.0
- [x] CMake 3.28.3
- [ ] `valgrind` for memory leak testing (optional)
- [ ] `gdb` for debugging (optional)

---

## Progress Tracking

### Status Legend
- [ ] Not started
- [x] Completed
- [~] In progress
- [!] Blocked

### Current Status Summary
- **Phase 1**: Not started
- **Phase 2**: Not started
- **Phase 3**: Not started
- **Phase 4**: Not started
- **Phase 5**: Not started

### Last Updated
2025-10-13

---

## Notes

### Design Decisions
- **CSV over ROS2 topics**: Simpler for initial implementation, easier offline analysis
- **Ray-casting over full RF simulation**: Propagation model integration too complex for timeline
- **Bulk transfer over UDP Echo**: Better throughput measurement fidelity

### Future Enhancements
- ROS2 topic publishing for real-time monitoring
- Web-based dashboard for live visualization
- Integration with PX4 autopilot (RealGazebo-PX4)
- Multi-threaded NS-3 for better performance
- PCAP export for Wireshark analysis
- Support for heterogeneous UAV types

### References
- [NS-3 Documentation](https://www.nsnam.org/documentation/)
- [Gazebo Harmonic Plugins](https://gazebosim.org/api/sim/8/createsystemplugins.html)
- [IEEE 802.11a Standard](https://en.wikipedia.org/wiki/IEEE_802.11a-1999)
- Current plugin implementation: [ns3_gazebo_world.cpp](ns3_gazebo_plugin/ns3_gazebo_world.cpp)
