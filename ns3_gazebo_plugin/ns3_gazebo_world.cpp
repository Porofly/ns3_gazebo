/**
 * @file ns3_gazebo_world.cpp
 * @brief NS-3 WiFi Network Simulator Integration with Gazebo Harmonic
 *
 * UPGRADE NOTES (NS-3 3.29 → 3.45, Gazebo Classic → Harmonic):
 * - Migrated from gazebo::WorldPlugin to gz::sim::System architecture
 * - Updated WiFi Standard API: WIFI_PHY_STANDARD_* → WIFI_STANDARD_*
 * - Changed namespace: gazebo → gz::sim, ignition → gz
 * - Updated Helper initialization: YansWifiChannelHelper::Default() pattern
 * - Added C++20 standard support for NS-3 3.45 compatibility
 */

#include <cstdio>
#include <thread>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <sstream>

// NS-3 headers
#include "ns3/core-module.h"
#include "ns3/node-container.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"

// Gazebo Harmonic headers
#include <gz/sim/Server.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>


namespace ns3_gazebo_world {

// Number of NS-3 nodes to create
// Node 0: Base station (fixed at origin)
// Node 1: Mobile robot (Gazebo controlled)
// Nodes 2+: Additional fixed nodes or robots (can be added as needed)
static const int COUNT=2;  // Currently: 1 base station + 1 robot

// Global variables to store WiFi signal quality metrics
struct SignalQuality {
  double rssi = -100.0;  // Received Signal Strength Indicator (dBm)
  double snr = 0.0;      // Signal-to-Noise Ratio (dB)
  double rxPower = -100.0; // Received power (dBm)
  uint64_t lastUpdateTime = 0;
  bool hasData = false;
};

static std::map<uint32_t, SignalQuality> g_signalQuality;

// Packet statistics tracking
struct PacketStats {
  uint64_t packetsSent = 0;
  uint64_t packetsReceived = 0;
  double lossRate = 0.0;
};

static std::map<uint32_t, PacketStats> g_packetStats;

// Callback to capture PHY layer statistics (NS-3 3.45 signature)
void MonitorSignalCallback(uint32_t nodeId, ns3::Ptr<const ns3::Packet> packet,
                           uint16_t channelFreqMhz, ns3::WifiTxVector txVector,
                           ns3::MpduInfo aMpdu, ns3::SignalNoiseDbm signalNoise,
                           uint16_t staId) {
  g_signalQuality[nodeId].rssi = signalNoise.signal;
  g_signalQuality[nodeId].snr = signalNoise.signal - signalNoise.noise;
  g_signalQuality[nodeId].rxPower = signalNoise.signal;
  g_signalQuality[nodeId].lastUpdateTime = ns3::Simulator::Now().GetNanoSeconds();
  g_signalQuality[nodeId].hasData = true;

  // Count received packets
  g_packetStats[nodeId].packetsReceived++;
}

// Callback for packet transmission (PhyTxBegin signature: packet, txPowerW)
void PacketTxCallback(uint32_t nodeId, ns3::Ptr<const ns3::Packet> packet, double txPowerW) {
  g_packetStats[nodeId].packetsSent++;
}

ns3::NetDeviceContainer ns3_setup(ns3::NodeContainer& ns3_nodes) {

  // run ns3 real-time with checksums
  ns3::GlobalValue::Bind("SimulatorImplementationType",
                          ns3::StringValue("ns3::RealtimeSimulatorImpl"));
  ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

  // Create ns3_nodes
  ns3_nodes.Create(COUNT);

  // Physical layer - use default configuration to avoid ObjectFactory issues
  // UPGRADE: Changed from default constructor to Default() method for NS-3 3.45 compatibility
  ns3::YansWifiChannelHelper wifiChannel = ns3::YansWifiChannelHelper::Default();
  ns3::YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(wifiChannel.Create());

  // WiFi settings
  ns3::WifiHelper wifi;
  // UPGRADE: Updated API from WIFI_PHY_STANDARD_80211a to WIFI_STANDARD_80211a (NS-3 3.45)
  wifi.SetStandard(ns3::WIFI_STANDARD_80211a);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                          "DataMode", ns3::StringValue("OfdmRate54Mbps"));

  // ad-hoc Wifi network
  ns3::WifiMacHelper wifiMac;
  wifiMac.SetType("ns3::AdhocWifiMac");

  // Install the wireless devices onto our ghost ns3_nodes.
  ns3::NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, ns3_nodes);

  // Setup PHY monitoring callbacks to capture real signal quality
  for (uint32_t i = 0; i < ns3_nodes.GetN(); ++i) {
    ns3::Ptr<ns3::NetDevice> dev = devices.Get(i);
    ns3::Ptr<ns3::WifiNetDevice> wifiDev = ns3::DynamicCast<ns3::WifiNetDevice>(dev);
    if (wifiDev) {
      ns3::Ptr<ns3::WifiPhy> phy = wifiDev->GetPhy();
      if (phy) {
        // Connect to MonitorSniffer trace to get RSSI/SNR from received packets
        phy->TraceConnectWithoutContext("MonitorSnifferRx",
          ns3::MakeBoundCallback(&MonitorSignalCallback, i));
        // Connect to PhyTx trace to count transmitted packets
        phy->TraceConnectWithoutContext("PhyTxBegin",
          ns3::MakeBoundCallback(&PacketTxCallback, i));
        std::cout << "Signal monitoring and packet counting enabled for Node " << i << "\n";
      }
    }
  }

  // Enable PCAP tracing (optional - uncomment to capture packets)
  // wifiPhy.EnablePcapAll("ns3_gazebo_wifi");

  // ========== NS-3 Node Position Configuration ==========
  // Configure initial positions for all NS-3 nodes
  // Node 0: Fixed base station/AP at world origin
  // Node 1: Mobile robot (position updated by Gazebo in real-time)
  // Nodes 2+: Additional nodes can be added below

  ns3::Ptr<ns3::ListPositionAllocator>positionAlloc =
                         ns3::CreateObject<ns3::ListPositionAllocator>();

  positionAlloc->Add(ns3::Vector(0.0, 0.0, 0.0));   // Node 0: Base station at origin
  positionAlloc->Add(ns3::Vector(0.0, 0.0, 0.0));   // Node 1: Robot (updated by Gazebo)

  // ===== Add more nodes here if needed =====
  // Example: Add fixed relay nodes or additional base stations
  // Uncomment and modify the lines below to add more nodes
  // Remember to update COUNT at the top of this file!

  // positionAlloc->Add(ns3::Vector(10.0, 0.0, 0.0));  // Node 2: Fixed at 10m
  // positionAlloc->Add(ns3::Vector(20.0, 0.0, 0.0));  // Node 3: Fixed at 20m
  // positionAlloc->Add(ns3::Vector(0.0, 10.0, 0.0));  // Node 4: Fixed at (0,10,0)

  // Or use a loop for multiple nodes:
  // for (int i=2; i<COUNT; i++) {
  //   positionAlloc->Add(ns3::Vector(i * 5.0, 0.0, 0.0)); // Nodes at 5m intervals
  // }
  // ==========================================

  ns3::MobilityHelper mobility;
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(ns3_nodes);

  std::cout << "NS-3 Node Configuration:\n";
  std::cout << "  Node 0: Base Station (fixed at origin)\n";
  std::cout << "  Node 1: Mobile Robot (Gazebo controlled)\n";
  if (COUNT > 2) {
    for (int i=2; i<COUNT; i++) {
      ns3::Ptr<ns3::MobilityModel> mob = ns3_nodes.Get(i)->GetObject<ns3::MobilityModel>();
      if (mob) {
        ns3::Vector pos = mob->GetPosition();
        std::cout << "  Node " << i << ": Fixed at ("
                  << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
      }
    }
  }

  // ========== TAP Bridge Configuration for Network Namespaces ==========
  // Setup TAP bridges to connect NS-3 nodes to Linux network namespaces
  // Each NS-3 node gets a TAP device: wifi_tap1 for Node 0, wifi_tap2 for Node 1, etc.
  //
  // IMPORTANT: TapBridge does NOT use Internet Stack!
  // IP addresses are configured in Linux network namespaces, not in NS-3.
  // NS-3 only simulates the WiFi PHY/MAC layer.

  std::cout << "\n=== Setting up TAP Bridges ===\n";
  ns3::TapBridgeHelper tapBridge;
  tapBridge.SetAttribute("Mode", ns3::StringValue("UseLocal"));

  for (uint32_t i = 0; i < ns3_nodes.GetN(); ++i) {
    std::string tapDeviceName = "wifi_tap" + std::to_string(i + 1);
    tapBridge.SetAttribute("DeviceName", ns3::StringValue(tapDeviceName));

    try {
      tapBridge.Install(ns3_nodes.Get(i), devices.Get(i));
      std::cout << "  Node " << i << " -> TAP device: " << tapDeviceName << " [OK]\n";
    } catch (const std::exception& e) {
      std::cerr << "  Node " << i << " -> TAP device: " << tapDeviceName << " [FAILED: " << e.what() << "]\n";
    }
  }
  std::cout << "TAP bridges configured successfully!\n";
  std::cout << "WiFi simulation: 802.11a, 54Mbps, PHY/MAC only\n";
  std::cout << "IP stack runs in Linux namespaces (not NS-3)\n";
  std::cout << "==========================================\n\n";

  return devices;
}

static void ns3_thread_function(void) {
  std::cout << "Starting ns-3 Wifi simulator in thread.\n";
  ns3::Simulator::Run();
  ns3::Simulator::Destroy();
  std::cout << "Ending ns-3 Wifi simulator in thread.\n";
}

class NS3GazeboWorld : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPreUpdate,
                       public gz::sim::ISystemUpdate {
  private:
  ns3::NodeContainer ns3_nodes;
  ns3::NetDeviceContainer devices;
  std::thread ns3_thread;
  std::map<std::string, gz::sim::Entity> robot_entities;  // Map robot name -> entity
  int models_search_counter;  // Counter for periodic robot search

  public:
  NS3GazeboWorld() : models_search_counter(0) {
    std::cout << "NS3GazeboWorld Plugin Constructor\n";
  }

  ~NS3GazeboWorld() {
    if (ns3_thread.joinable()) {
      ns3_thread.join(); // gracefully let the robot thread stop
      std::cout << "Stopped ns-3 Wifi simulator in main.\n";
    }
  }

  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override {
    std::cout << "NS3GazeboWorld Plugin Configure\n";

    // set up ns-3
    devices = ns3_setup(ns3_nodes);

    // set to run for one year
    ns3::Simulator::Stop(ns3::Seconds(60*60*24*365.));

    // start the ns3 thread
    ns3_thread = std::thread(ns3_thread_function);
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override {
    // Periodically search for new robot models (every 300 frames ~= 3 seconds)
    // This allows dynamic spawning of robots after simulation starts
    models_search_counter++;

    if (models_search_counter % 300 == 1) {  // Search on first update and every 300 frames
      int robots_before = robot_entities.size();

      _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
          [&](const gz::sim::Entity &_entity,
              const gz::sim::components::Model *,
              const gz::sim::components::Name *_name) -> bool {
            std::string model_name = _name->Data();

            // Match any model that starts with "robot"
            if (model_name.find("robot") == 0) {
              // Only add if not already tracked
              if (robot_entities.find(model_name) == robot_entities.end()) {
                robot_entities[model_name] = _entity;
                std::cout << "*** NEW robot detected: " << model_name
                          << " (Entity ID: " << _entity << ")\n";
              }
            }
            return true;
          });

      int robots_after = robot_entities.size();
      if (robots_before != robots_after) {
        std::cout << "Total robots tracked: " << robots_after << "\n";
      }
    }
  }

  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override {
    // Update all robot positions to NS-3
    // Mapping: robot1 -> NS-3 Node 0, robot2 -> NS-3 Node 1

    static int update_count = 0;
    update_count++;

    int robot_idx = 0;
    for (auto& [robot_name, entity] : robot_entities) {
      if (robot_idx >= ns3_nodes.GetN()) {
        std::cerr << "WARNING: More robots than NS-3 nodes! Skipping " << robot_name << "\n";
        break;
      }

      auto poseComp = _ecm.Component<gz::sim::components::Pose>(entity);
      if (poseComp) {
        gz::math::Pose3d pose = poseComp->Data();

        // Get Gazebo robot position
        double x = pose.Pos().X();
        double y = pose.Pos().Y();
        double z = pose.Pos().Z();

        // Update corresponding NS-3 node position
        ns3::Ptr<ns3::Node> node = ns3_nodes.Get(robot_idx);
        ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility =
            node->GetObject<ns3::ConstantPositionMobilityModel>();

        if (mobility) {
          ns3::Vector position(x, y, z);
          mobility->SetPosition(position);
        }
      }

      robot_idx++;
    }

    // Print network status periodically
    if (update_count % 100 == 0 && !robot_entities.empty()) {
      std::cout << "\n=== NS-3 Multi-Robot Network Status ===\n";

      int idx = 0;
      for (auto& [robot_name, entity] : robot_entities) {
        auto poseComp = _ecm.Component<gz::sim::components::Pose>(entity);
        if (poseComp) {
          gz::math::Pose3d pose = poseComp->Data();
          std::cout << robot_name << " (NS-3 Node " << idx << "): ("
                    << std::fixed << std::setprecision(2)
                    << pose.Pos().X() << ", "
                    << pose.Pos().Y() << ", "
                    << pose.Pos().Z() << ")\n";

          // Show signal quality if available
          if (g_signalQuality[idx].hasData) {
            std::cout << "  RSSI: " << std::fixed << std::setprecision(1)
                      << g_signalQuality[idx].rssi << " dBm, "
                      << "SNR: " << g_signalQuality[idx].snr << " dB\n";
            std::cout << "  Packets RX: " << g_packetStats[idx].packetsReceived << "\n";
          }
        }
        idx++;
      }

      // Calculate inter-robot distance if we have 2 robots
      if (robot_entities.size() >= 2) {
        auto it = robot_entities.begin();
        auto robot1_entity = it->second;
        ++it;
        auto robot2_entity = it->second;

        auto pose1 = _ecm.Component<gz::sim::components::Pose>(robot1_entity);
        auto pose2 = _ecm.Component<gz::sim::components::Pose>(robot2_entity);

        if (pose1 && pose2) {
          double dx = pose1->Data().Pos().X() - pose2->Data().Pos().X();
          double dy = pose1->Data().Pos().Y() - pose2->Data().Pos().Y();
          double dz = pose1->Data().Pos().Z() - pose2->Data().Pos().Z();
          double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

          std::cout << "\nInter-robot distance: " << std::fixed << std::setprecision(2)
                    << distance << " m\n";
        }
      }

      std::cout << "======================================\n\n";
    }
  }
};  // NS3GazeboWorld class

} // namespace ns3_gazebo_world

#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(ns3_gazebo_world::NS3GazeboWorld,
              gz::sim::System,
              ns3_gazebo_world::NS3GazeboWorld::ISystemConfigure,
              ns3_gazebo_world::NS3GazeboWorld::ISystemPreUpdate,
              ns3_gazebo_world::NS3GazeboWorld::ISystemUpdate)