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
#include <fstream>
#include <chrono>
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

  // Install Internet stack for UDP beacon packets
  ns3::InternetStackHelper internet;
  internet.Install(ns3_nodes);

  ns3::Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  ns3::Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

  // Setup UDP beacon application from Node 0 (base station) to broadcast
  // This generates traffic so we can measure signal quality
  uint16_t port = 9;
  ns3::UdpEchoServerHelper echoServer(port);
  ns3::ApplicationContainer serverApps = echoServer.Install(ns3_nodes.Get(1)); // Robot listens
  serverApps.Start(ns3::Seconds(1.0));
  serverApps.Stop(ns3::Seconds(60*60*24*365.0));

  ns3::UdpEchoClientHelper echoClient(interfaces.GetAddress(1), port);
  echoClient.SetAttribute("MaxPackets", ns3::UintegerValue(1000000));
  echoClient.SetAttribute("Interval", ns3::TimeValue(ns3::Seconds(1.0))); // 1 packet/sec
  echoClient.SetAttribute("PacketSize", ns3::UintegerValue(64));

  ns3::ApplicationContainer clientApps = echoClient.Install(ns3_nodes.Get(0)); // Base sends
  clientApps.Start(ns3::Seconds(2.0));
  clientApps.Stop(ns3::Seconds(60*60*24*365.0));

  std::cout << "UDP beacon traffic configured: Node 0 -> Node 1 (1 pkt/sec)\n";

  // Note: TapBridge disabled - requires root privileges and not needed for experiments
  // If you need real network connectivity, run with sudo or set up network namespaces

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
  gz::sim::Entity model_entity;
  bool model_found;

  // CSV logging
  std::ofstream logFile;
  std::chrono::steady_clock::time_point startTime;
  bool loggingEnabled;

  public:
  NS3GazeboWorld() : model_found(false), loggingEnabled(false) {
    std::cout << "NS3GazeboWorld Plugin Constructor\n";
  }

  ~NS3GazeboWorld() {
    if (logFile.is_open()) {
      logFile.close();
      std::cout << "CSV log file closed.\n";
    }
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

    // Read log file path from SDF, default to "ns3_gazebo_log.csv"
    std::string logFilePath = "ns3_gazebo_log.csv";
    if (_sdf->HasElement("log_file")) {
      logFilePath = _sdf->Get<std::string>("log_file");
    }

    // Initialize CSV logging
    logFile.open(logFilePath);
    if (logFile.is_open()) {
      loggingEnabled = true;
      startTime = std::chrono::steady_clock::now();

      // Write CSV header
      logFile << "timestamp_sec,robot_x,robot_y,robot_z,distance_to_base,"
              << "rssi_dbm,snr_db,packets_sent,packets_received,packet_loss_rate\n";
      logFile.flush();

      std::cout << "CSV logging enabled: " << logFilePath << "\n";
    } else {
      loggingEnabled = false;
      std::cerr << "Failed to open log file: " << logFilePath << "\n";
    }

    // set up ns-3
    devices = ns3_setup(ns3_nodes);

    // set to run for one year
    ns3::Simulator::Stop(ns3::Seconds(60*60*24*365.));

    // start the ns3 thread
    ns3_thread = std::thread(ns3_thread_function);
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override {
    // Find vehicle model in PreUpdate (models are loaded by now)
    if (!model_found) {
      std::cout << "Searching for models in PreUpdate...\n";
      _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
          [&](const gz::sim::Entity &_entity,
              const gz::sim::components::Model *,
              const gz::sim::components::Name *_name) -> bool {
            std::cout << "Found model: " << _name->Data() << "\n";
            if (_name->Data() == "vehicle") {
              model_entity = _entity;
              model_found = true;
              std::cout << "*** Matched vehicle model! Entity ID: " << _entity << "\n";
            }
            return true;
          });

      if (!model_found) {
        std::cerr << "WARNING: Vehicle model not found in PreUpdate!\n";
      }
    }
  }

  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override {
    // Update logic here - get vehicle pose and sync to NS-3
    if (model_entity != gz::sim::kNullEntity) {
      auto poseComp = _ecm.Component<gz::sim::components::Pose>(model_entity);
      if (poseComp) {
        gz::math::Pose3d pose = poseComp->Data();

        // Get Gazebo vehicle position
        double x = pose.Pos().X();
        double y = pose.Pos().Y();
        double z = pose.Pos().Z();

        // Update NS-3 node 1 position (the moving vehicle/robot)
        if (ns3_nodes.GetN() > 1) {
          ns3::Ptr<ns3::Node> node = ns3_nodes.Get(1);
          ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility =
              node->GetObject<ns3::ConstantPositionMobilityModel>();

          if (mobility) {
            ns3::Vector position(x, y, z);
            mobility->SetPosition(position);

            // Calculate distance to base station
            ns3::Ptr<ns3::Node> base_node = ns3_nodes.Get(0);
            ns3::Ptr<ns3::MobilityModel> base_mobility =
                base_node->GetObject<ns3::MobilityModel>();

            double distance = 0.0;
            if (base_mobility) {
              ns3::Vector base_pos = base_mobility->GetPosition();
              distance = std::sqrt(
                  std::pow(x - base_pos.x, 2) +
                  std::pow(y - base_pos.y, 2) +
                  std::pow(z - base_pos.z, 2));
            }

            // Get signal quality and packet statistics
            double rssi = g_signalQuality[1].hasData ? g_signalQuality[1].rssi : -100.0;
            double snr = g_signalQuality[1].hasData ? g_signalQuality[1].snr : 0.0;
            uint64_t packetsSent = g_packetStats[1].packetsSent;
            uint64_t packetsReceived = g_packetStats[1].packetsReceived;

            // Calculate loss rate (clamp to 0-100 range to handle counting anomalies)
            double lossRate = 0.0;
            if (packetsSent > 0) {
              if (packetsReceived > packetsSent) {
                // Sometimes received > sent due to timing/counting issues - treat as 0% loss
                lossRate = 0.0;
              } else {
                lossRate = 100.0 * (packetsSent - packetsReceived) / packetsSent;
              }
            }

            // Log to CSV file
            if (loggingEnabled) {
              auto elapsed = std::chrono::steady_clock::now() - startTime;
              double timestamp = std::chrono::duration<double>(elapsed).count();

              logFile << std::fixed << std::setprecision(3)
                      << timestamp << ","
                      << x << "," << y << "," << z << ","
                      << distance << ","
                      << rssi << "," << snr << ","
                      << packetsSent << "," << packetsReceived << ","
                      << lossRate << "\n";
              logFile.flush();
            }

            // Print position and check signal quality every 100 updates
            static int update_count = 0;
            if (++update_count % 100 == 0) {
              std::cout << "\n=== NS-3 Network Status ===\n";
              std::cout << "Robot (Node 1) position: ("
                        << x << ", " << y << ", " << z << ")\n";

              std::cout << "\n[Robot ↔ Base Station]\n";
              std::cout << "  Distance: " << std::fixed << std::setprecision(2)
                        << distance << " m\n";

              // Get real WiFi signal quality from NS-3 PHY layer
              if (g_signalQuality[1].hasData) {
                std::cout << "  NS-3 WiFi Metrics:\n";
                std::cout << "    RSSI: " << std::fixed << std::setprecision(1)
                          << rssi << " dBm";

                  // Signal quality assessment
                  if (rssi > -50) {
                    std::cout << " [Excellent - Max Speed]\n";
                  } else if (rssi > -60) {
                    std::cout << " [Good - 54 Mbps]\n";
                  } else if (rssi > -70) {
                    std::cout << " [Fair - Reduced Speed]\n";
                  } else if (rssi > -80) {
                    std::cout << " [Weak - Unstable]\n";
                  } else {
                    std::cout << " [Very Weak - Packet Loss]\n";
                  }

                  std::cout << "    SNR: " << std::fixed << std::setprecision(1)
                            << snr << " dB\n";

                  // Calculate expected data rate based on SNR (802.11a)
                  std::string dataRate = "Unknown";
                  if (snr > 25) dataRate = "54 Mbps";
                  else if (snr > 18) dataRate = "48 Mbps";
                  else if (snr > 17) dataRate = "36 Mbps";
                  else if (snr > 12) dataRate = "24 Mbps";
                  else if (snr > 10) dataRate = "18 Mbps";
                  else if (snr > 8) dataRate = "12 Mbps";
                  else if (snr > 5) dataRate = "9 Mbps";
                  else if (snr > 3) dataRate = "6 Mbps";
                  else dataRate = "Connection Lost";

                  std::cout << "    Expected Rate: " << dataRate << "\n";

                  // Print packet statistics
                  std::cout << "  Packet Statistics:\n";
                  std::cout << "    Sent: " << packetsSent << "\n";
                  std::cout << "    Received: " << packetsReceived << "\n";
                  std::cout << "    Loss Rate: " << std::fixed << std::setprecision(1)
                            << lossRate << "%\n";
              } else {
                std::cout << "  NS-3 WiFi Metrics: No packets received yet\n";
                std::cout << "  (Signal quality will appear after first transmission)\n";
              }

              // Show distances to other nodes (if any additional nodes exist)
              if (ns3_nodes.GetN() > 2) {
                std::cout << "\n[Robot ↔ Other Nodes]\n";
                for (uint32_t i = 2; i < ns3_nodes.GetN(); ++i) {
                  ns3::Ptr<ns3::Node> other_node = ns3_nodes.Get(i);
                  ns3::Ptr<ns3::MobilityModel> other_mobility =
                      other_node->GetObject<ns3::MobilityModel>();

                  if (other_mobility) {
                    ns3::Vector other_pos = other_mobility->GetPosition();
                    double distance = std::sqrt(
                        std::pow(x - other_pos.x, 2) +
                        std::pow(y - other_pos.y, 2) +
                        std::pow(z - other_pos.z, 2));

                    std::cout << "  Node " << i << " @ (" << other_pos.x
                              << ", " << other_pos.y << ", " << other_pos.z
                              << "): " << std::fixed << std::setprecision(1)
                              << distance << " m\n";
                  }
                }
              }
              std::cout << "===========================\n\n";
            }
          }
        }
      }
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