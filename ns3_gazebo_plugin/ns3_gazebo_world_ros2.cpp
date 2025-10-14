/**
 * @file ns3_gazebo_world_ros2.cpp
 * @brief NS-3 WiFi Network Simulator Integration with Gazebo Harmonic + ROS2 Dynamic Robot Spawning
 *
 * FEATURES:
 * - Dynamic robot spawning via ROS2 launch files
 * - ROS2 service-based NS-3 node registration
 * - Automatic model detection with PostUpdate
 * - Multi-robot position synchronization
 * - Thread-safe operation with mutex protection
 *
 * UPGRADE NOTES (NS-3 3.29 → 3.45, Gazebo Classic → Harmonic):
 * - Migrated from gazebo::WorldPlugin to gz::sim::System architecture
 * - Updated WiFi Standard API: WIFI_PHY_STANDARD_* → WIFI_STANDARD_*
 * - Changed namespace: gazebo → gz::sim, ignition → gz
 * - Updated Helper initialization: YansWifiChannelHelper::Default() pattern
 * - Added C++20 standard support for NS-3 3.45 compatibility
 * - Added ROS2 Jazzy integration for dynamic robot management
 */

#include <cstdio>
#include <thread>
#include <mutex>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <chrono>
#include <sstream>
#include <map>
#include <set>
#include <string>

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

// ROS2 headers
#include <rclcpp/rclcpp.hpp>
#include "ns3_gazebo_interfaces/srv/register_robot.hpp"
#include "ns3_gazebo_interfaces/srv/unregister_robot.hpp"
#include "ns3_gazebo_interfaces/msg/network_status.hpp"

namespace ns3_gazebo_world {

// ============================================================================
// Global Data Structures
// ============================================================================

// WiFi signal quality metrics
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

// Robot binding information
struct RobotBinding {
  gz::sim::Entity gazeboEntity;      // Gazebo entity ID
  uint32_t ns3NodeId;                 // NS-3 node ID
  std::string modelName;              // Model name for debugging
  bool isActive;                      // Is this binding active?
  std::chrono::steady_clock::time_point lastUpdate;
  std::chrono::steady_clock::time_point creationTime;
};

// ============================================================================
// NS-3 Callbacks
// ============================================================================

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

// ============================================================================
// NS-3 Network Setup Function
// ============================================================================

ns3::NetDeviceContainer ns3_setup(ns3::NodeContainer& ns3_nodes,
                                   int maxNodes,
                                   const gz::math::Vector3d& baseStationPos) {

  std::cout << "\n=== NS-3 Network Setup ===\n";

  // run ns3 real-time with checksums
  ns3::GlobalValue::Bind("SimulatorImplementationType",
                          ns3::StringValue("ns3::RealtimeSimulatorImpl"));
  ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

  // Create node pool
  ns3_nodes.Create(maxNodes);
  std::cout << "Created NS-3 node pool: " << maxNodes << " nodes\n";

  // Physical layer - use default configuration
  ns3::YansWifiChannelHelper wifiChannel = ns3::YansWifiChannelHelper::Default();
  ns3::YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(wifiChannel.Create());

  // WiFi settings
  ns3::WifiHelper wifi;
  wifi.SetStandard(ns3::WIFI_STANDARD_80211a);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                          "DataMode", ns3::StringValue("OfdmRate54Mbps"));

  // ad-hoc WiFi network
  ns3::WifiMacHelper wifiMac;
  wifiMac.SetType("ns3::AdhocWifiMac");

  // Install the wireless devices onto all nodes
  ns3::NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, ns3_nodes);
  std::cout << "WiFi devices installed: 802.11a @ 54Mbps\n";

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
      }
    }
  }
  std::cout << "Signal monitoring enabled for all nodes\n";

  // Configure initial positions - all start at origin, will be updated dynamically
  ns3::Ptr<ns3::ListPositionAllocator> positionAlloc =
                         ns3::CreateObject<ns3::ListPositionAllocator>();

  // Set base station position (Node 0)
  positionAlloc->Add(ns3::Vector(baseStationPos.X(), baseStationPos.Y(), baseStationPos.Z()));

  // All other nodes start at origin (will be updated when robots spawn)
  for (int i = 1; i < maxNodes; ++i) {
    positionAlloc->Add(ns3::Vector(0.0, 0.0, 0.0));
  }

  ns3::MobilityHelper mobility;
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(ns3_nodes);

  std::cout << "Node 0 (Base Station) positioned at ("
            << baseStationPos.X() << ", "
            << baseStationPos.Y() << ", "
            << baseStationPos.Z() << ")\n";

  // Install Internet stack for UDP beacon packets
  ns3::InternetStackHelper internet;
  internet.Install(ns3_nodes);

  ns3::Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.1.0", "255.255.255.0");
  ns3::Ipv4InterfaceContainer interfaces = ipv4.Assign(devices);

  std::cout << "IP addresses assigned: 10.1.1.0/24\n";

  // Setup UDP echo applications for traffic generation
  // This generates periodic packets so we can measure signal quality
  uint16_t port = 9;

  // UDP Echo Server on all nodes (so any node can receive)
  ns3::UdpEchoServerHelper echoServer(port);
  for (uint32_t i = 0; i < ns3_nodes.GetN(); ++i) {
    ns3::ApplicationContainer serverApp = echoServer.Install(ns3_nodes.Get(i));
    serverApp.Start(ns3::Seconds(1.0));
    serverApp.Stop(ns3::Seconds(60*60*24*365.0));
  }

  // UDP Echo Client on Node 0 (base station) sending to broadcast
  // This ensures all robots receive packets for RSSI measurement
  ns3::UdpEchoClientHelper echoClient(ns3::Ipv4Address("10.1.1.255"), port);
  echoClient.SetAttribute("MaxPackets", ns3::UintegerValue(1000000));
  echoClient.SetAttribute("Interval", ns3::TimeValue(ns3::Seconds(1.0))); // 1 packet/sec
  echoClient.SetAttribute("PacketSize", ns3::UintegerValue(64));

  ns3::ApplicationContainer clientApp = echoClient.Install(ns3_nodes.Get(0));
  clientApp.Start(ns3::Seconds(2.0));
  clientApp.Stop(ns3::Seconds(60*60*24*365.0));

  std::cout << "UDP echo applications configured:\n";
  std::cout << "  - Echo servers on all nodes (port " << port << ")\n";
  std::cout << "  - Base station broadcasting to 10.1.1.255 (1 pkt/sec)\n";
  std::cout << "=========================\n\n";

  return devices;
}

// NS-3 simulator thread function
static void ns3_thread_function(void) {
  std::cout << "NS-3 simulator thread started\n";
  ns3::Simulator::Run();
  ns3::Simulator::Destroy();
  std::cout << "NS-3 simulator thread stopped\n";
}

// ============================================================================
// Gazebo System Plugin Class
// ============================================================================

class NS3GazeboWorld : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemPostUpdate,
                       public gz::sim::ISystemUpdate {

public:
  NS3GazeboWorld() :
    maxNodes(100),
    baseStationNodeId(0),
    loggingEnabled(false),
    ecmPtr(nullptr) {
  }

  ~NS3GazeboWorld() override {
    // Cleanup
    std::cout << "\nShutting down NS3GazeboWorld plugin...\n";

    // Stop ROS2
    if (rosNode) {
      rclcpp::shutdown();
    }
    if (rosSpinThread.joinable()) {
      rosSpinThread.join();
    }

    // Close log file
    if (logFile.is_open()) {
      logFile.close();
    }

    // Stop NS-3
    if (ns3_thread.joinable()) {
      ns3_thread.join();
    }

    std::cout << "Plugin shutdown complete\n";
  }

  // ========== ISystemConfigure ==========
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &_eventMgr) override {

    std::cout << "\n";
    std::cout << "================================================\n";
    std::cout << "  NS3-Gazebo World Plugin (ROS2 Integration)   \n";
    std::cout << "================================================\n\n";

    // Read configuration from SDF
    if (_sdf->HasElement("max_nodes")) {
      maxNodes = _sdf->Get<int>("max_nodes");
    }
    std::cout << "Max NS-3 nodes: " << maxNodes << "\n";

    if (_sdf->HasElement("base_station_node_id")) {
      baseStationNodeId = _sdf->Get<uint32_t>("base_station_node_id");
    }
    std::cout << "Base station node ID: " << baseStationNodeId << "\n";

    gz::math::Vector3d basePos(0, 0, 0);
    if (_sdf->HasElement("base_station_position")) {
      basePos = _sdf->Get<gz::math::Vector3d>("base_station_position");
    }
    std::cout << "Base station position: ("
              << basePos.X() << ", "
              << basePos.Y() << ", "
              << basePos.Z() << ")\n";

    // Setup CSV logging
    std::string logFilePath = "ns3_gazebo_log.csv";
    if (_sdf->HasElement("log_file")) {
      logFilePath = _sdf->Get<std::string>("log_file");
    }

    logFile.open(logFilePath);
    if (logFile.is_open()) {
      loggingEnabled = true;
      startTime = std::chrono::steady_clock::now();

      // Write CSV header
      logFile << "timestamp_sec,model_name,ns3_node_id,robot_x,robot_y,robot_z,"
              << "distance_to_base,rssi_dbm,snr_db,packets_sent,packets_received,"
              << "packets_lost,packet_loss_rate\n";
      logFile.flush();

      std::cout << "CSV logging enabled: " << logFilePath << "\n";
    } else {
      loggingEnabled = false;
      std::cerr << "WARNING: Failed to open log file: " << logFilePath << "\n";
    }

    // Setup NS-3 network with node pool
    std::cout << "\n";
    devices = ns3_setup(ns3_nodes, maxNodes, basePos);

    // Mark base station as used
    usedNodeIds.insert(baseStationNodeId);

    // Initialize ROS2
    std::cout << "\n=== ROS2 Integration Setup ===\n";
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
      std::cout << "ROS2 initialized\n";
    }

    rosNode = std::make_shared<rclcpp::Node>("ns3_gazebo_world");
    std::cout << "ROS2 node created: ns3_gazebo_world\n";

    // Create service: register_robot
    registerService = rosNode->create_service<ns3_gazebo_interfaces::srv::RegisterRobot>(
      "/ns3_gazebo/register_robot",
      [this](const std::shared_ptr<ns3_gazebo_interfaces::srv::RegisterRobot::Request> req,
             std::shared_ptr<ns3_gazebo_interfaces::srv::RegisterRobot::Response> res) {
        std::lock_guard<std::mutex> lock(bindingMutex);
        this->handleRegisterRobot(req, res);
      });
    std::cout << "Service created: /ns3_gazebo/register_robot\n";

    // Create service: unregister_robot
    unregisterService = rosNode->create_service<ns3_gazebo_interfaces::srv::UnregisterRobot>(
      "/ns3_gazebo/unregister_robot",
      [this](const std::shared_ptr<ns3_gazebo_interfaces::srv::UnregisterRobot::Request> req,
             std::shared_ptr<ns3_gazebo_interfaces::srv::UnregisterRobot::Response> res) {
        std::lock_guard<std::mutex> lock(bindingMutex);
        this->handleUnregisterRobot(req, res);
      });
    std::cout << "Service created: /ns3_gazebo/unregister_robot\n";

    // Create publisher for network status
    networkStatusPub = rosNode->create_publisher<ns3_gazebo_interfaces::msg::NetworkStatus>(
      "/ns3_gazebo/network_status", 10);
    std::cout << "Publisher created: /ns3_gazebo/network_status\n";

    // Spin ROS2 in separate thread
    rosSpinThread = std::thread([this]() {
      rclcpp::spin(rosNode);
    });
    std::cout << "ROS2 spin thread started\n";
    std::cout << "==============================\n\n";

    // Start NS-3 simulator thread
    ns3::Simulator::Stop(ns3::Seconds(60*60*24*365.));  // Run for 1 year
    ns3_thread = std::thread(ns3_thread_function);

    std::cout << "✓ Plugin configuration complete!\n";
    std::cout << "  Waiting for robots to be spawned...\n\n";
  }

  // ========== ISystemPostUpdate ==========
  void PostUpdate(const gz::sim::UpdateInfo &_info,
                  const gz::sim::EntityComponentManager &_ecm) override {

    // Store ECM pointer for service callbacks
    ecmPtr = const_cast<gz::sim::EntityComponentManager*>(&_ecm);

    // Detect newly spawned models
    _ecm.EachNew<gz::sim::components::Model,
                 gz::sim::components::Name>(
      [&](const gz::sim::Entity &_entity,
          const gz::sim::components::Model *,
          const gz::sim::components::Name *_name) -> bool {

        std::lock_guard<std::mutex> lock(bindingMutex);

        // Skip if already bound
        if (robotBindings.find(_entity) != robotBindings.end()) {
          return true;
        }

        std::string modelName = _name->Data();
        std::cout << "New model detected: " << modelName << "\n";
        std::cout << "  (Use ROS2 service to bind to NS-3: "
                  << "ros2 service call /ns3_gazebo/register_robot ...)\n";

        // Note: Auto-binding removed - all robots must be registered via ROS2 service
        // This provides explicit control over which models get NS-3 network capabilities

        return true;
      });
  }

  // ========== ISystemUpdate ==========
  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override {

    std::lock_guard<std::mutex> lock(bindingMutex);

    // Update all bound robots
    for (auto& [entity, binding] : robotBindings) {
      if (!binding.isActive) continue;

      auto poseComp = _ecm.Component<gz::sim::components::Pose>(entity);
      if (!poseComp) continue;

      gz::math::Pose3d pose = poseComp->Data();
      double x = pose.Pos().X();
      double y = pose.Pos().Y();
      double z = pose.Pos().Z();

      // Update NS-3 mobility model
      ns3::Ptr<ns3::Node> node = ns3_nodes.Get(binding.ns3NodeId);
      ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility =
          node->GetObject<ns3::ConstantPositionMobilityModel>();

      if (mobility) {
        ns3::Vector position(x, y, z);
        mobility->SetPosition(position);
        binding.lastUpdate = std::chrono::steady_clock::now();

        // Log to CSV if enabled
        if (loggingEnabled) {
          logRobotData(binding, x, y, z);
        }
      }
    }

    // Periodic status output and ROS2 publishing
    static int update_count = 0;
    if (++update_count % 100 == 0) {
      printNetworkStatus();
      publishNetworkStatus();
    }
  }

private:
  // Configuration
  int maxNodes;
  uint32_t baseStationNodeId;
  bool loggingEnabled;

  // NS-3
  ns3::NodeContainer ns3_nodes;
  ns3::NetDeviceContainer devices;
  std::thread ns3_thread;

  // Robot bindings
  std::map<gz::sim::Entity, RobotBinding> robotBindings;
  std::set<uint32_t> usedNodeIds;
  std::mutex bindingMutex;

  // Gazebo
  gz::sim::EntityComponentManager* ecmPtr;

  // ROS2
  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Service<ns3_gazebo_interfaces::srv::RegisterRobot>::SharedPtr registerService;
  rclcpp::Service<ns3_gazebo_interfaces::srv::UnregisterRobot>::SharedPtr unregisterService;
  rclcpp::Publisher<ns3_gazebo_interfaces::msg::NetworkStatus>::SharedPtr networkStatusPub;
  std::thread rosSpinThread;

  // Logging
  std::ofstream logFile;
  std::chrono::steady_clock::time_point startTime;

  // ========== Helper Methods ==========

  uint32_t getNextAvailableNodeId() {
    for (uint32_t i = 1; i < ns3_nodes.GetN(); ++i) {
      if (usedNodeIds.count(i) == 0) {
        return i;
      }
    }
    return ns3_nodes.GetN();  // No available node
  }

  bool bindRobotToNode(gz::sim::Entity entity,
                       const std::string& modelName,
                       uint32_t nodeId,
                       const gz::sim::EntityComponentManager &_ecm) {

    // Validate node ID
    if (nodeId >= ns3_nodes.GetN()) {
      std::cerr << "ERROR: Node ID " << nodeId << " exceeds max nodes ("
               << ns3_nodes.GetN() << ")\n";
      return false;
    }

    if (usedNodeIds.count(nodeId) > 0) {
      std::cerr << "ERROR: Node ID " << nodeId << " already in use\n";
      return false;
    }

    // Get initial pose
    auto poseComp = _ecm.Component<gz::sim::components::Pose>(entity);
    if (!poseComp) {
      std::cerr << "ERROR: Cannot get pose for " << modelName << "\n";
      return false;
    }

    gz::math::Pose3d pose = poseComp->Data();

    // Set NS-3 node initial position
    ns3::Ptr<ns3::Node> node = ns3_nodes.Get(nodeId);
    ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility =
        node->GetObject<ns3::ConstantPositionMobilityModel>();

    if (mobility) {
      ns3::Vector position(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
      mobility->SetPosition(position);
    } else {
      std::cerr << "ERROR: NS-3 Node " << nodeId << " has no mobility model\n";
      return false;
    }

    // Create binding
    RobotBinding binding;
    binding.gazeboEntity = entity;
    binding.ns3NodeId = nodeId;
    binding.modelName = modelName;
    binding.isActive = true;
    binding.lastUpdate = std::chrono::steady_clock::now();
    binding.creationTime = std::chrono::steady_clock::now();

    robotBindings[entity] = binding;
    usedNodeIds.insert(nodeId);

    return true;
  }

  void handleRegisterRobot(
      const std::shared_ptr<ns3_gazebo_interfaces::srv::RegisterRobot::Request> req,
      std::shared_ptr<ns3_gazebo_interfaces::srv::RegisterRobot::Response> res) {

    std::cout << "\n[ROS2 Service] Register request: " << req->model_name << "\n";

    // Find entity by name
    gz::sim::Entity entity = gz::sim::kNullEntity;

    if (ecmPtr) {
      ecmPtr->Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *_name) -> bool {
          if (_name->Data() == req->model_name) {
            entity = _entity;
            return false;  // Stop searching
          }
          return true;
        });
    }

    if (entity == gz::sim::kNullEntity) {
      res->success = false;
      res->message = "Model not found: " + req->model_name;
      std::cerr << "ERROR: " << res->message << "\n";
      return;
    }

    // Check if already bound
    if (robotBindings.find(entity) != robotBindings.end()) {
      res->success = false;
      res->assigned_node_id = robotBindings[entity].ns3NodeId;
      res->message = "Already bound to Node " + std::to_string(res->assigned_node_id);
      std::cout << "WARNING: " << res->message << "\n";
      return;
    }

    // Determine node ID
    uint32_t nodeId = req->requested_node_id;
    if (nodeId == 0) {
      nodeId = getNextAvailableNodeId();
    }

    // Validate node ID
    if (nodeId >= ns3_nodes.GetN()) {
      res->success = false;
      res->message = "Node ID " + std::to_string(nodeId) + " exceeds max nodes";
      std::cerr << "ERROR: " << res->message << "\n";
      return;
    }

    if (usedNodeIds.count(nodeId) > 0) {
      res->success = false;
      res->message = "Node ID " + std::to_string(nodeId) + " already in use";
      std::cerr << "ERROR: " << res->message << "\n";
      return;
    }

    // Bind robot to NS-3 node
    bool success = bindRobotToNode(entity, req->model_name, nodeId, *ecmPtr);

    if (success) {
      res->success = true;
      res->assigned_node_id = nodeId;
      res->message = "Successfully bound to NS-3 Node " + std::to_string(nodeId);
      std::cout << "✓ " << res->message << "\n\n";
    } else {
      res->success = false;
      res->message = "Failed to bind robot";
      std::cerr << "ERROR: " << res->message << "\n\n";
    }
  }

  void handleUnregisterRobot(
      const std::shared_ptr<ns3_gazebo_interfaces::srv::UnregisterRobot::Request> req,
      std::shared_ptr<ns3_gazebo_interfaces::srv::UnregisterRobot::Response> res) {

    std::cout << "\n[ROS2 Service] Unregister request: " << req->model_name << "\n";

    // Find binding by model name
    gz::sim::Entity entity = gz::sim::kNullEntity;
    for (const auto& [ent, binding] : robotBindings) {
      if (binding.modelName == req->model_name) {
        entity = ent;
        break;
      }
    }

    if (entity == gz::sim::kNullEntity) {
      res->success = false;
      res->message = "Robot not registered: " + req->model_name;
      std::cerr << "ERROR: " << res->message << "\n";
      return;
    }

    // Remove binding
    uint32_t nodeId = robotBindings[entity].ns3NodeId;
    robotBindings.erase(entity);
    usedNodeIds.erase(nodeId);

    res->success = true;
    res->message = "Successfully unregistered from Node " + std::to_string(nodeId);
    std::cout << "✓ " << res->message << "\n\n";
  }

  void logRobotData(const RobotBinding& binding, double x, double y, double z) {
    auto elapsed = std::chrono::steady_clock::now() - startTime;
    double timestamp = std::chrono::duration<double>(elapsed).count();

    // Calculate distance to base station
    ns3::Ptr<ns3::MobilityModel> baseMobility =
        ns3_nodes.Get(baseStationNodeId)->GetObject<ns3::MobilityModel>();
    ns3::Vector basePos = baseMobility->GetPosition();

    double distance = std::sqrt(
        std::pow(x - basePos.x, 2) +
        std::pow(y - basePos.y, 2) +
        std::pow(z - basePos.z, 2));

    // Get signal quality
    double rssi = g_signalQuality[binding.ns3NodeId].hasData
                  ? g_signalQuality[binding.ns3NodeId].rssi : -100.0;
    double snr = g_signalQuality[binding.ns3NodeId].hasData
                 ? g_signalQuality[binding.ns3NodeId].snr : 0.0;

    // Get packet stats
    uint64_t sent = g_packetStats[binding.ns3NodeId].packetsSent;
    uint64_t received = g_packetStats[binding.ns3NodeId].packetsReceived;
    uint64_t lost = (sent > received) ? (sent - received) : 0;
    double lossRate = (sent > 0) ? (100.0 * lost / sent) : 0.0;

    logFile << timestamp << ","
            << binding.modelName << ","
            << binding.ns3NodeId << ","
            << x << "," << y << "," << z << ","
            << distance << ","
            << rssi << "," << snr << ","
            << sent << "," << received << ","
            << lost << "," << lossRate << "\n";
    logFile.flush();
  }

  void printNetworkStatus() {
    if (robotBindings.empty()) {
      return;  // No robots to display
    }

    std::cout << "\n=== NS-3 Network Status ===\n";
    std::cout << "Active robots: " << robotBindings.size() << "\n\n";

    for (const auto& [entity, binding] : robotBindings) {
      if (!binding.isActive) continue;

      // Get current position
      ns3::Ptr<ns3::Node> node = ns3_nodes.Get(binding.ns3NodeId);
      ns3::Ptr<ns3::MobilityModel> mobility =
          node->GetObject<ns3::MobilityModel>();
      ns3::Vector pos = mobility->GetPosition();

      // Calculate distance to base station
      ns3::Ptr<ns3::MobilityModel> baseMobility =
          ns3_nodes.Get(baseStationNodeId)->GetObject<ns3::MobilityModel>();
      double distance = mobility->GetDistanceFrom(baseMobility);

      // Get signal quality
      double rssi = g_signalQuality[binding.ns3NodeId].hasData
                    ? g_signalQuality[binding.ns3NodeId].rssi : -100.0;
      double snr = g_signalQuality[binding.ns3NodeId].hasData
                   ? g_signalQuality[binding.ns3NodeId].snr : 0.0;

      // Get packet stats
      uint64_t sent = g_packetStats[binding.ns3NodeId].packetsSent;
      uint64_t received = g_packetStats[binding.ns3NodeId].packetsReceived;
      // Calculate loss rate (handle broadcast case where received > sent)
      double lossRate = 0.0;
      if (sent > 0) {
        if (received <= sent) {
          lossRate = 100.0 * (sent - received) / static_cast<double>(sent);
        } else {
          // Broadcast amplification: more packets received than sent
          // This is normal when receiving broadcasts from multiple sources
          lossRate = 0.0;
        }
      }

      std::cout << "[" << binding.modelName << " (Node " << binding.ns3NodeId << ")]\n";
      std::cout << "  Position: (" << std::fixed << std::setprecision(2)
                << pos.x << ", " << pos.y << ", " << pos.z << ")\n";
      std::cout << "  Distance to base: " << distance << " m\n";
      std::cout << "  RSSI: " << rssi << " dBm, SNR: " << snr << " dB\n";
      std::cout << "  Packets: " << sent << " sent, " << received << " received, "
                << std::fixed << std::setprecision(1) << lossRate << "% loss\n\n";
    }
    std::cout << "===========================\n";
  }

  void publishNetworkStatus() {
    for (const auto& [entity, binding] : robotBindings) {
      if (!binding.isActive) continue;

      // Get current position
      ns3::Ptr<ns3::Node> node = ns3_nodes.Get(binding.ns3NodeId);
      ns3::Ptr<ns3::MobilityModel> mobility =
          node->GetObject<ns3::MobilityModel>();
      ns3::Vector pos = mobility->GetPosition();

      // Calculate distance to base station
      ns3::Ptr<ns3::MobilityModel> baseMobility =
          ns3_nodes.Get(baseStationNodeId)->GetObject<ns3::MobilityModel>();
      double distance = mobility->GetDistanceFrom(baseMobility);

      // Get signal quality
      double rssi = g_signalQuality[binding.ns3NodeId].hasData
                    ? g_signalQuality[binding.ns3NodeId].rssi : -100.0;
      double snr = g_signalQuality[binding.ns3NodeId].hasData
                   ? g_signalQuality[binding.ns3NodeId].snr : 0.0;

      // Get packet stats
      uint64_t sent = g_packetStats[binding.ns3NodeId].packetsSent;
      uint64_t received = g_packetStats[binding.ns3NodeId].packetsReceived;
      // Calculate loss rate (handle broadcast case where received > sent)
      double lossRate = 0.0;
      if (sent > 0) {
        if (received <= sent) {
          lossRate = 100.0 * (sent - received) / static_cast<double>(sent);
        } else {
          // Broadcast amplification: more packets received than sent
          // This is normal when receiving broadcasts from multiple sources
          lossRate = 0.0;
        }
      }

      // Publish ROS2 message
      auto msg = ns3_gazebo_interfaces::msg::NetworkStatus();
      msg.model_name = binding.modelName;
      msg.ns3_node_id = binding.ns3NodeId;
      msg.x = pos.x;
      msg.y = pos.y;
      msg.z = pos.z;
      msg.rssi = rssi;
      msg.snr = snr;
      msg.distance_to_base = distance;
      msg.packets_sent = sent;
      msg.packets_received = received;
      msg.packet_loss_rate = lossRate;

      networkStatusPub->publish(msg);
    }
  }
};

}  // namespace ns3_gazebo_world

// Register plugin
#include <gz/plugin/Register.hh>
GZ_ADD_PLUGIN(
    ns3_gazebo_world::NS3GazeboWorld,
    gz::sim::System,
    ns3_gazebo_world::NS3GazeboWorld::ISystemConfigure,
    ns3_gazebo_world::NS3GazeboWorld::ISystemPostUpdate,
    ns3_gazebo_world::NS3GazeboWorld::ISystemUpdate)
