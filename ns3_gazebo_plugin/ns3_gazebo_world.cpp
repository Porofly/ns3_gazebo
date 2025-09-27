#include <cstdio>
#include <thread>
#include <iostream>

#include <gz/sim/Server.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components.hh>
#include <gz/math/Pose3.hh>
#include <sdf/Element.hh>

#include "ns3/core-module.h"
#include "ns3/node-container.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"

namespace ns3_gazebo_world {

static const int COUNT=5;

void ns3_setup(ns3::NodeContainer& ns3_nodes) {

  // run ns3 real-time with checksums
  ns3::GlobalValue::Bind("SimulatorImplementationType",
                          ns3::StringValue("ns3::RealtimeSimulatorImpl"));
  ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

  // Create ns3_nodes
  ns3_nodes.Create(COUNT);

  // physical layer - use default configuration to avoid ObjectFactory issues
  ns3::YansWifiChannelHelper wifiChannel = ns3::YansWifiChannelHelper::Default();
  ns3::YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(wifiChannel.Create());

  // Wifi settings
  ns3::WifiHelper wifi;
  wifi.SetStandard(ns3::WIFI_STANDARD_80211a);
  wifi.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                          "DataMode", ns3::StringValue("OfdmRate54Mbps"));

  // ad-hoc Wifi network
  ns3::WifiMacHelper wifiMac;
  wifiMac.SetType("ns3::AdhocWifiMac");

  // Install the wireless devices onto our ghost ns3_nodes.
  ns3::NetDeviceContainer devices = wifi.Install(wifiPhy, wifiMac, ns3_nodes);

  // antenna locations
  ns3::Ptr<ns3::ListPositionAllocator>positionAlloc =
                         ns3::CreateObject<ns3::ListPositionAllocator>();
  for (int i=0; i<COUNT; i++) {
    positionAlloc->Add(ns3::Vector(0.0, 0.0, 0.0));
  }
  ns3::MobilityHelper mobility;
  mobility.SetPositionAllocator(positionAlloc);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(ns3_nodes);

  // connect Wifi through TapBridge devices
  ns3::TapBridgeHelper tapBridge;
  tapBridge.SetAttribute("Mode", ns3::StringValue("UseLocal"));
  char buffer[10];
  for (int i=0; i<COUNT; i++) {
    sprintf(buffer, "wifi_tap%d", i+1);
    tapBridge.SetAttribute("DeviceName", ns3::StringValue(buffer));
    tapBridge.Install(ns3_nodes.Get(i), devices.Get(i));
  }
}

static void ns3_thread_function(void) {
  std::cout << "Starting ns-3 Wifi simulator in thread.\n";
  ns3::Simulator::Run();
  ns3::Simulator::Destroy();
  std::cout << "Ending ns-3 Wifi simulator in thread.\n";
}

class NS3GazeboWorld : public gz::sim::System,
                       public gz::sim::ISystemConfigure,
                       public gz::sim::ISystemUpdate {
  private:
  ns3::NodeContainer ns3_nodes;
  std::thread ns3_thread;
  gz::sim::Entity model_entity;

  public:
  NS3GazeboWorld() {
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

    // Find vehicle model
    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](const gz::sim::Entity &_entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name *_name) -> bool {
          if (_name->Data() == "vehicle") {
            model_entity = _entity;
            auto poseComp = _ecm.Component<gz::sim::components::Pose>(_entity);
            if (poseComp) {
              gz::math::Pose3d pose = poseComp->Data();
              float x = pose.Pos().X();
              std::cout << "Configure Point.x: " << x << "\n";
            }
          }
          return true;
        });

    // set up ns-3
    ns3_setup(ns3_nodes);

    // set to run for one year
    ns3::Simulator::Stop(ns3::Seconds(60*60*24*365.));

    // start the ns3 thread
    ns3_thread = std::thread(ns3_thread_function);
  }

  void Update(const gz::sim::UpdateInfo &_info,
              gz::sim::EntityComponentManager &_ecm) override {
    // Update logic here - get vehicle pose
    if (model_entity != gz::sim::kNullEntity) {
      auto poseComp = _ecm.Component<gz::sim::components::Pose>(model_entity);
      if (poseComp) {
        gz::math::Pose3d pose = poseComp->Data();
        float x = pose.Pos().X();
        // Uncomment for debugging: std::cout << "OnUpdate Point.x: " << x << "\n";
      }
    }
  }
};

} // namespace ns3_gazebo_world

#include <gz/plugin/Register.hh>

GZ_ADD_PLUGIN(ns3_gazebo_world::NS3GazeboWorld,
              gz::sim::System,
              ns3_gazebo_world::NS3GazeboWorld::ISystemConfigure,
              ns3_gazebo_world::NS3GazeboWorld::ISystemUpdate)