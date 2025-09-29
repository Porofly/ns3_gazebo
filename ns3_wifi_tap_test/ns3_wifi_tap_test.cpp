#include <getopt.h>
#include <cstdio>
#include <iostream> // std::cout
#include <iomanip> // std::setprecision

#include "ns3/core-module.h"
#include "ns3/node-container.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"
#include "ns3/random-variable-stream.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/wifi-mac-helper.h"
#include "ns3/csma-module.h" // for csma
#include "ns3/internet-module.h" // for IPv4 addressing

std::string mode = "infrastructure";

// parse user input
int get_options(int argc, char *argv[]) {

  // parse options
  int option_index; // not used
  while (1) {

    const struct option long_options[] = {
      // options
      {"help",                          no_argument, 0, 'h'},
      {"Help",                          no_argument, 0, 'H'},
      {"mode",                    required_argument, 0, 'm'},

      // end
      {0,0,0,0}
    };

    int ch = getopt_long(argc, argv, "hHm:", long_options, &option_index);

    if (ch == -1) {
      // no more arguments
      break;
    }
    if (ch == 0) {
      // command options set flags and use ch==0
      continue;
    }
    switch (ch) {
      case 'h': {	// help
        std::cout << "Usage: -h|-H|-m <infrastructure|adhoc|csma>\n"
                  << "default infrastructure";
        exit(0);
      }
      case 'H': {	// Help
        std::cout << "Usage: -h|-H|-m <infrastructure|adhoc|csma>\n"
                  << "default infrastructure";
        exit(0);
      }
      case 'm': {	// count
        mode = std::string(optarg);
        break;
      }
      default:
        exit(1);
    }
  }
  return 0;
}

// set fixt position for two nodes
void set_mobility(ns3::NodeContainer& nodes) {

  // all antenna locations start deterministically at 0, 0, 0
  ns3::Ptr<ns3::ListPositionAllocator>positionAlloc =
                         ns3::CreateObject<ns3::ListPositionAllocator>();
  positionAlloc->Add(ns3::Vector(0.0, 0.0, 0.0));
  positionAlloc->Add(ns3::Vector(1.0, 0.0, 0.0));

  ns3::MobilityHelper gs_mobility;
  gs_mobility.SetPositionAllocator(positionAlloc);
  gs_mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  gs_mobility.Install(nodes.Get(0));
  gs_mobility.Install(nodes.Get(1));
}

int main(int argc, char *argv[]) {

  // maybe change mode from infrastructure to adhoc or csma
  get_options(argc, argv);

  // Initialize NS-3 logging (important for NS-3 3.45)
  ns3::Time::SetResolution(ns3::Time::NS);

  // run ns3 real-time with checksums
  ns3::GlobalValue::Bind("SimulatorImplementationType",
                          ns3::StringValue("ns3::RealtimeSimulatorImpl"));
  ns3::GlobalValue::Bind("ChecksumEnabled", ns3::BooleanValue(true));

  // ns3 nodes
  ns3::NodeContainer nodes;
  nodes.Create(2);

  // ns3 Net devices
  ns3::NetDeviceContainer devices;

  // Wifi settings - Enhanced for NS-3 3.45 TAP bridge compatibility
  ns3::WifiHelper wifi;
  wifi.SetStandard(ns3::WIFI_STANDARD_80211b);
  wifi.SetRemoteStationManager ("ns3::AarfWifiManager");

  // Enable promiscuous mode for TAP bridge compatibility in NS-3 3.45
  // This allows the WiFi device to support SendFrom method
  ns3::Config::SetDefault("ns3::WifiNetDevice::Mtu", ns3::UintegerValue(1500));

  // wifi MAC
  ns3::WifiMacHelper wifiMac;


  // physical layer - Fixed for NS-3 3.45
  ns3::YansWifiChannelHelper wifiChannel = ns3::YansWifiChannelHelper::Default();
  ns3::YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel(wifiChannel.Create());

  // ssid for Infrastructure mode
  ns3::Ssid ssid = ns3::Ssid ("wifi-default");

  if(mode == "csma") {
    // CSMA works
    ns3::CsmaHelper csma;
    devices = csma.Install(nodes);

  } else if (mode == "adhoc") {
    // Ad hoc works
    set_mobility(nodes);
    wifiMac.SetType("ns3::AdhocWifiMac");
    devices = wifi.Install(wifiPhy, wifiMac, nodes);

  } else if (mode == "infrastructure") {

    set_mobility(nodes);

    // AP
    wifiMac.SetType ("ns3::ApWifiMac",
                     "Ssid", ns3::SsidValue (ssid));
    devices = wifi.Install (wifiPhy, wifiMac, nodes.Get(0));

    // Sta
    wifiMac.SetType ("ns3::StaWifiMac",
                     "Ssid", ns3::SsidValue (ssid),
                     "ActiveProbing", ns3::BooleanValue (false));
    ns3::NetDeviceContainer staDevices = wifi.Install (
                                      wifiPhy, wifiMac, nodes.Get(1));
    devices.Add (staDevices);


  } else {
    std::cerr << "Invalid mode: " << mode << "\n";
  }

  // Debug output
  std::cout << "Created " << devices.GetN() << " devices for " << nodes.GetN() << " nodes\n";

  // Install Internet Protocol Stack - Required for NS-3 3.45 TAP bridge
  ns3::InternetStackHelper internetStack;
  internetStack.Install(nodes);

  // Assign IP addresses
  ns3::Ipv4AddressHelper address;
  if (mode == "csma") {
    address.SetBase("10.1.1.0", "255.255.255.0");
  } else {
    address.SetBase("192.168.1.0", "255.255.255.0");
  }
  ns3::Ipv4InterfaceContainer interfaces = address.Assign(devices);

  // bind nodes to devices - Fixed for NS-3 3.45 WiFi TAP bridge compatibility
  if (devices.GetN() >= 2) {
    ns3::TapBridgeHelper tapBridge;

    if (mode == "csma") {
      // CSMA devices support UseBridge mode
      tapBridge.SetAttribute("Mode", ns3::StringValue("UseBridge"));
    } else {
      // WiFi devices should use UseLocal mode in NS-3 3.45
      tapBridge.SetAttribute("Mode", ns3::StringValue("UseLocal"));
      tapBridge.SetAttribute("IpAddress", ns3::StringValue("192.168.1.1"));
      tapBridge.SetAttribute("Netmask", ns3::StringValue("255.255.255.0"));
    }

    try {
      tapBridge.SetAttribute("DeviceName", ns3::StringValue("wifi_tap1"));
      tapBridge.Install(nodes.Get(0), devices.Get(0));
      std::cout << "Installed TAP bridge on node 0\n";

      // Reset attributes for second device
      tapBridge.SetAttribute("DeviceName", ns3::StringValue("wifi_tap2"));
      if (mode != "csma") {
        tapBridge.SetAttribute("IpAddress", ns3::StringValue("192.168.1.2"));
      }
      tapBridge.Install(nodes.Get(1), devices.Get(1));
      std::cout << "Installed TAP bridge on node 1\n";
    } catch (const std::exception& e) {
      std::cerr << "Error installing TAP bridge: " << e.what() << "\n";
      return 1;
    }
  } else {
    std::cerr << "Error: Not enough devices created (" << devices.GetN() << ")\n";
    return 1;
  }

  // set to run for a while - Changed to 30 seconds for testing
  ns3::Simulator::Stop(ns3::Seconds(30.0)); // 30 seconds for testing

  // run
  std::cout << "Starting ns-3 Wifi " << mode << " mode test.\n";
  ns3::Simulator::Run();
  ns3::Simulator::Destroy();
  std::cout << "Ending ns-3 Wifi mode test.\n";
  return 0;
}

