// Adapted from ns-3.29/src/tap-bridge/examples/tap-wifi-virtual-machine.cc
// Updated for NS-3 3.45 with WiFi MAC queue scheduler patch

#include <iostream>
#include <fstream>

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"

// network devices, do not exceed COUNT in nns_setup.py
static const int COUNT=5;

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("LxcNs3Wifi");

int
main (int argc, char *argv[])
{
  CommandLine cmd (__FILE__);
  cmd.Parse (argc, argv);

  //
  // We are interacting with the outside, real, world.  This means we have to
  // interact in real-time and therefore means we have to use the real-time
  // simulator and take the time to calculate checksums.
  //
  GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));
  GlobalValue::Bind ("ChecksumEnabled", BooleanValue (true));

  //
  // Create ghost nodes for network namespaces
  //
  NodeContainer nodes;
  nodes.Create (COUNT);

  //
  // We're going to use 802.11a so set up a wifi helper to reflect that.
  // NS-3 3.45: Patched wifi-mac-queue-scheduler-impl.h to support TapBridge
  //
  std::cout << "Using WiFi 802.11a network with NS-3 3.45 (patched for TapBridge)" << std::endl;

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211a);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate54Mbps"));

  //
  // No reason for pesky access points, so we'll use an ad-hoc network.
  //
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");

  //
  // Configure the physical layer.
  //
  YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper wifiPhy;
  wifiPhy.SetChannel (wifiChannel.Create ());

  //
  // Install the wireless devices onto our ghost nodes.
  //
  NetDeviceContainer devices = wifi.Install (wifiPhy, wifiMac, nodes);

  //
  // We need location information since we are talking about wifi, so add a
  // constant position to the ghost nodes.
  //
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  for (int i=0; i<COUNT; i++) {
    positionAlloc->Add (Vector (i * 5.0, 0.0, 0.0));  // 5 meters apart
  }
  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodes);

  //
  // Use the TapBridgeHelper to connect to the pre-configured tap devices.
  // We go with "UseLocal" mode since the wifi devices do not
  // support promiscuous mode (because of their nature).  This is a special
  // case mode that allows us to extend a linux bridge into ns-3 IFF we will
  // only see traffic from one other device on that bridge.  That is the case
  // for this configuration.
  //
  TapBridgeHelper tapBridge;
  tapBridge.SetAttribute ("Mode", StringValue ("UseLocal"));
  char buffer[16];
  for (int i=0; i<COUNT; i++) {
    sprintf(buffer, "wifi_tap%d", i+1);
    tapBridge.SetAttribute ("DeviceName", StringValue(buffer));
    try {
      tapBridge.Install (nodes.Get(i), devices.Get(i));
      std::cout << "TapBridge installed for " << buffer << " on WiFi device" << std::endl;
    } catch (const std::exception& e) {
      std::cerr << "TapBridge install failed for " << buffer << ": " << e.what() << std::endl;
    }
  }

  //
  // Run the simulation for a year, use CTRL-C to stop.
  //
  std::cout << "Starting NS-3 WiFi simulator for " << COUNT << " namespaces." << std::endl;
  std::cout << "WiFi characteristics: 802.11a, 54Mbps, range-based signal attenuation" << std::endl;
  Simulator::Stop (Seconds (60*60*24*365.));
  Simulator::Run ();
  Simulator::Destroy ();
  std::cout << "NS-3 WiFi simulator stopped." << std::endl;
  return 0;
}