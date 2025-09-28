/*
 * NS-3 Hello World Network Example
 *
 * This example demonstrates basic NS-3 and Gazebo integration by creating
 * a simple two-node WiFi network where one node sends "Hello World"
 * messages to another node.
 *
 * Author: NS3-Gazebo Project
 * License: MIT License
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("HelloWorldNetwork");

/**
 * @brief Packet reception callback
 * @param packet Received packet
 */
void PacketReceived(Ptr<const Packet> packet, const Address& address)
{
    std::cout << "Time: " << Simulator::Now().GetSeconds()
              << "s - Packet received: " << packet->GetSize()
              << " bytes from " << address << std::endl;
}

/**
 * @brief Packet transmission callback
 * @param packet Transmitted packet
 */
void PacketTransmitted(Ptr<const Packet> packet)
{
    std::cout << "Time: " << Simulator::Now().GetSeconds()
              << "s - Packet transmitted: " << packet->GetSize()
              << " bytes" << std::endl;
}

/**
 * @brief Setup WiFi network configuration
 * @param nodes NodeContainer to configure
 * @return NetDeviceContainer with WiFi devices
 */
NetDeviceContainer SetupWiFiNetwork(NodeContainer nodes)
{
    // Create WiFi helper
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211g);

    // Setup channel model
    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());

    // Configure transmission power
    phy.Set("TxPowerStart", DoubleValue(20.0));  // 20 dBm
    phy.Set("TxPowerEnd", DoubleValue(20.0));

    // Setup MAC layer
    WifiMacHelper mac;
    Ssid ssid = Ssid("hello-world-network");

    // Configure as infrastructure network
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));

    // Install WiFi on all nodes
    NetDeviceContainer devices = wifi.Install(phy, mac, nodes);

    return devices;
}

/**
 * @brief Setup node positions
 * @param nodes NodeContainer to position
 */
void SetupNodePositions(NodeContainer nodes)
{
    MobilityHelper mobility;

    // Use list position allocator for precise positioning
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();

    // Node 0 at origin
    positionAlloc->Add(Vector(0.0, 0.0, 0.5));

    // Node 1 at 5 meters distance
    positionAlloc->Add(Vector(5.0, 0.0, 0.5));

    mobility.SetPositionAllocator(positionAlloc);

    // Static mobility (nodes don't move)
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);
}

/**
 * @brief Setup Internet stack and IP addresses
 * @param nodes NodeContainer to configure
 * @param devices NetDeviceContainer with network devices
 * @return Ipv4InterfaceContainer with assigned addresses
 */
Ipv4InterfaceContainer SetupInternetStack(NodeContainer nodes, NetDeviceContainer devices)
{
    // Install Internet stack
    InternetStackHelper internet;
    internet.Install(nodes);

    // Assign IP addresses
    Ipv4AddressHelper address;
    address.SetBase("10.1.1.0", "255.255.255.0");

    Ipv4InterfaceContainer interfaces = address.Assign(devices);

    return interfaces;
}

/**
 * @brief Setup applications for communication
 * @param nodes NodeContainer with nodes
 * @param interfaces Ipv4InterfaceContainer with network interfaces
 */
void SetupApplications(NodeContainer nodes, Ipv4InterfaceContainer interfaces)
{
    // Create UDP echo server on node 1
    UdpEchoServerHelper echoServer(9);
    ApplicationContainer serverApps = echoServer.Install(nodes.Get(1));
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(10.0));

    // Create UDP echo client on node 0
    UdpEchoClientHelper echoClient(interfaces.GetAddress(1), 9);
    echoClient.SetAttribute("MaxPackets", UintegerValue(10));
    echoClient.SetAttribute("Interval", TimeValue(Seconds(1.0)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));

    ApplicationContainer clientApps = echoClient.Install(nodes.Get(0));
    clientApps.Start(Seconds(2.0));
    clientApps.Stop(Seconds(10.0));

    // Setup packet tracing
    Config::ConnectWithoutContext("/NodeList/1/ApplicationList/0/$ns3::UdpEchoServer/Rx",
                                  MakeCallback(&PacketReceived));
    Config::ConnectWithoutContext("/NodeList/0/ApplicationList/0/$ns3::UdpEchoClient/Tx",
                                  MakeCallback(&PacketTransmitted));
}

/**
 * @brief Print simulation statistics
 * @param interfaces Ipv4InterfaceContainer for statistics
 */
void PrintStatistics(Ipv4InterfaceContainer interfaces)
{
    std::cout << "\n================================" << std::endl;
    std::cout << "   Simulation Statistics" << std::endl;
    std::cout << "================================" << std::endl;

    // Get statistics from interfaces
    Ptr<Ipv4> ipv4_0 = nodes.Get(0)->GetObject<Ipv4>();
    Ptr<Ipv4> ipv4_1 = nodes.Get(1)->GetObject<Ipv4>();

    if (ipv4_0 && ipv4_1) {
        std::cout << "Node 0 IP: " << interfaces.GetAddress(0) << std::endl;
        std::cout << "Node 1 IP: " << interfaces.GetAddress(1) << std::endl;
    }

    std::cout << "Simulation duration: 10 seconds" << std::endl;
    std::cout << "Expected packets: 10" << std::endl;
    std::cout << "WiFi standard: 802.11g" << std::endl;
    std::cout << "Distance: 5 meters" << std::endl;
    std::cout << "\nSimulation completed successfully!" << std::endl;
}

/**
 * @brief Main function
 */
int main(int argc, char *argv[])
{
    // Parse command line arguments
    CommandLine cmd;
    cmd.Parse(argc, argv);

    // Enable logging
    LogComponentEnable("UdpEchoClientApplication", LOG_LEVEL_INFO);
    LogComponentEnable("UdpEchoServerApplication", LOG_LEVEL_INFO);

    std::cout << "NS-3 Hello World Network Example" << std::endl;
    std::cout << "================================" << std::endl;

    // Create nodes
    std::cout << "Creating 2 WiFi nodes..." << std::endl;
    NodeContainer nodes;
    nodes.Create(2);

    // Setup WiFi network
    std::cout << "Setting up WiFi network..." << std::endl;
    NetDeviceContainer devices = SetupWiFiNetwork(nodes);

    // Setup node positions
    std::cout << "Positioning nodes..." << std::endl;
    SetupNodePositions(nodes);

    // Setup Internet stack
    std::cout << "Installing Internet stack..." << std::endl;
    Ipv4InterfaceContainer interfaces = SetupInternetStack(nodes, devices);

    // Setup applications
    std::cout << "Installing applications..." << std::endl;
    SetupApplications(nodes, interfaces);

    // Enable packet capture (optional)
    AsciiTraceHelper ascii;
    YansWifiPhyHelper::EnableAsciiAll(ascii.CreateFileStream("hello_world.tr"));
    YansWifiPhyHelper::EnablePcapAll("hello_world");

    std::cout << "Starting simulation..." << std::endl;
    std::cout << "\nPacket transmission log:" << std::endl;

    // Run simulation
    Simulator::Stop(Seconds(10.0));
    Simulator::Run();

    // Print statistics
    PrintStatistics(interfaces);

    // Cleanup
    Simulator::Destroy();

    return 0;
}