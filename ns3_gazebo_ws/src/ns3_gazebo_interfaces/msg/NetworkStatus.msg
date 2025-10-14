# Network status message for a single robot
string model_name          # Gazebo model name
uint32 ns3_node_id         # NS-3 node ID

# Position
float64 x
float64 y
float64 z

# Network metrics
float64 rssi               # Received Signal Strength Indicator (dBm)
float64 snr                # Signal-to-Noise Ratio (dB)
float64 distance_to_base   # Distance to base station (m)

# Packet statistics
uint64 packets_sent
uint64 packets_received
float64 packet_loss_rate   # Percentage (0-100)
