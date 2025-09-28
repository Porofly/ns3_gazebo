#!/usr/bin/env python3

"""
Network Monitor Node for Simple Robot Navigation Example

This node monitors network performance including:
- Latency measurements
- Throughput analysis
- Packet loss detection
- Signal strength monitoring
- NS-3 integration

Author: NS3-Gazebo Project
License: MIT License
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String, Float64
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import json
import time
import math
import threading
import socket
import subprocess
from typing import Dict, List, Optional
from collections import deque


class NetworkMonitor(Node):
    """
    Network Monitor Node

    Monitors network performance and provides real-time statistics
    for the robot navigation system.
    """

    def __init__(self):
        super().__init__('network_monitor')

        # Parameters
        self.declare_parameter('enable_network', True)
        self.declare_parameter('monitoring_rate', 1.0)
        self.declare_parameter('ping_target', '10.1.1.2')
        self.declare_parameter('interface', 'wlan0')
        self.declare_parameter('history_size', 100)

        self.enable_network = self.get_parameter('enable_network').value
        self.monitoring_rate = self.get_parameter('monitoring_rate').value
        self.ping_target = self.get_parameter('ping_target').value
        self.interface = self.get_parameter('interface').value
        self.history_size = self.get_parameter('history_size').value

        # QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Network statistics storage
        self.latency_history = deque(maxlen=self.history_size)
        self.throughput_history = deque(maxlen=self.history_size)
        self.packet_loss_history = deque(maxlen=self.history_size)
        self.signal_strength_history = deque(maxlen=self.history_size)

        # Network metrics
        self.current_latency = 0.0
        self.current_throughput = 0.0
        self.current_packet_loss = 0.0
        self.current_signal_strength = 0.0
        self.total_bytes_sent = 0
        self.total_bytes_received = 0
        self.total_packets_sent = 0
        self.total_packets_received = 0

        # Timing
        self.last_measurement_time = time.time()
        self.measurement_lock = threading.Lock()

        # Publishers
        self.network_status_pub = self.create_publisher(
            String, 'network_status', self.reliable_qos)

        self.latency_pub = self.create_publisher(
            Float64, 'network/latency', self.reliable_qos)

        self.throughput_pub = self.create_publisher(
            Float64, 'network/throughput', self.reliable_qos)

        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, 'diagnostics', self.reliable_qos)

        # Subscribers
        self.robot_status_sub = self.create_subscription(
            String, 'robot_status', self.robot_status_callback, self.reliable_qos)

        self.network_data_sub = self.create_subscription(
            String, 'network_data', self.network_data_callback, self.reliable_qos)

        # Timers
        if self.enable_network:
            self.monitor_timer = self.create_timer(
                1.0 / self.monitoring_rate, self.monitor_network)
            self.publish_timer = self.create_timer(1.0, self.publish_network_status)
            self.diagnostics_timer = self.create_timer(2.0, self.publish_diagnostics)

        self.get_logger().info(f'Network Monitor initialized (enabled: {self.enable_network})')

    def robot_status_callback(self, msg: String):
        """Process robot status updates"""
        try:
            status = json.loads(msg.data)
            # Update network statistics based on robot status
            self.update_network_load(status)
        except json.JSONDecodeError:
            self.get_logger().warn('Invalid robot status JSON')

    def network_data_callback(self, msg: String):
        """Process network data transmissions"""
        try:
            data = json.loads(msg.data)
            payload_size = data.get('payload_size', 0)

            with self.measurement_lock:
                self.total_bytes_sent += payload_size
                self.total_packets_sent += 1

            # Simulate network delay and processing
            self.simulate_network_transmission(data)

        except json.JSONDecodeError:
            self.get_logger().warn('Invalid network data JSON')

    def simulate_network_transmission(self, data: Dict):
        """Simulate network transmission with NS-3 integration"""
        # This would interface with NS-3 simulation
        # For now, simulate basic network behavior

        payload_size = data.get('payload_size', 0)

        # Simulate transmission delay based on payload size
        base_delay = 0.001  # 1ms base delay
        size_delay = payload_size / 1000000  # 1µs per byte (1 Mbps)
        total_delay = base_delay + size_delay

        # Add some random variation
        import random
        jitter = random.uniform(-0.0005, 0.0005)  # ±0.5ms jitter
        actual_delay = max(0, total_delay + jitter)

        # Update latency statistics
        with self.measurement_lock:
            self.latency_history.append(actual_delay * 1000)  # Convert to ms
            self.current_latency = actual_delay * 1000

    def monitor_network(self):
        """Monitor network performance metrics"""
        if not self.enable_network:
            return

        # Measure ping latency
        self.measure_ping_latency()

        # Measure throughput
        self.measure_throughput()

        # Monitor signal strength
        self.monitor_signal_strength()

        # Calculate packet loss
        self.calculate_packet_loss()

    def measure_ping_latency(self):
        """Measure network latency using ping"""
        try:
            # Simple ping measurement
            result = subprocess.run(
                ['ping', '-c', '1', '-W', '1', self.ping_target],
                capture_output=True, text=True, timeout=2
            )

            if result.returncode == 0:
                # Parse ping output for latency
                output_lines = result.stdout.split('\n')
                for line in output_lines:
                    if 'time=' in line:
                        time_part = line.split('time=')[1].split(' ')[0]
                        latency = float(time_part)

                        with self.measurement_lock:
                            self.latency_history.append(latency)
                            self.current_latency = latency
                        break
            else:
                # Ping failed, record high latency
                with self.measurement_lock:
                    self.latency_history.append(1000.0)  # 1 second timeout
                    self.current_latency = 1000.0

        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, ValueError):
            with self.measurement_lock:
                self.latency_history.append(1000.0)
                self.current_latency = 1000.0

    def measure_throughput(self):
        """Calculate network throughput"""
        current_time = time.time()

        with self.measurement_lock:
            time_diff = current_time - self.last_measurement_time

            if time_diff > 0:
                # Calculate throughput in bits per second
                bytes_diff = self.total_bytes_sent - getattr(self, 'last_bytes_sent', 0)
                throughput_bps = (bytes_diff * 8) / time_diff
                throughput_mbps = throughput_bps / 1000000  # Convert to Mbps

                self.throughput_history.append(throughput_mbps)
                self.current_throughput = throughput_mbps
                self.last_bytes_sent = self.total_bytes_sent

            self.last_measurement_time = current_time

    def monitor_signal_strength(self):
        """Monitor WiFi signal strength"""
        try:
            # Try to get signal strength from iwconfig or similar
            result = subprocess.run(
                ['iwconfig', self.interface],
                capture_output=True, text=True, timeout=2
            )

            if result.returncode == 0:
                # Parse signal strength from iwconfig output
                output = result.stdout
                if 'Signal level=' in output:
                    signal_part = output.split('Signal level=')[1].split(' ')[0]
                    signal_strength = float(signal_part)

                    with self.measurement_lock:
                        self.signal_strength_history.append(signal_strength)
                        self.current_signal_strength = signal_strength
                else:
                    # Simulate signal strength if not available
                    self.simulate_signal_strength()
            else:
                self.simulate_signal_strength()

        except (subprocess.TimeoutExpired, subprocess.CalledProcessError, ValueError):
            self.simulate_signal_strength()

    def simulate_signal_strength(self):
        """Simulate WiFi signal strength based on distance"""
        # This would integrate with robot position and NS-3 propagation model
        # For now, simulate based on simple distance model

        import random
        base_strength = -45.0  # dBm at 1 meter
        random_variation = random.uniform(-5.0, 5.0)
        simulated_strength = base_strength + random_variation

        with self.measurement_lock:
            self.signal_strength_history.append(simulated_strength)
            self.current_signal_strength = simulated_strength

    def calculate_packet_loss(self):
        """Calculate packet loss percentage"""
        with self.measurement_lock:
            if self.total_packets_sent > 0:
                # Simulate some packet loss based on signal strength
                if self.current_signal_strength < -70:
                    loss_rate = 0.05  # 5% loss for weak signal
                elif self.current_signal_strength < -60:
                    loss_rate = 0.02  # 2% loss for medium signal
                else:
                    loss_rate = 0.001  # 0.1% loss for strong signal

                import random
                if random.random() < loss_rate:
                    self.total_packets_received = max(0, self.total_packets_sent - 1)
                else:
                    self.total_packets_received = self.total_packets_sent

                loss_percentage = ((self.total_packets_sent - self.total_packets_received)
                                 / self.total_packets_sent) * 100
                self.packet_loss_history.append(loss_percentage)
                self.current_packet_loss = loss_percentage

    def update_network_load(self, robot_status: Dict):
        """Update network load based on robot activity"""
        # Adjust network performance based on robot navigation state
        navigation = robot_status.get('navigation', {})
        is_navigating = navigation.get('is_navigating', False)

        if is_navigating:
            # Higher network load during navigation
            self.total_bytes_sent += 500  # Additional sensor data
        else:
            # Lower network load when idle
            self.total_bytes_sent += 100  # Basic status updates

    def get_statistics(self) -> Dict:
        """Get current network statistics"""
        with self.measurement_lock:
            return {
                'latency': {
                    'current': self.current_latency,
                    'average': sum(self.latency_history) / len(self.latency_history)
                              if self.latency_history else 0,
                    'min': min(self.latency_history) if self.latency_history else 0,
                    'max': max(self.latency_history) if self.latency_history else 0
                },
                'throughput': {
                    'current': self.current_throughput,
                    'average': sum(self.throughput_history) / len(self.throughput_history)
                              if self.throughput_history else 0
                },
                'packet_loss': {
                    'current': self.current_packet_loss,
                    'average': sum(self.packet_loss_history) / len(self.packet_loss_history)
                              if self.packet_loss_history else 0
                },
                'signal_strength': {
                    'current': self.current_signal_strength,
                    'average': sum(self.signal_strength_history) / len(self.signal_strength_history)
                              if self.signal_strength_history else 0
                },
                'totals': {
                    'bytes_sent': self.total_bytes_sent,
                    'bytes_received': self.total_bytes_received,
                    'packets_sent': self.total_packets_sent,
                    'packets_received': self.total_packets_received
                }
            }

    def publish_network_status(self):
        """Publish network status information"""
        stats = self.get_statistics()

        status_msg = String()
        status_msg.data = json.dumps({
            'timestamp': time.time(),
            'network_enabled': self.enable_network,
            'statistics': stats
        })
        self.network_status_pub.publish(status_msg)

        # Publish individual metrics
        latency_msg = Float64()
        latency_msg.data = self.current_latency
        self.latency_pub.publish(latency_msg)

        throughput_msg = Float64()
        throughput_msg.data = self.current_throughput
        self.throughput_pub.publish(throughput_msg)

    def publish_diagnostics(self):
        """Publish network diagnostics"""
        stats = self.get_statistics()

        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        # Network status diagnostic
        network_status = DiagnosticStatus()
        network_status.name = 'Network Status'
        network_status.hardware_id = 'network_monitor'

        # Determine overall network health
        latency = stats['latency']['current']
        packet_loss = stats['packet_loss']['current']
        signal_strength = stats['signal_strength']['current']

        if latency > 100 or packet_loss > 5 or signal_strength < -80:
            network_status.level = DiagnosticStatus.ERROR
            network_status.message = 'Poor network performance'
        elif latency > 50 or packet_loss > 2 or signal_strength < -65:
            network_status.level = DiagnosticStatus.WARN
            network_status.message = 'Degraded network performance'
        else:
            network_status.level = DiagnosticStatus.OK
            network_status.message = 'Network operating normally'

        # Add detailed values
        network_status.values = [
            KeyValue(key='Latency (ms)', value=f'{latency:.2f}'),
            KeyValue(key='Throughput (Mbps)', value=f'{stats["throughput"]["current"]:.3f}'),
            KeyValue(key='Packet Loss (%)', value=f'{packet_loss:.2f}'),
            KeyValue(key='Signal Strength (dBm)', value=f'{signal_strength:.1f}'),
            KeyValue(key='Bytes Sent', value=str(self.total_bytes_sent)),
            KeyValue(key='Packets Sent', value=str(self.total_packets_sent))
        ]

        diag_array.status.append(network_status)
        self.diagnostics_pub.publish(diag_array)

    def log_network_summary(self):
        """Log network performance summary"""
        stats = self.get_statistics()
        self.get_logger().info(
            f'Network Stats - Latency: {stats["latency"]["current"]:.1f}ms, '
            f'Throughput: {stats["throughput"]["current"]:.2f}Mbps, '
            f'Loss: {stats["packet_loss"]["current"]:.1f}%, '
            f'Signal: {stats["signal_strength"]["current"]:.1f}dBm'
        )


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    try:
        network_monitor = NetworkMonitor()
        rclpy.spin(network_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        if 'network_monitor' in locals():
            network_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()