#!/usr/bin/env python3
"""
Forward Follower Robot Node

Moves forward at 1 m/s and follows leader's angular velocity when communication is available
Stops completely (linear.x=0, angular.z=0) when communication is lost (timeout)

Purpose: Observe communication quality degradation with distance
- Close to leader: Moves forward and follows leader's rotation (communication OK)
- Far from leader: Stops completely (communication lost)

Note: Differential drive robots cannot move in y direction (no lateral movement)
      Instead, we follow angular.z (rotation) which is supported by diff drive.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math
import csv
import os


class ForwardFollowerRobot(Node):
    """
    Forward follower robot that maintains constant forward velocity
    and follows leader's angular velocity (rotation) when communication is available.
    """

    def __init__(self):
        super().__init__('forward_follower_robot')

        # Declare parameters
        self.declare_parameter('leader_state_topic', '/robot1/state')
        self.declare_parameter('follower_cmd_vel_topic', '/robot2/cmd_vel')
        self.declare_parameter('follower_odom_topic', '/robot2/odometry')
        self.declare_parameter('base_linear_x', 1.0)  # Constant forward velocity
        self.declare_parameter('angular_scale', 1.0)  # Scale for angular velocity
        self.declare_parameter('timeout_threshold', 1.0)  # Communication timeout in seconds

        # Statistics parameters
        self.declare_parameter('enable_statistics', True)
        self.declare_parameter('csv_output_file', '/tmp/follower_stats.csv')
        self.declare_parameter('statistics_rate', 1.0)  # Hz

        # Get parameters
        leader_state_topic = self.get_parameter('leader_state_topic').value
        follower_cmd_vel_topic = self.get_parameter('follower_cmd_vel_topic').value
        follower_odom_topic = self.get_parameter('follower_odom_topic').value
        self.base_linear_x = self.get_parameter('base_linear_x').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.timeout_threshold = self.get_parameter('timeout_threshold').value

        # Statistics parameters
        self.enable_statistics = self.get_parameter('enable_statistics').value
        self.csv_output_file = self.get_parameter('csv_output_file').value
        self.statistics_rate = self.get_parameter('statistics_rate').value

        # Communication state
        self.leader_angular_z = 0.0
        self.current_linear_x = 0.0  # Will be base_linear_x when connected, 0 when timeout
        self.last_state_time = None
        self.communication_status = "NO_COMMUNICATION"

        # Statistics data
        self.packet_count = 0
        self.leader_position = None  # From leader state message
        self.follower_position = None  # From follower odometry
        self.current_distance = None
        self.start_time = time.time()

        # CSV file handler
        self.csv_file = None
        self.csv_writer = None
        if self.enable_statistics:
            self._init_csv_file()

        # Subscriber to leader's state (via WiFi/NS-3)
        self.leader_state_sub = self.create_subscription(
            Odometry,
            leader_state_topic,
            self.leader_state_callback,
            10
        )

        # Subscriber to follower's odometry (via Direct Network)
        self.follower_odom_sub = self.create_subscription(
            Odometry,
            follower_odom_topic,
            self.follower_odom_callback,
            10
        )

        # Publisher for follower's velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            follower_cmd_vel_topic,
            10
        )

        # Timer to publish cmd_vel at 10 Hz
        self.cmd_vel_timer = self.create_timer(0.1, self.publish_cmd_vel)

        # Timer for statistics logging
        if self.enable_statistics:
            stats_period = 1.0 / self.statistics_rate
            self.statistics_timer = self.create_timer(stats_period, self.log_statistics)

        self.get_logger().info(
            f'Forward Follower Robot Node Started\n'
            f'  Subscribing to: {leader_state_topic} (WiFi via NS-3)\n'
            f'  Subscribing to: {follower_odom_topic} (Direct Network)\n'
            f'  Publishing to: {follower_cmd_vel_topic} (Direct Network → Gazebo)\n'
            f'  Base forward velocity: {self.base_linear_x} m/s\n'
            f'  Angular velocity scale: {self.angular_scale}\n'
            f'  Timeout threshold: {self.timeout_threshold} seconds\n'
            f'  Statistics enabled: {self.enable_statistics}\n'
            f'  CSV output file: {self.csv_output_file if self.enable_statistics else "N/A"}'
        )

        self.get_logger().info(
            '\n'
            '========================================\n'
            'Forward Follower Mode:\n'
            '  - Moves forward at 1 m/s (when connected)\n'
            '  - Follows leader\'s rotation (angular.z)\n'
            '  - STOPS COMPLETELY on timeout\n'
            '========================================\n'
        )

    def _init_csv_file(self):
        """Initialize CSV file for statistics logging"""
        try:
            # Create directory if it doesn't exist
            csv_dir = os.path.dirname(self.csv_output_file)
            if csv_dir and not os.path.exists(csv_dir):
                os.makedirs(csv_dir)

            self.csv_file = open(self.csv_output_file, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # Write header
            self.csv_writer.writerow([
                'timestamp', 'elapsed_time_s', 'distance_m', 'packet_count',
                'communication_status', 'linear_x', 'angular_z'
            ])
            self.csv_file.flush()
            self.get_logger().info(f'CSV statistics file created: {self.csv_output_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to create CSV file: {e}')
            self.enable_statistics = False

    def follower_odom_callback(self, msg: Odometry):
        """
        Callback for follower's odometry (received via Direct Network from Gazebo).
        Stores follower's current position for distance calculation.
        """
        self.follower_position = msg.pose.pose.position

    def leader_state_callback(self, msg: Odometry):
        """
        Callback for leader's state messages (received via WiFi/NS-3).
        Extracts angular velocity (rotation) and updates communication timestamp.
        """
        # Count packets received
        self.packet_count += 1

        # Extract angular velocity from leader's state
        self.leader_angular_z = msg.twist.twist.angular.z * self.angular_scale

        # Store leader position for distance calculation
        self.leader_position = msg.pose.pose.position

        # Update communication timestamp
        self.last_state_time = time.time()
        self.communication_status = "CONNECTED"

        # Log leader state (throttled)
        if not hasattr(self, '_leader_log_counter'):
            self._leader_log_counter = 0

        self._leader_log_counter += 1
        if self._leader_log_counter % 20 == 0:  # Log every 20th message
            self.get_logger().info(
                f'[WiFi/NS-3] Leader state received: angular.z={msg.twist.twist.angular.z:.3f} rad/s, '
                f'packet_count={self.packet_count}',
                throttle_duration_sec=2.0
            )

    def publish_cmd_vel(self):
        """
        Timer callback to publish cmd_vel at 10 Hz.
        Checks communication timeout and publishes velocity command.
        """
        # Check communication timeout
        current_time = time.time()
        if self.last_state_time is None:
            # Never received any message
            self.communication_status = "NO_COMMUNICATION"
            self.current_linear_x = 0.0
            self.leader_angular_z = 0.0
        elif (current_time - self.last_state_time) > self.timeout_threshold:
            # Communication timeout - STOP COMPLETELY
            self.communication_status = "TIMEOUT"
            self.current_linear_x = 0.0
            self.leader_angular_z = 0.0
        else:
            # Communication OK - move forward and follow rotation
            self.communication_status = "CONNECTED"
            self.current_linear_x = self.base_linear_x

        # Create velocity command
        twist = Twist()
        twist.linear.x = self.current_linear_x  # 1 m/s when connected, 0 when timeout
        twist.angular.z = self.leader_angular_z  # Follow leader's rotation or 0 on timeout

        # Publish command
        self.cmd_vel_pub.publish(twist)

        # Log velocity command (throttled)
        if not hasattr(self, '_cmd_log_counter'):
            self._cmd_log_counter = 0

        self._cmd_log_counter += 1
        if self._cmd_log_counter % 50 == 0:  # Log every 50th message (5 seconds at 10Hz)
            # Time since last message
            if self.last_state_time is not None:
                time_since_last = current_time - self.last_state_time
                timeout_info = f'{time_since_last:.2f}s ago'
            else:
                timeout_info = 'never'

            self.get_logger().info(
                f'[{self.communication_status}] cmd_vel: linear.x={twist.linear.x:.2f} (constant), '
                f'angular.z={twist.angular.z:.2f} (leader), last_msg={timeout_info}',
                throttle_duration_sec=5.0
            )

            # Log communication quality
            if self.communication_status == "TIMEOUT":
                self.get_logger().warn(
                    f'Communication timeout! No messages for {time_since_last:.2f}s. '
                    'Robot STOPPED completely. Leader may be too far.',
                    throttle_duration_sec=5.0
                )
            elif self.communication_status == "NO_COMMUNICATION":
                self.get_logger().warn(
                    'No communication established yet. Robot STOPPED. Waiting for leader state...',
                    throttle_duration_sec=5.0
                )

    def calculate_distance(self):
        """
        Calculate Euclidean distance between leader and follower robots.
        Returns distance in meters, or None if positions are not available.
        """
        if self.leader_position is None or self.follower_position is None:
            return None

        dx = self.leader_position.x - self.follower_position.x
        dy = self.leader_position.y - self.follower_position.y
        dz = self.leader_position.z - self.follower_position.z

        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        return distance

    def log_statistics(self):
        """
        Timer callback to log statistics at specified rate.
        Logs to console and CSV file if enabled.
        """
        # Calculate current distance
        self.current_distance = self.calculate_distance()

        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time

        # Prepare statistics data
        distance_str = f'{self.current_distance:.3f}' if self.current_distance is not None else 'N/A'

        # Log to console
        self.get_logger().info(
            f'[STATS] Time: {elapsed_time:.1f}s | Distance: {distance_str}m | '
            f'Packets: {self.packet_count} | Status: {self.communication_status} | '
            f'linear.x: {self.current_linear_x:.2f} | angular.z: {self.leader_angular_z:.2f}',
            throttle_duration_sec=0.5
        )

        # Write to CSV if enabled
        if self.enable_statistics and self.csv_writer is not None:
            try:
                self.csv_writer.writerow([
                    time.time(),  # timestamp
                    elapsed_time,  # elapsed_time_s
                    self.current_distance if self.current_distance is not None else '',  # distance_m
                    self.packet_count,  # packet_count
                    self.communication_status,  # communication_status
                    self.current_linear_x,  # linear_x
                    self.leader_angular_z  # angular_z
                ])
                self.csv_file.flush()
            except Exception as e:
                self.get_logger().error(f'Failed to write to CSV: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ForwardFollowerRobot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the follower robot before exiting
        twist = Twist()
        node.cmd_vel_pub.publish(twist)

        # Close CSV file if open
        if node.csv_file is not None:
            try:
                node.csv_file.close()
                node.get_logger().info(f'CSV statistics file closed: {node.csv_output_file}')
            except Exception as e:
                node.get_logger().error(f'Failed to close CSV file: {e}')

        node.get_logger().info('Forward Follower Robot Node Stopped')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
