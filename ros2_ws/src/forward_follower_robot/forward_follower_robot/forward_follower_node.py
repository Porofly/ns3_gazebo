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
        self.declare_parameter('base_linear_x', 1.0)  # Constant forward velocity
        self.declare_parameter('angular_scale', 1.0)  # Scale for angular velocity
        self.declare_parameter('timeout_threshold', 1.0)  # Communication timeout in seconds

        # Get parameters
        leader_state_topic = self.get_parameter('leader_state_topic').value
        follower_cmd_vel_topic = self.get_parameter('follower_cmd_vel_topic').value
        self.base_linear_x = self.get_parameter('base_linear_x').value
        self.angular_scale = self.get_parameter('angular_scale').value
        self.timeout_threshold = self.get_parameter('timeout_threshold').value

        # Communication state
        self.leader_angular_z = 0.0
        self.current_linear_x = 0.0  # Will be base_linear_x when connected, 0 when timeout
        self.last_state_time = None
        self.communication_status = "NO_COMMUNICATION"

        # Subscriber to leader's state (via WiFi/NS-3)
        self.leader_state_sub = self.create_subscription(
            Odometry,
            leader_state_topic,
            self.leader_state_callback,
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

        self.get_logger().info(
            f'Forward Follower Robot Node Started\n'
            f'  Subscribing to: {leader_state_topic} (WiFi via NS-3)\n'
            f'  Publishing to: {follower_cmd_vel_topic} (Direct Network → Gazebo)\n'
            f'  Base forward velocity: {self.base_linear_x} m/s\n'
            f'  Angular velocity scale: {self.angular_scale}\n'
            f'  Timeout threshold: {self.timeout_threshold} seconds'
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

    def leader_state_callback(self, msg: Odometry):
        """
        Callback for leader's state messages (received via WiFi/NS-3).
        Extracts angular velocity (rotation) and updates communication timestamp.
        """
        # Extract angular velocity from leader's state
        self.leader_angular_z = msg.twist.twist.angular.z * self.angular_scale

        # Update communication timestamp
        self.last_state_time = time.time()
        self.communication_status = "CONNECTED"

        # Log leader state (throttled)
        if not hasattr(self, '_leader_log_counter'):
            self._leader_log_counter = 0

        self._leader_log_counter += 1
        if self._leader_log_counter % 20 == 0:  # Log every 20th message
            self.get_logger().info(
                f'[WiFi/NS-3] Leader state received: angular.z={msg.twist.twist.angular.z:.3f} rad/s',
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
        node.get_logger().info('Forward Follower Robot Node Stopped')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
