#!/usr/bin/env python3
"""
Follower robot node that subscribes to leader robot's state via WiFi/NS-3
and matches the leader's x-direction velocity.

Network Flow:
  Leader (nns1) → /robot1/state → WiFi (10.0.0.1) → NS-3 WiFi Simulation
                                   ↓ (RSSI, packet loss)
  Follower (nns2) ← /robot1/state ← WiFi (10.0.0.4)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class FollowerRobot(Node):
    """
    ROS2 node that follows a leader robot by matching its x-direction velocity.
    Receives leader's state via WiFi network (NS-3 simulation).
    """

    def __init__(self):
        super().__init__('follower_robot')

        # Declare parameters
        self.declare_parameter('leader_state_topic', '/robot1/state')
        self.declare_parameter('follower_cmd_vel_topic', '/robot2/cmd_vel')
        self.declare_parameter('follower_odom_topic', '/robot2/odometry')
        self.declare_parameter('velocity_scale', 1.0)  # Scale factor for following
        self.declare_parameter('angular_gain', 0.0)  # Optional: match angular velocity too

        # Get parameters
        leader_state_topic = self.get_parameter('leader_state_topic').value
        follower_cmd_vel_topic = self.get_parameter('follower_cmd_vel_topic').value
        follower_odom_topic = self.get_parameter('follower_odom_topic').value
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.angular_gain = self.get_parameter('angular_gain').value

        # Subscriber to leader's state (via WiFi/NS-3)
        self.leader_state_sub = self.create_subscription(
            Odometry,
            leader_state_topic,
            self.leader_state_callback,
            10
        )

        # Subscriber to own odometry (from relay_bridge via Direct Network)
        self.own_odom_sub = self.create_subscription(
            Odometry,
            follower_odom_topic,
            self.own_odom_callback,
            10
        )

        # Publisher for follower's velocity commands
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            follower_cmd_vel_topic,
            10
        )

        # Store leader's velocity
        self.leader_linear_x = 0.0
        self.leader_angular_z = 0.0
        self.own_position = None

        self.get_logger().info(
            f'Follower Robot Node Started\n'
            f'  Subscribing to leader state: {leader_state_topic} (WiFi via NS-3)\n'
            f'  Subscribing to own odometry: {follower_odom_topic} (Direct Network)\n'
            f'  Publishing cmd_vel to: {follower_cmd_vel_topic} (Direct Network → Gazebo)\n'
            f'  Velocity scale: {self.velocity_scale}\n'
            f'  Angular gain: {self.angular_gain}'
        )

    def leader_state_callback(self, msg: Odometry):
        """
        Callback for leader's state messages (received via WiFi/NS-3).
        Extracts x-direction velocity and publishes it as follower's command.
        """
        # Extract linear velocity in x direction from leader's state
        # The velocity is in the robot's frame (twist.twist.linear.x)
        self.leader_linear_x = msg.twist.twist.linear.x
        self.leader_angular_z = msg.twist.twist.angular.z

        # Create velocity command for follower
        twist = Twist()
        twist.linear.x = self.leader_linear_x * self.velocity_scale

        # Optionally match angular velocity too (set angular_gain > 0)
        if self.angular_gain > 0.0:
            twist.angular.z = self.leader_angular_z * self.angular_gain

        # Publish command (via Direct Network → relay_bridge → Gazebo)
        self.cmd_vel_pub.publish(twist)

        # Log velocity (throttled)
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0

        self._log_counter += 1
        if self._log_counter % 10 == 0:  # Log every 10th message
            self.get_logger().info(
                f'[WiFi/NS-3] Leader state received: linear.x={self.leader_linear_x:.3f} m/s',
                throttle_duration_sec=1.0
            )
            self.get_logger().info(
                f'[Following] Commanding robot2: linear.x={twist.linear.x:.3f} m/s'
            )

    def own_odom_callback(self, msg: Odometry):
        """
        Callback for own odometry (from relay_bridge via Direct Network).
        Used for monitoring own position.
        """
        self.own_position = msg.pose.pose.position

        # Log own position (throttled)
        if not hasattr(self, '_own_log_counter'):
            self._own_log_counter = 0

        self._own_log_counter += 1
        if self._own_log_counter % 20 == 0:  # Log every 20th message
            self.get_logger().info(
                f'[Own State] Position: ({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f})',
                throttle_duration_sec=2.0
            )


def main(args=None):
    rclpy.init(args=args)
    node = FollowerRobot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the follower robot before exiting
        twist = Twist()
        node.cmd_vel_pub.publish(twist)
        node.get_logger().info('Follower Robot Node Stopped')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
