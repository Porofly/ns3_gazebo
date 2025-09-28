#!/usr/bin/env python3

"""
Robot Controller Node for Simple Robot Navigation Example

This node provides basic robot control functionality including:
- Goal navigation interface
- Status monitoring
- Safety checks
- Network communication

Author: NS3-Gazebo Project
License: MIT License
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

import json
import math
import time
from typing import Optional, Tuple


class RobotController(Node):
    """
    Robot Controller Node

    Manages robot navigation, safety, and network communication.
    """

    def __init__(self):
        super().__init__('robot_controller')

        # Parameters
        self.declare_parameter('max_linear_velocity', 1.0)
        self.declare_parameter('max_angular_velocity', 2.0)
        self.declare_parameter('safety_distance', 0.5)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('network_update_rate', 2.0)

        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.network_rate = self.get_parameter('network_update_rate').value

        # QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # State variables
        self.current_pose: Optional[PoseStamped] = None
        self.current_velocity: Optional[Twist] = None
        self.laser_data: Optional[LaserScan] = None
        self.is_navigating = False
        self.emergency_stop = False
        self.goal_reached = False

        # Navigation action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', self.reliable_qos)

        self.robot_status_pub = self.create_publisher(
            String, 'robot_status', self.reliable_qos)

        self.network_data_pub = self.create_publisher(
            String, 'network_data', self.reliable_qos)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odometry_callback, self.sensor_qos)

        self.laser_sub = self.create_subscription(
            LaserScan, 'scan', self.laser_callback, self.sensor_qos)

        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback, self.reliable_qos)

        self.emergency_sub = self.create_subscription(
            Bool, 'emergency_stop', self.emergency_callback, self.reliable_qos)

        # Timers
        self.status_timer = self.create_timer(0.5, self.publish_status)
        self.network_timer = self.create_timer(
            1.0 / self.network_rate, self.publish_network_data)
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info('Robot Controller initialized')

    def odometry_callback(self, msg: Odometry):
        """Process odometry data"""
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def laser_callback(self, msg: LaserScan):
        """Process laser scan data"""
        self.laser_data = msg

    def goal_callback(self, msg: PoseStamped):
        """Handle new goal pose"""
        self.get_logger().info(f'Received new goal: x={msg.pose.position.x:.2f}, '
                              f'y={msg.pose.position.y:.2f}')
        self.navigate_to_goal(msg)

    def emergency_callback(self, msg: Bool):
        """Handle emergency stop signal"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn('Emergency stop activated!')
            self.stop_robot()
        else:
            self.get_logger().info('Emergency stop deactivated')

    def navigate_to_goal(self, goal: PoseStamped):
        """Navigate to specified goal using Nav2"""
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation server not available')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self.is_navigating = True
        self.goal_reached = False

        future = self.nav_client.send_goal_async(
            goal_msg, feedback_callback=self.navigation_feedback)

        future.add_done_callback(self.navigation_response)

    def navigation_response(self, future):
        """Handle navigation action response"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by navigation server')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted by navigation server')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.navigation_result)

    def navigation_feedback(self, feedback_msg):
        """Handle navigation feedback"""
        # Log progress or update status
        distance_remaining = feedback_msg.feedback.distance_remaining
        if distance_remaining < self.goal_tolerance:
            self.get_logger().info(f'Close to goal: {distance_remaining:.2f}m remaining')

    def navigation_result(self, future):
        """Handle navigation result"""
        result = future.result().result
        self.is_navigating = False

        if result:
            self.get_logger().info('Goal reached successfully!')
            self.goal_reached = True
        else:
            self.get_logger().warn('Failed to reach goal')
            self.goal_reached = False

    def safety_check(self):
        """Perform safety checks and emergency stop if needed"""
        if not self.laser_data or self.emergency_stop:
            return

        # Check for obstacles in front
        min_distance = float('inf')
        for i, range_val in enumerate(self.laser_data.ranges):
            if math.isfinite(range_val):
                min_distance = min(min_distance, range_val)

        if min_distance < self.safety_distance:
            self.get_logger().warn(f'Obstacle detected at {min_distance:.2f}m')
            # Navigation system should handle this, but we can add extra safety

    def stop_robot(self):
        """Immediately stop the robot"""
        stop_msg = Twist()
        self.cmd_vel_pub.publish(stop_msg)

    def publish_status(self):
        """Publish robot status information"""
        if not self.current_pose:
            return

        status = {
            'timestamp': time.time(),
            'position': {
                'x': self.current_pose.pose.position.x,
                'y': self.current_pose.pose.position.y,
                'z': self.current_pose.pose.position.z
            },
            'orientation': {
                'x': self.current_pose.pose.orientation.x,
                'y': self.current_pose.pose.orientation.y,
                'z': self.current_pose.pose.orientation.z,
                'w': self.current_pose.pose.orientation.w
            },
            'velocity': {
                'linear': {
                    'x': self.current_velocity.linear.x if self.current_velocity else 0.0,
                    'y': self.current_velocity.linear.y if self.current_velocity else 0.0,
                    'z': self.current_velocity.linear.z if self.current_velocity else 0.0
                },
                'angular': {
                    'x': self.current_velocity.angular.x if self.current_velocity else 0.0,
                    'y': self.current_velocity.angular.y if self.current_velocity else 0.0,
                    'z': self.current_velocity.angular.z if self.current_velocity else 0.0
                }
            },
            'navigation': {
                'is_navigating': self.is_navigating,
                'goal_reached': self.goal_reached,
                'emergency_stop': self.emergency_stop
            }
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.robot_status_pub.publish(status_msg)

    def publish_network_data(self):
        """Publish data for network simulation"""
        if not self.current_pose:
            return

        # Create network payload (simulating sensor data transmission)
        network_data = {
            'timestamp': time.time(),
            'robot_id': 'diff_drive_robot',
            'message_type': 'status_update',
            'payload_size': 1024,  # bytes
            'data': {
                'pose': {
                    'x': self.current_pose.pose.position.x,
                    'y': self.current_pose.pose.position.y,
                    'theta': self.get_yaw_from_quaternion(
                        self.current_pose.pose.orientation)
                },
                'status': 'navigating' if self.is_navigating else 'idle',
                'battery': 85.5,  # Simulated battery level
                'signal_strength': -45.2  # Simulated WiFi signal strength (dBm)
            }
        }

        network_msg = String()
        network_msg.data = json.dumps(network_data)
        self.network_data_pub.publish(network_msg)

    def get_yaw_from_quaternion(self, quaternion) -> float:
        """Convert quaternion to yaw angle"""
        import tf_transformations
        euler = tf_transformations.euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ])
        return euler[2]  # yaw

    def get_distance_to_goal(self, goal: PoseStamped) -> float:
        """Calculate distance to goal"""
        if not self.current_pose:
            return float('inf')

        dx = goal.pose.position.x - self.current_pose.pose.position.x
        dy = goal.pose.position.y - self.current_pose.pose.position.y
        return math.sqrt(dx*dx + dy*dy)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    try:
        robot_controller = RobotController()
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'robot_controller' in locals():
            robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()