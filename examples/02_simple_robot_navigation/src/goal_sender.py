#!/usr/bin/env python3

"""
Goal Sender Node for Simple Robot Navigation Example

This node provides automated goal sending functionality for testing
robot navigation. It can send predefined goal sequences or respond
to user input for manual goal setting.

Author: NS3-Gazebo Project
License: MIT License
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry

import json
import time
import math
from typing import List, Dict, Optional, Tuple


class GoalSender(Node):
    """
    Goal Sender Node

    Manages goal sending for robot navigation testing.
    Supports both automated sequences and manual goal input.
    """

    def __init__(self):
        super().__init__('goal_sender')

        # Parameters
        self.declare_parameter('auto_mode', True)
        self.declare_parameter('goal_timeout', 30.0)
        self.declare_parameter('goal_tolerance', 0.5)
        self.declare_parameter('sequence_delay', 5.0)
        self.declare_parameter('max_goals', 5)

        self.auto_mode = self.get_parameter('auto_mode').value
        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.sequence_delay = self.get_parameter('sequence_delay').value
        self.max_goals = self.get_parameter('max_goals').value

        # QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # State variables
        self.current_pose: Optional[PoseStamped] = None
        self.current_goal: Optional[PoseStamped] = None
        self.goal_index = 0
        self.goal_start_time = 0.0
        self.is_navigating = False
        self.sequence_active = False
        self.goals_completed = 0

        # Predefined goal sequence
        self.goal_sequence = [
            {'x': 2.0, 'y': 1.0, 'theta': 0.0, 'name': 'Point A'},
            {'x': 4.0, 'y': 3.0, 'theta': 1.57, 'name': 'Point B'},
            {'x': 1.0, 'y': 4.0, 'theta': 3.14, 'name': 'Point C'},
            {'x': -1.0, 'y': 2.0, 'theta': -1.57, 'name': 'Point D'},
            {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'name': 'Home'}
        ]

        # Publishers
        self.goal_pub = self.create_publisher(
            PoseStamped, 'goal_pose', self.reliable_qos)

        self.goal_status_pub = self.create_publisher(
            String, 'goal_status', self.reliable_qos)

        # Subscribers
        self.robot_status_sub = self.create_subscription(
            String, 'robot_status', self.robot_status_callback, self.reliable_qos)

        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odometry_callback, self.reliable_qos)

        self.manual_goal_sub = self.create_subscription(
            PoseStamped, 'manual_goal', self.manual_goal_callback, self.reliable_qos)

        self.sequence_control_sub = self.create_subscription(
            String, 'sequence_control', self.sequence_control_callback, self.reliable_qos)

        # Timers
        self.status_timer = self.create_timer(1.0, self.publish_goal_status)
        self.goal_timer = self.create_timer(0.5, self.check_goal_progress)

        if self.auto_mode:
            # Start automatic goal sequence after a delay
            self.sequence_timer = self.create_timer(5.0, self.start_goal_sequence)

        self.get_logger().info(f'Goal Sender initialized (auto_mode: {self.auto_mode})')

    def odometry_callback(self, msg: Odometry):
        """Process odometry data"""
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

    def robot_status_callback(self, msg: String):
        """Process robot status updates"""
        try:
            status = json.loads(msg.data)
            navigation = status.get('navigation', {})
            self.is_navigating = navigation.get('is_navigating', False)
            goal_reached = navigation.get('goal_reached', False)

            if goal_reached and self.current_goal:
                self.handle_goal_reached()

        except json.JSONDecodeError:
            self.get_logger().warn('Invalid robot status JSON')

    def manual_goal_callback(self, msg: PoseStamped):
        """Handle manually sent goals"""
        self.get_logger().info('Received manual goal, pausing automatic sequence')
        self.sequence_active = False
        self.send_goal(msg)

    def sequence_control_callback(self, msg: String):
        """Handle sequence control commands"""
        try:
            command = json.loads(msg.data)
            action = command.get('action', '')

            if action == 'start':
                self.start_goal_sequence()
            elif action == 'pause':
                self.sequence_active = False
                self.get_logger().info('Goal sequence paused')
            elif action == 'resume':
                self.sequence_active = True
                self.get_logger().info('Goal sequence resumed')
            elif action == 'reset':
                self.reset_sequence()
            elif action == 'next':
                self.send_next_goal()

        except json.JSONDecodeError:
            self.get_logger().warn('Invalid sequence control JSON')

    def start_goal_sequence(self):
        """Start the automatic goal sequence"""
        if self.auto_mode and not self.sequence_active:
            self.sequence_active = True
            self.goal_index = 0
            self.goals_completed = 0
            self.get_logger().info('Starting automatic goal sequence')
            self.send_next_goal()

        # Disable the one-time startup timer
        if hasattr(self, 'sequence_timer'):
            self.sequence_timer.cancel()

    def send_next_goal(self):
        """Send the next goal in the sequence"""
        if not self.sequence_active or self.goal_index >= len(self.goal_sequence):
            if self.goal_index >= len(self.goal_sequence):
                self.get_logger().info('Goal sequence completed!')
                self.sequence_active = False
            return

        goal_data = self.goal_sequence[self.goal_index]
        goal_msg = self.create_goal_message(
            goal_data['x'], goal_data['y'], goal_data['theta'])

        self.get_logger().info(
            f'Sending goal {self.goal_index + 1}/{len(self.goal_sequence)}: '
            f'{goal_data["name"]} at ({goal_data["x"]:.1f}, {goal_data["y"]:.1f})')

        self.send_goal(goal_msg)
        self.goal_index += 1

    def create_goal_message(self, x: float, y: float, theta: float) -> PoseStamped:
        """Create a goal message from coordinates"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Convert theta to quaternion
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = math.sin(theta / 2.0)
        goal.pose.orientation.w = math.cos(theta / 2.0)

        return goal

    def send_goal(self, goal: PoseStamped):
        """Send a goal to the robot"""
        self.current_goal = goal
        self.goal_start_time = time.time()
        self.goal_pub.publish(goal)

        self.get_logger().info(
            f'Goal sent: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')

    def handle_goal_reached(self):
        """Handle when a goal is reached"""
        if not self.current_goal:
            return

        elapsed_time = time.time() - self.goal_start_time
        self.goals_completed += 1

        self.get_logger().info(
            f'Goal reached in {elapsed_time:.1f} seconds! '
            f'Total goals completed: {self.goals_completed}')

        self.current_goal = None

        # Schedule next goal if in sequence mode
        if self.sequence_active and self.goal_index < len(self.goal_sequence):
            # Wait before sending next goal
            delay_timer = self.create_timer(
                self.sequence_delay, self.delayed_next_goal)

    def delayed_next_goal(self):
        """Send next goal after delay"""
        self.send_next_goal()
        # Cancel the single-use timer
        for timer in self._timers:
            if timer.callback == self.delayed_next_goal:
                timer.cancel()
                break

    def check_goal_progress(self):
        """Check progress toward current goal"""
        if not self.current_goal or not self.current_pose:
            return

        # Check for timeout
        elapsed_time = time.time() - self.goal_start_time
        if elapsed_time > self.goal_timeout:
            self.get_logger().warn(
                f'Goal timeout after {elapsed_time:.1f} seconds, sending next goal')
            self.handle_goal_timeout()

        # Check distance to goal
        distance = self.calculate_distance_to_goal()
        if distance < self.goal_tolerance and not self.is_navigating:
            self.get_logger().info(f'Close to goal ({distance:.2f}m), considering reached')
            self.handle_goal_reached()

    def calculate_distance_to_goal(self) -> float:
        """Calculate distance to current goal"""
        if not self.current_goal or not self.current_pose:
            return float('inf')

        dx = self.current_goal.pose.position.x - self.current_pose.pose.position.x
        dy = self.current_goal.pose.position.y - self.current_pose.pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def handle_goal_timeout(self):
        """Handle goal timeout"""
        self.get_logger().warn('Goal timed out, moving to next goal')
        self.current_goal = None

        if self.sequence_active:
            self.send_next_goal()

    def reset_sequence(self):
        """Reset the goal sequence"""
        self.sequence_active = False
        self.goal_index = 0
        self.goals_completed = 0
        self.current_goal = None
        self.get_logger().info('Goal sequence reset')

    def publish_goal_status(self):
        """Publish goal status information"""
        status = {
            'timestamp': time.time(),
            'auto_mode': self.auto_mode,
            'sequence_active': self.sequence_active,
            'goal_index': self.goal_index,
            'total_goals': len(self.goal_sequence),
            'goals_completed': self.goals_completed,
            'current_goal': {
                'x': self.current_goal.pose.position.x if self.current_goal else None,
                'y': self.current_goal.pose.position.y if self.current_goal else None,
                'distance': self.calculate_distance_to_goal() if self.current_goal else None,
                'elapsed_time': time.time() - self.goal_start_time if self.current_goal else 0
            },
            'sequence_progress': {
                'completed': self.goals_completed,
                'remaining': len(self.goal_sequence) - self.goal_index,
                'percentage': (self.goals_completed / len(self.goal_sequence)) * 100
                             if len(self.goal_sequence) > 0 else 0
            }
        }

        status_msg = String()
        status_msg.data = json.dumps(status)
        self.goal_status_pub.publish(status_msg)

    def create_custom_sequence(self, goals: List[Dict]):
        """Create a custom goal sequence"""
        self.goal_sequence = goals
        self.reset_sequence()
        self.get_logger().info(f'Custom sequence loaded with {len(goals)} goals')

    def add_waypoint(self, x: float, y: float, theta: float, name: str = "Waypoint"):
        """Add a waypoint to the current sequence"""
        waypoint = {'x': x, 'y': y, 'theta': theta, 'name': name}
        self.goal_sequence.append(waypoint)
        self.get_logger().info(f'Added waypoint: {name} at ({x:.1f}, {y:.1f})')

    def get_sequence_status(self) -> Dict:
        """Get detailed sequence status"""
        return {
            'active': self.sequence_active,
            'auto_mode': self.auto_mode,
            'current_index': self.goal_index,
            'total_goals': len(self.goal_sequence),
            'completed_goals': self.goals_completed,
            'remaining_goals': len(self.goal_sequence) - self.goal_index,
            'current_goal_distance': self.calculate_distance_to_goal(),
            'goal_timeout': self.goal_timeout,
            'sequence_delay': self.sequence_delay
        }


def main(args=None):
    """Main function"""
    rclpy.init(args=args)

    try:
        goal_sender = GoalSender()
        rclpy.spin(goal_sender)
    except KeyboardInterrupt:
        pass
    finally:
        if 'goal_sender' in locals():
            goal_sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()