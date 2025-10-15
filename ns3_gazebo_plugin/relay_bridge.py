#!/usr/bin/env python3
"""
ROS2-Gazebo Relay Bridge

This bridge runs on the HOST and relays robot control commands from
network namespaces (via ROS2 on direct network) to Gazebo simulation.

Network Architecture:
  nns1 (10.128.0.2) → ROS2 /robot1/cmd_vel → [Direct Network]
       ↓
  Host (10.128.0.1) → This Bridge → Gazebo Transport /model/robot1/cmd_vel
       ↓
  Gazebo Simulation → robot1 moves

Usage:
  1. Ensure direct network is configured:
     cd ../scripts && sudo python3 nns_setup.py setup -c 2 --include_direct

  2. Start Gazebo and spawn robots:
     gz sim ns3_gazebo_ros2.sdf
     ./spawn_two_robots.sh

  3. Run this bridge on HOST:
     python3 relay_bridge.py

  4. Control from namespace:
     sudo ip netns exec nns1 bash -c "source /opt/ros/jazzy/setup.bash && \\
       ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \\
       '{linear: {x: 0.5}, angular: {z: 0.0}}'"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import subprocess
import sys


class ROS2GazeboRelayBridge(Node):
    """Bridge that relays ROS2 cmd_vel topics to Gazebo Transport"""

    def __init__(self, robot_names):
        super().__init__('ros2_gazebo_relay_bridge')

        self.robot_names = robot_names
        self.subscribers = {}

        # Create a subscriber for each robot
        for robot_name in robot_names:
            ros2_topic = f'/{robot_name}/cmd_vel'
            subscriber = self.create_subscription(
                Twist,
                ros2_topic,
                lambda msg, name=robot_name: self.cmd_vel_callback(msg, name),
                10
            )
            self.subscribers[robot_name] = subscriber
            self.get_logger().info(f'Subscribed to {ros2_topic}')

        self.get_logger().info('ROS2-Gazebo Relay Bridge started')
        self.get_logger().info(f'Relaying commands for robots: {", ".join(robot_names)}')
        self.get_logger().info('Direct Network: Listening on 10.128.0.x interfaces')

    def cmd_vel_callback(self, msg: Twist, robot_name: str):
        """
        Callback for ROS2 cmd_vel messages.
        Converts and forwards to Gazebo Transport.
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Gazebo topic name
        gz_topic = f'/model/{robot_name}/cmd_vel'

        # Construct Gazebo message
        gz_msg = f"linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}"

        # Send to Gazebo via gz topic command
        try:
            cmd = [
                'gz', 'topic',
                '-t', gz_topic,
                '-m', 'gz.msgs.Twist',
                '-p', gz_msg
            ]

            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=1.0
            )

            if result.returncode == 0:
                self.get_logger().info(
                    f'{robot_name}: linear.x={linear_x:.2f}, angular.z={angular_z:.2f} → Gazebo',
                    throttle_duration_sec=1.0  # Log at most once per second
                )
            else:
                self.get_logger().error(
                    f'Failed to send to Gazebo: {result.stderr}',
                    throttle_duration_sec=5.0
                )

        except subprocess.TimeoutExpired:
            self.get_logger().error(
                f'Gazebo command timeout for {robot_name}',
                throttle_duration_sec=5.0
            )
        except Exception as e:
            self.get_logger().error(
                f'Error sending to Gazebo: {e}',
                throttle_duration_sec=5.0
            )


def main(args=None):
    """Main entry point"""

    # Default robot names
    default_robots = ['robot1', 'robot2']

    # Parse command line arguments
    if args is None:
        args = sys.argv[1:]

    robot_names = args if args else default_robots

    # Initialize ROS2
    rclpy.init()

    try:
        # Create and run the bridge
        bridge = ROS2GazeboRelayBridge(robot_names)

        print("\n" + "="*60)
        print("ROS2-Gazebo Relay Bridge Running")
        print("="*60)
        print(f"Robots: {', '.join(robot_names)}")
        print("\nNetwork Architecture:")
        print("  Namespace (10.128.0.x) → ROS2 → [Direct Network]")
        print("                              ↓")
        print("  Host (this bridge)      → Gazebo Transport")
        print("                              ↓")
        print("  Gazebo Simulation       → Robot movement")
        print("\nPress Ctrl+C to stop")
        print("="*60 + "\n")

        rclpy.spin(bridge)

    except KeyboardInterrupt:
        print("\n\nShutting down relay bridge...")

    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        return 1

    finally:
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
