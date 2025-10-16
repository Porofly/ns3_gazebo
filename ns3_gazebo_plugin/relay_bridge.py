#!/usr/bin/env python3
"""
ROS2-Gazebo Bidirectional Relay Bridge

This bridge runs on the HOST and provides bidirectional communication:
1. ROS2 cmd_vel → Gazebo (robot control)
2. Gazebo odometry → ROS2 (robot state)

Network Architecture:
  nns1 (10.128.0.2) → ROS2 /robot1/cmd_vel → [Direct Network]
       ↓
  Host (10.128.0.1) → This Bridge → Gazebo Transport /model/robot1/cmd_vel
       ↓
  Gazebo Simulation → robot1 moves
       ↓
  Gazebo Transport /model/robot1/odometry → This Bridge
       ↓
  Host (10.128.0.1) → ROS2 /robot1/odometry → [Direct Network]
       ↓
  nns1 (10.128.0.2) ← receives odometry

Usage:
  1. Ensure direct network is configured:
     cd ../scripts && sudo python3 nns_setup.py setup -c 2 --include_direct

  2. Start Gazebo and spawn robots:
     gz sim ns3_gazebo_ros2.sdf
     ./spawn_two_robots.sh

  3. Run this bridge on HOST:
     python3 relay_bridge.py robot1 robot2

  4. Control from namespace:
     sudo ip netns exec nns1 bash -c "source /opt/ros/jazzy/setup.bash && \\
       ros2 topic pub /robot1/cmd_vel geometry_msgs/msg/Twist \\
       '{linear: {x: 0.5}, angular: {z: 0.0}}'"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3, TwistWithCovariance, PoseWithCovariance
from std_msgs.msg import Header
import subprocess
import sys
import threading
import json
import re


class ROS2GazeboRelayBridge(Node):
    """Bidirectional bridge between ROS2 and Gazebo Transport"""

    def __init__(self, robot_names):
        super().__init__('ros2_gazebo_relay_bridge')

        self.robot_names = robot_names
        self.cmd_vel_subscribers = {}
        self.odom_publishers = {}
        self.odom_threads = {}
        self.odom_processes = {}
        self.running = True

        # Create cmd_vel subscriber and odometry publisher for each robot
        for robot_name in robot_names:
            # Subscribe to ROS2 cmd_vel
            ros2_cmd_vel_topic = f'/{robot_name}/cmd_vel'
            subscriber = self.create_subscription(
                Twist,
                ros2_cmd_vel_topic,
                lambda msg, name=robot_name: self.cmd_vel_callback(msg, name),
                10
            )
            self.cmd_vel_subscribers[robot_name] = subscriber
            self.get_logger().info(f'[CMD_VEL] Subscribed to {ros2_cmd_vel_topic}')

            # Publish ROS2 odometry
            ros2_odom_topic = f'/{robot_name}/odometry'
            publisher = self.create_publisher(Odometry, ros2_odom_topic, 10)
            self.odom_publishers[robot_name] = publisher
            self.get_logger().info(f'[ODOMETRY] Publishing to {ros2_odom_topic}')

            # Start odometry relay thread
            thread = threading.Thread(
                target=self.gazebo_odom_relay_thread,
                args=(robot_name,),
                daemon=True
            )
            thread.start()
            self.odom_threads[robot_name] = thread
            self.get_logger().info(f'[ODOMETRY] Started Gazebo odometry relay for {robot_name}')

        self.get_logger().info('ROS2-Gazebo Bidirectional Relay Bridge started')
        self.get_logger().info(f'Relaying for robots: {", ".join(robot_names)}')
        self.get_logger().info('Direct Network: Listening on 10.128.0.x interfaces')

    def cmd_vel_callback(self, msg: Twist, robot_name: str):
        """
        Callback for ROS2 cmd_vel messages.
        Converts and forwards to Gazebo Transport.
        Uses non-blocking Popen to avoid timeout issues.
        """
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        # Gazebo topic name
        gz_topic = f'/model/{robot_name}/cmd_vel'

        # Construct Gazebo message
        gz_msg = f"linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}"

        # Send to Gazebo via gz topic command (non-blocking)
        try:
            cmd = [
                'gz', 'topic',
                '-t', gz_topic,
                '-m', 'gz.msgs.Twist',
                '-p', gz_msg
            ]

            # Use Popen for non-blocking execution
            # This prevents timeout issues and speeds up the callback
            proc = subprocess.Popen(
                cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True
            )

            # Log success (throttled)
            if not hasattr(self, f'_cmd_vel_log_counter_{robot_name}'):
                setattr(self, f'_cmd_vel_log_counter_{robot_name}', 0)

            counter = getattr(self, f'_cmd_vel_log_counter_{robot_name}')
            counter += 1
            setattr(self, f'_cmd_vel_log_counter_{robot_name}', counter)

            if counter % 10 == 0:  # Log every 10th command
                self.get_logger().info(
                    f'[CMD_VEL] {robot_name}: linear.x={linear_x:.2f}, angular.z={angular_z:.2f} → Gazebo',
                    throttle_duration_sec=1.0
                )

        except Exception as e:
            self.get_logger().error(
                f'[CMD_VEL] Error sending to Gazebo: {e}',
                throttle_duration_sec=5.0
            )

    def gazebo_odom_relay_thread(self, robot_name: str):
        """
        Thread function that subscribes to Gazebo odometry and publishes to ROS2.
        Uses 'gz topic' command to echo Gazebo Transport messages.
        """
        gz_odom_topic = f'/model/{robot_name}/odometry'

        self.get_logger().info(f'[ODOMETRY] {robot_name}: Starting Gazebo echo for {gz_odom_topic}')

        try:
            # Start gz topic echo command
            proc = subprocess.Popen(
                ['gz', 'topic', '-e', '-t', gz_odom_topic],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1
            )
            self.odom_processes[robot_name] = proc

            buffer = ""
            message_started = False

            # Read output line by line
            for line in iter(proc.stdout.readline, ''):
                if not self.running:
                    break

                line = line.strip()

                # Detect message boundary
                if line.startswith('header {') or line.startswith('pose {'):
                    message_started = True
                    buffer = line + "\n"
                elif message_started:
                    buffer += line + "\n"

                    # Check if message is complete (simple heuristic)
                    if line == '}' and buffer.count('{') == buffer.count('}'):
                        # Try to parse and publish
                        try:
                            odom_msg = self.parse_gazebo_odometry(buffer, robot_name)
                            if odom_msg:
                                self.odom_publishers[robot_name].publish(odom_msg)
                        except Exception as e:
                            self.get_logger().error(
                                f'[ODOMETRY] {robot_name}: Parse error: {e}',
                                throttle_duration_sec=5.0
                            )

                        # Reset buffer
                        buffer = ""
                        message_started = False

        except Exception as e:
            self.get_logger().error(f'[ODOMETRY] {robot_name}: Thread error: {e}')
        finally:
            if robot_name in self.odom_processes:
                self.odom_processes[robot_name].terminate()
                self.get_logger().info(f'[ODOMETRY] {robot_name}: Stopped')

    def parse_gazebo_odometry(self, gz_msg_text: str, robot_name: str):
        """
        Parse Gazebo odometry message text and convert to ROS2 Odometry message.
        This is a simplified parser for gz.msgs.Odometry format.
        """
        try:
            odom = Odometry()
            odom.header = Header()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = f'{robot_name}/base_link'

            # Improved parsing with more flexible regex patterns
            # Parse pose - position
            pos_match = re.search(
                r'position\s*\{\s*x:\s*([-\d.e+]+)\s+y:\s*([-\d.e+]+)\s+z:\s*([-\d.e+]+)',
                gz_msg_text,
                re.MULTILINE
            )
            if pos_match:
                odom.pose.pose.position.x = float(pos_match.group(1))
                odom.pose.pose.position.y = float(pos_match.group(2))
                odom.pose.pose.position.z = float(pos_match.group(3))

            # Parse pose - orientation (quaternion)
            orient_match = re.search(
                r'orientation\s*\{\s*x:\s*([-\d.e+]+)\s+y:\s*([-\d.e+]+)\s+z:\s*([-\d.e+]+)\s+w:\s*([-\d.e+]+)',
                gz_msg_text,
                re.MULTILINE
            )
            if orient_match:
                odom.pose.pose.orientation.x = float(orient_match.group(1))
                odom.pose.pose.orientation.y = float(orient_match.group(2))
                odom.pose.pose.orientation.z = float(orient_match.group(3))
                odom.pose.pose.orientation.w = float(orient_match.group(4))

            # Parse twist - linear velocity
            lin_vel_match = re.search(
                r'linear:\s*\{\s*x:\s*([-\d.e+]+)\s+y:\s*([-\d.e+]+)\s+z:\s*([-\d.e+]+)',
                gz_msg_text,
                re.MULTILINE
            )
            if lin_vel_match:
                odom.twist.twist.linear.x = float(lin_vel_match.group(1))
                odom.twist.twist.linear.y = float(lin_vel_match.group(2))
                odom.twist.twist.linear.z = float(lin_vel_match.group(3))

            # Parse twist - angular velocity
            ang_vel_match = re.search(
                r'angular:\s*\{\s*x:\s*([-\d.e+]+)\s+y:\s*([-\d.e+]+)\s+z:\s*([-\d.e+]+)',
                gz_msg_text,
                re.MULTILINE
            )
            if ang_vel_match:
                odom.twist.twist.angular.x = float(ang_vel_match.group(1))
                odom.twist.twist.angular.y = float(ang_vel_match.group(2))
                odom.twist.twist.angular.z = float(ang_vel_match.group(3))

            # Verify we got at least velocity data (most important for following)
            if lin_vel_match:
                return odom
            else:
                # No velocity data, skip this message
                return None

        except Exception as e:
            # Only log errors occasionally to avoid spam
            if not hasattr(self, f'_parse_error_counter_{robot_name}'):
                setattr(self, f'_parse_error_counter_{robot_name}', 0)

            counter = getattr(self, f'_parse_error_counter_{robot_name}')
            counter += 1
            setattr(self, f'_parse_error_counter_{robot_name}', counter)

            if counter % 50 == 0:  # Log every 50th error
                self.get_logger().error(
                    f'[ODOMETRY] {robot_name}: Parse exception (count: {counter}): {e}',
                    throttle_duration_sec=10.0
                )

            return None

    def shutdown(self):
        """Clean shutdown of all threads and processes"""
        self.running = False

        # Terminate all odometry processes
        for robot_name, proc in self.odom_processes.items():
            if proc and proc.poll() is None:
                proc.terminate()
                self.get_logger().info(f'[ODOMETRY] {robot_name}: Terminated gz topic process')


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

    bridge = None
    try:
        # Create and run the bridge
        bridge = ROS2GazeboRelayBridge(robot_names)

        print("\n" + "="*70)
        print("ROS2-Gazebo Bidirectional Relay Bridge Running")
        print("="*70)
        print(f"Robots: {', '.join(robot_names)}")
        print("\nBidirectional Communication:")
        print("  [CMD_VEL] Namespace ROS2 → Direct Network → Host → Gazebo")
        print("  [ODOMETRY] Gazebo → Host → Direct Network → Namespace ROS2")
        print("\nNetwork Details:")
        print("  Direct Network: 10.128.0.x (cmd_vel & odometry)")
        print("  WiFi Network: 10.0.0.x (robot-to-robot via NS-3)")
        print("\nPress Ctrl+C to stop")
        print("="*70 + "\n")

        rclpy.spin(bridge)

    except KeyboardInterrupt:
        print("\n\nShutting down relay bridge...")

    except Exception as e:
        print(f"\nError: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc()
        return 1

    finally:
        if bridge:
            bridge.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == '__main__':
    sys.exit(main())
