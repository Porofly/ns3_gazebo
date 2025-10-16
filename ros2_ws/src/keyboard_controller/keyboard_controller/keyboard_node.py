#!/usr/bin/env python3
"""
Keyboard teleoperation node for robot control.
Publishes velocity commands to /robot1/cmd_vel based on keyboard input.
Also subscribes to /robot1/odometry and republishes for follower robots.
"""

import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class KeyboardController(Node):
    """
    ROS2 node that reads keyboard input and publishes velocity commands.
    """

    def __init__(self):
        super().__init__('keyboard_controller')

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)

        # Publisher for robot state (for other robots to follow via WiFi/NS-3)
        self.state_pub = self.create_publisher(Odometry, '/robot1/state', 10)

        # Velocity parameters
        self.linear_speed = 1.0  # m/s
        self.angular_speed = 1.0  # rad/s
        self.linear_increment = 0.1
        self.angular_increment = 0.1

        # Current velocity
        self.current_linear = 0.0
        self.current_angular = 0.0

        # Terminal settings
        self.settings = None

        # Timer to publish state periodically (10 Hz)
        self.state_timer = self.create_timer(0.1, self.publish_state)

        self.get_logger().info('Keyboard Controller Node Started')
        self.get_logger().info('Publishing cmd_vel to /robot1/cmd_vel (Direct Network → Gazebo)')
        self.get_logger().info('Publishing state to /robot1/state (WiFi network via NS-3) at 10 Hz')
        self.print_instructions()

    def print_instructions(self):
        """Print control instructions."""
        msg = """
==========================================
Keyboard Control Instructions:
==========================================
Arrow Keys / WASD:
  ↑ / W : Increase forward velocity
  ↓ / S : Increase backward velocity
  ← / A : Increase left turn
  → / D : Increase right turn

  SPACE : Stop (set all velocities to 0)
  Q/ESC : Quit

  + : Increase speed increment
  - : Decrease speed increment

Current Settings:
  Linear Speed: {:.2f} m/s
  Angular Speed: {:.2f} rad/s
==========================================
""".format(self.linear_speed, self.angular_speed)
        self.get_logger().info(msg)

    def publish_state(self):
        """
        Periodically publish robot state based on current cmd_vel.
        This runs at 10 Hz via timer.
        """
        # Create Odometry message with current velocity
        state = Odometry()
        state.header.stamp = self.get_clock().now().to_msg()
        state.header.frame_id = 'odom'
        state.child_frame_id = 'robot1/base_link'

        # Fill in twist (velocity) - this is what the follower needs
        state.twist.twist.linear.x = self.current_linear
        state.twist.twist.linear.y = 0.0
        state.twist.twist.linear.z = 0.0
        state.twist.twist.angular.x = 0.0
        state.twist.twist.angular.y = 0.0
        state.twist.twist.angular.z = self.current_angular

        # Publish to WiFi network (via NS-3)
        self.state_pub.publish(state)

        # Log state publishing (throttled)
        if not hasattr(self, '_state_log_counter'):
            self._state_log_counter = 0

        self._state_log_counter += 1
        if self._state_log_counter % 50 == 0:  # Log every 50th message (every 5 seconds at 10Hz)
            self.get_logger().info(
                f'[State] Publishing to /robot1/state: linear.x={self.current_linear:.2f}, '
                f'angular.z={self.current_angular:.2f}',
                throttle_duration_sec=5.0
            )

    def get_key(self):
        """Get a single keypress from stdin."""
        if select.select([sys.stdin], [], [], 0)[0] == [sys.stdin]:
            return sys.stdin.read(1)
        return None

    def publish_velocity(self):
        """Publish the current velocity command."""
        twist = Twist()
        twist.linear.x = self.current_linear
        twist.angular.z = self.current_angular
        self.cmd_vel_pub.publish(twist)

    def run(self):
        """Main control loop."""
        # Save terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        try:
            tty.setcbreak(sys.stdin.fileno())

            while rclpy.ok():
                key = self.get_key()

                if key is not None:
                    # Forward/Backward
                    if key == '\x1b':  # ESC sequence
                        next_keys = sys.stdin.read(2)
                        if next_keys == '[A':  # Up arrow
                            self.current_linear += self.linear_increment
                        elif next_keys == '[B':  # Down arrow
                            self.current_linear -= self.linear_increment
                        elif next_keys == '[D':  # Left arrow
                            self.current_angular += self.angular_increment
                        elif next_keys == '[C':  # Right arrow
                            self.current_angular -= self.angular_increment
                        else:  # Just ESC
                            break

                    # WASD controls
                    elif key.lower() == 'w':
                        self.current_linear += self.linear_increment
                    elif key.lower() == 's':
                        self.current_linear -= self.linear_increment
                    elif key.lower() == 'a':
                        self.current_angular += self.angular_increment
                    elif key.lower() == 'd':
                        self.current_angular -= self.angular_increment

                    # Stop
                    elif key == ' ':
                        self.current_linear = 0.0
                        self.current_angular = 0.0
                        self.get_logger().info('STOP - All velocities set to 0')

                    # Quit
                    elif key.lower() == 'q':
                        break

                    # Adjust increments
                    elif key == '+' or key == '=':
                        self.linear_increment += 0.05
                        self.angular_increment += 0.05
                        self.get_logger().info(
                            f'Speed increment increased: linear={self.linear_increment:.2f}, '
                            f'angular={self.angular_increment:.2f}'
                        )
                    elif key == '-' or key == '_':
                        self.linear_increment = max(0.05, self.linear_increment - 0.05)
                        self.angular_increment = max(0.05, self.angular_increment - 0.05)
                        self.get_logger().info(
                            f'Speed increment decreased: linear={self.linear_increment:.2f}, '
                            f'angular={self.angular_increment:.2f}'
                        )

                    # Clamp velocities
                    self.current_linear = max(-self.linear_speed,
                                             min(self.linear_speed, self.current_linear))
                    self.current_angular = max(-self.angular_speed,
                                              min(self.angular_speed, self.current_angular))

                    # Publish velocity
                    self.publish_velocity()

                    # Display current state
                    self.get_logger().info(
                        f'Velocity -> Linear: {self.current_linear:.2f} m/s, '
                        f'Angular: {self.current_angular:.2f} rad/s'
                    )

                # Spin once to process callbacks
                rclpy.spin_once(self, timeout_sec=0.01)

        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

            # Stop the robot before exiting
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Keyboard Controller Node Stopped')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardController()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
