#!/usr/bin/env python3
"""
Distance Experiment Controller

Moves the vehicle in a straight line at constant speed for distance-based
network quality measurements.
"""

import time
import subprocess
import sys

def send_cmd_vel(linear_x, angular_z=0.0):
    """Send velocity command to Gazebo"""
    cmd = [
        'gz', 'topic', '-t', '/cmd_vel', '-m', 'gz.msgs.Twist',
        '-p', f'linear: {{x: {linear_x}}}, angular: {{z: {angular_z}}}'
    ]
    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def main():
    """
    Move vehicle forward at constant speed for distance experiment

    Parameters:
    - Speed: 0.5 m/s
    - Duration: 120 seconds
    - Expected distance: ~100m
    """

    speed = 0.5  # m/s
    duration = 120  # seconds

    print("Distance Experiment Controller")
    print(f"Speed: {speed} m/s")
    print(f"Duration: {duration} seconds")
    print(f"Expected distance: {speed * duration} m")
    print()

    # Wait for Gazebo to start
    print("Waiting 5 seconds for Gazebo to initialize...")
    time.sleep(5)

    print("Starting movement...")
    start_time = time.time()

    try:
        while (time.time() - start_time) < duration:
            # Send forward command
            send_cmd_vel(speed)

            # Status update every 10 seconds
            elapsed = time.time() - start_time
            if int(elapsed) % 10 == 0 and int(elapsed) > 0:
                distance = speed * elapsed
                print(f"  {int(elapsed):3d}s - Distance: ~{distance:.1f}m")

            time.sleep(0.1)  # Send commands at 10Hz

    except KeyboardInterrupt:
        print("\nStopped by user")

    finally:
        # Stop the vehicle
        print("Stopping vehicle...")
        send_cmd_vel(0.0)
        time.sleep(0.5)
        send_cmd_vel(0.0)

    elapsed = time.time() - start_time
    distance = speed * elapsed
    print(f"\nExperiment completed:")
    print(f"  Duration: {elapsed:.1f}s")
    print(f"  Distance: ~{distance:.1f}m")

if __name__ == "__main__":
    main()
