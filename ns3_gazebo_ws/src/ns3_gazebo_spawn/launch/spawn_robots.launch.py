#!/usr/bin/env python3
"""
ROS2 Launch file to spawn multiple robots in Gazebo and register them with NS-3

Usage:
    ros2 launch ns3_gazebo_spawn spawn_robots.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to robot model
    ns3_gazebo_dir = os.path.join(os.path.expanduser('~'), 'realgazebo', 'ns3_gazebo')
    robot_sdf = os.path.join(ns3_gazebo_dir, 'models', 'robot_simple', 'model.sdf')

    ld = LaunchDescription()

    # Robot configurations: name, x, y, z positions, ns3_node_id
    robots = [
        {'name': 'robot_1', 'x': 5.0,  'y': 0.0,  'z': 0.0, 'node_id': 1},
        {'name': 'robot_2', 'x': -5.0, 'y': 0.0,  'z': 0.0, 'node_id': 2},
        {'name': 'robot_3', 'x': 0.0,  'y': 5.0,  'z': 0.0, 'node_id': 3},
        {'name': 'robot_4', 'x': 0.0,  'y': -5.0, 'z': 0.0, 'node_id': 4},
    ]

    delay = 3.0  # Initial delay for Gazebo and NS-3 plugin to be ready

    for i, robot in enumerate(robots):
        # Calculate delays
        spawn_delay = delay + (i * 2.0)  # Spawn robots 2 seconds apart
        register_delay = spawn_delay + 1.0  # Register 1 second after spawn

        # 1. Spawn robot in Gazebo using gz service
        spawn_cmd = [
            'gz', 'service',
            '-s', '/world/ns3_gazebo_world/create',
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '5000',
            '--req',
            f'sdf_filename: "{robot_sdf}", '
            f'name: "{robot["name"]}", '
            f'pose: {{position: {{x: {robot["x"]}, y: {robot["y"]}, z: {robot["z"]}}} }}'
        ]

        spawn_action = ExecuteProcess(
            cmd=spawn_cmd,
            output='screen',
            shell=False
        )

        # 2. Register robot with NS-3 using ROS2 service
        register_cmd = [
            'ros2', 'service', 'call',
            '/ns3_gazebo/register_robot',
            'ns3_gazebo_interfaces/srv/RegisterRobot',
            f'{{model_name: "{robot["name"]}", requested_node_id: {robot["node_id"]}}}'
        ]

        register_action = ExecuteProcess(
            cmd=register_cmd,
            output='screen',
            shell=False
        )

        # Add spawn action with delay
        ld.add_action(TimerAction(
            period=spawn_delay,
            actions=[
                LogInfo(msg=f'========================================'),
                LogInfo(msg=f'Spawning {robot["name"]} at ({robot["x"]}, {robot["y"]}, {robot["z"]})...'),
                LogInfo(msg=f'========================================'),
                spawn_action
            ]
        ))

        # Add register action with delay
        ld.add_action(TimerAction(
            period=register_delay,
            actions=[
                LogInfo(msg=f'Registering {robot["name"]} with NS-3 Node {robot["node_id"]}...'),
                register_action,
                LogInfo(msg=f'')
            ]
        ))

    # Final status message
    final_delay = delay + (len(robots) * 2.0) + 2.0
    ld.add_action(TimerAction(
        period=final_delay,
        actions=[
            LogInfo(msg=''),
            LogInfo(msg='========================================'),
            LogInfo(msg=f'All {len(robots)} robots spawned and registered!'),
            LogInfo(msg='========================================'),
            LogInfo(msg=''),
            LogInfo(msg='Monitor network status:'),
            LogInfo(msg='  ros2 topic echo /ns3_gazebo/network_status'),
            LogInfo(msg=''),
            LogInfo(msg='Control robots:'),
            LogInfo(msg='  ros2 topic pub /robot_1/cmd_vel geometry_msgs/msg/Twist "{{linear: {{x: 0.5}}}}"'),
            LogInfo(msg=''),
        ]
    ))

    return ld
