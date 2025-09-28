#!/usr/bin/env python3

"""
ROS2 Launch file for Simple Robot Navigation Example

This launch file starts all necessary nodes for the robot navigation
demonstration including Gazebo simulation, navigation stack, and
network monitoring.

Author: NS3-Gazebo Project
License: MIT License
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Package and file paths
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')
    pkg_nav2_bringup = FindPackageShare(package='nav2_bringup').find('nav2_bringup')

    # Get the directory of this launch file
    example_dir = os.path.dirname(os.path.realpath(__file__))

    # Configuration files
    world_file = os.path.join(example_dir, 'robot_world.world')
    robot_urdf = os.path.join(example_dir, 'urdf', 'diff_drive_robot.urdf')
    nav_params = os.path.join(example_dir, 'config', 'nav_params.yaml')
    robot_config = os.path.join(example_dir, 'config', 'robot_config.yaml')
    map_file = os.path.join(example_dir, 'maps', 'simple_map.yaml')

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true')

    declare_headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode')

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world file to load')

    declare_robot_name = DeclareLaunchArgument(
        'robot_name',
        default_value='diff_drive_robot',
        description='Name of the robot')

    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position of robot')

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y position of robot')

    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='0.1',
        description='Initial z position of robot')

    declare_yaw = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
        description='Initial yaw orientation of robot')

    declare_enable_network = DeclareLaunchArgument(
        'enable_network',
        default_value='true',
        description='Enable network simulation')

    # Gazebo launch
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'false',
            'gui': PythonExpression([
                "not ", LaunchConfiguration('headless')
            ])
        }.items()
    )

    # Robot state publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': open(robot_urdf, 'r').read()
        }]
    )

    # Spawn robot in Gazebo
    spawn_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', LaunchConfiguration('robot_name'),
            '-file', robot_urdf,
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose'),
            '-Y', LaunchConfiguration('yaw')
        ],
        output='screen'
    )

    # Navigation launch
    nav2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': nav_params
        }.items()
    )

    # Robot controller node
    robot_controller_cmd = Node(
        package='robot_navigation_example',
        executable='robot_controller.py',
        name='robot_controller',
        output='screen',
        parameters=[
            robot_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Network monitor node
    network_monitor_cmd = Node(
        package='robot_navigation_example',
        executable='network_monitor.py',
        name='network_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_network': LaunchConfiguration('enable_network')
        }],
        condition=IfCondition(LaunchConfiguration('enable_network'))
    )

    # Goal sender node (for automated goal setting)
    goal_sender_cmd = Node(
        package='robot_navigation_example',
        executable='goal_sender.py',
        name='goal_sender',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # RViz2 for visualization
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(example_dir, 'config', 'robot_nav.rviz')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        output='screen'
    )

    # TF static transforms
    tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser_frame']
    )

    tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_headless)
    ld.add_action(declare_world)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)
    ld.add_action(declare_yaw)
    ld.add_action(declare_enable_network)

    # Add actions
    ld.add_action(gazebo_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(tf_base_to_laser)
    ld.add_action(tf_map_to_odom)
    ld.add_action(nav2_cmd)
    ld.add_action(robot_controller_cmd)
    ld.add_action(network_monitor_cmd)
    ld.add_action(goal_sender_cmd)
    ld.add_action(rviz_cmd)

    return ld