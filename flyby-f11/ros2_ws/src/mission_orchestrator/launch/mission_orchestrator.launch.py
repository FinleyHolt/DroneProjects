#!/usr/bin/env python3
"""
mission_orchestrator.launch.py - Launch file for mission orchestration system

Launches:
    - mission_manager_node: Main orchestrator with services and action server

Dependencies:
    - vampire_bridge must be running (vampire_node)
    - state_grounding_node should be running for hot fact updates

Usage:
    ros2 launch mission_orchestrator mission_orchestrator.launch.py

Parameters can be overridden via command line:
    ros2 launch mission_orchestrator mission_orchestrator.launch.py \
        planning_timeout_ms:=3000 safety_check_hz:=10.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for mission orchestrator."""

    # Get package share directory
    pkg_share = get_package_share_directory('mission_orchestrator')
    config_file = os.path.join(pkg_share, 'config', 'orchestrator_params.yaml')

    # Declare launch arguments
    planning_timeout_arg = DeclareLaunchArgument(
        'planning_timeout_ms',
        default_value='5000',
        description='Timeout for planning queries in milliseconds'
    )

    tactical_timeout_arg = DeclareLaunchArgument(
        'tactical_timeout_ms',
        default_value='100',
        description='Timeout for tactical queries in milliseconds'
    )

    safety_check_hz_arg = DeclareLaunchArgument(
        'safety_check_hz',
        default_value='20.0',
        description='Safety monitoring rate in Hz'
    )

    status_publish_hz_arg = DeclareLaunchArgument(
        'status_publish_hz',
        default_value='1.0',
        description='Status publishing rate in Hz'
    )

    # Mission manager node
    mission_manager_node = Node(
        package='mission_orchestrator',
        executable='mission_manager_node.py',
        name='mission_manager',
        parameters=[
            config_file,
            {
                'planning_timeout_ms': LaunchConfiguration('planning_timeout_ms'),
                'tactical_timeout_ms': LaunchConfiguration('tactical_timeout_ms'),
                'safety_check_hz': LaunchConfiguration('safety_check_hz'),
                'status_publish_hz': LaunchConfiguration('status_publish_hz'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        # Launch arguments
        planning_timeout_arg,
        tactical_timeout_arg,
        safety_check_hz_arg,
        status_publish_hz_arg,
        # Nodes
        mission_manager_node,
    ])
