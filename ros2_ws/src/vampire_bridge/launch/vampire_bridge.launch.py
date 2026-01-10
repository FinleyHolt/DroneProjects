"""
vampire_bridge.launch.py - Launch file for Vampire reasoning node

Launches the VampireNode with configurable parameters for
UAV ontological reasoning.

Usage:
    ros2 launch vampire_bridge vampire_bridge.launch.py

    # With custom parameters
    ros2 launch vampire_bridge vampire_bridge.launch.py \
        vampire_binary:=/custom/path/vampire \
        cache_size:=2000
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for vampire_bridge."""

    # Get package share directory
    pkg_share = get_package_share_directory('vampire_bridge')

    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'vampire_params.yaml')

    # Declare launch arguments
    vampire_binary_arg = DeclareLaunchArgument(
        'vampire_binary',
        default_value='/usr/local/bin/vampire',
        description='Path to Vampire theorem prover binary'
    )

    ontology_path_arg = DeclareLaunchArgument(
        'ontology_path',
        default_value='/workspace/ontology/planning_mode',
        description='Path to UAV domain ontology'
    )

    templates_path_arg = DeclareLaunchArgument(
        'templates_path',
        default_value='/workspace/ontology/evaluation/benchmark_queries',
        description='Path to benchmark query templates'
    )

    default_timeout_arg = DeclareLaunchArgument(
        'default_timeout_ms',
        default_value='100',
        description='Default query timeout in milliseconds'
    )

    cache_size_arg = DeclareLaunchArgument(
        'cache_size',
        default_value='1000',
        description='Maximum number of cached query results'
    )

    # Vampire reasoning node
    vampire_node = Node(
        package='vampire_bridge',
        executable='vampire_node.py',
        name='vampire_reasoning',
        output='screen',
        parameters=[
            default_config,
            {
                'vampire_binary': LaunchConfiguration('vampire_binary'),
                'ontology_path': LaunchConfiguration('ontology_path'),
                'templates_path': LaunchConfiguration('templates_path'),
                'default_timeout_ms': LaunchConfiguration('default_timeout_ms'),
                'cache_size': LaunchConfiguration('cache_size'),
            }
        ],
        # Remap service if needed
        # remappings=[
        #     ('/vampire/query', '/reasoning/query'),
        # ],
    )

    return LaunchDescription([
        # Launch arguments
        vampire_binary_arg,
        ontology_path_arg,
        templates_path_arg,
        default_timeout_arg,
        cache_size_arg,
        # Nodes
        vampire_node,
    ])
