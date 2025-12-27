#!/usr/bin/env python3
"""
Flyby F-11 RL Training Launch File

Launches the complete training environment:
- MAVROS bridge (connects to SITL)
- State Grounding Node (connects MAVROS to ontology)
- Vampire Bridge (ontology reasoning)
- Episode Manager (RL episode management)

Usage:
    ros2 launch flyby_f11_simulation training.launch.py

Arguments:
    headless:=true          - Run without visualization
    sitl_instance:=0        - SITL instance number
    world_config:=path      - Path to world config JSON
    enable_vampire:=true    - Enable vampire reasoning
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run in headless mode (no GUI)'
    )

    sitl_instance_arg = DeclareLaunchArgument(
        'sitl_instance',
        default_value='0',
        description='SITL instance number for parallel training'
    )

    world_config_arg = DeclareLaunchArgument(
        'world_config',
        default_value='/simulation/worlds/training_world_config.json',
        description='Path to world configuration JSON'
    )

    enable_vampire_arg = DeclareLaunchArgument(
        'enable_vampire',
        default_value='true',
        description='Enable Vampire reasoning bridge'
    )

    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://127.0.0.1:14550@127.0.0.1:14551',
        description='FCU connection URL'
    )

    ontology_path_arg = DeclareLaunchArgument(
        'ontology_path',
        default_value='/workspace/ontology/planning_mode',
        description='Path to ontology files'
    )

    # Get launch configurations
    headless = LaunchConfiguration('headless')
    sitl_instance = LaunchConfiguration('sitl_instance')
    world_config = LaunchConfiguration('world_config')
    enable_vampire = LaunchConfiguration('enable_vampire')
    fcu_url = LaunchConfiguration('fcu_url')
    ontology_path = LaunchConfiguration('ontology_path')

    # MAVROS node
    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        name='mavros',
        output='screen',
        parameters=[{
            'fcu_url': fcu_url,
            'gcs_url': '',
            'target_system_id': 1,
            'target_component_id': 1,
            'fcu_protocol': 'v2.0',
        }],
        remappings=[
            # Remap to flyby namespace
            ('mavros/local_position/pose', '/flyby/state/pose'),
            ('mavros/local_position/velocity_local', '/flyby/state/velocity'),
            ('mavros/global_position/global', '/flyby/state/global_position'),
            ('mavros/state', '/flyby/state/fcu_state'),
            ('mavros/battery', '/flyby/state/battery'),
        ],
    )

    # State Grounding Node
    # Converts MAVROS topics to ontology-compatible state
    state_grounding_node = Node(
        package='vampire_bridge',
        executable='state_grounding_node',
        name='state_grounding',
        output='screen',
        parameters=[{
            'world_config': world_config,
            'update_rate_hz': 10.0,
            'geofence_buffer_m': 10.0,
            'nfz_buffer_m': 5.0,
        }],
        remappings=[
            ('input/pose', '/flyby/state/pose'),
            ('input/velocity', '/flyby/state/velocity'),
            ('input/battery', '/flyby/state/battery'),
            ('output/grounded_state', '/flyby/ontology/grounded_state'),
            ('output/safety_status', '/flyby/safety/status'),
        ],
    )

    # Vampire Bridge Node (conditional)
    vampire_node = Node(
        package='vampire_bridge',
        executable='vampire_node',
        name='vampire_bridge',
        output='screen',
        condition=IfCondition(enable_vampire),
        parameters=[{
            'vampire_binary': '/usr/local/bin/vampire',
            'ontology_path': ontology_path,
            'default_timeout_ms': 100,
            'cache_size': 1000,
            'max_concurrent_queries': 4,
        }],
        remappings=[
            ('input/grounded_state', '/flyby/ontology/grounded_state'),
            ('output/query_result', '/flyby/ontology/query_result'),
        ],
    )

    # Episode Manager Node
    episode_manager_node = Node(
        package='flyby_f11_simulation',
        executable='episode_manager_node',
        name='episode_manager',
        output='screen',
        parameters=[{
            'world_config': world_config,
            'reset_timeout_s': 10.0,
            'default_spawn_position': [0.0, 0.0, 0.0],
            'default_spawn_heading': 0.0,
            'max_episode_duration_s': 300.0,
        }],
        remappings=[
            ('input/pose', '/flyby/state/pose'),
            ('input/fcu_state', '/flyby/state/fcu_state'),
            ('output/episode_info', '/flyby/training/episode_info'),
            ('service/reset_episode', '/flyby/training/reset_episode'),
            ('service/get_spawn_position', '/flyby/training/get_spawn_position'),
        ],
    )

    # Delayed start for MAVROS (wait for SITL)
    delayed_mavros = TimerAction(
        period=5.0,
        actions=[mavros_node],
    )

    # Delayed start for state grounding (wait for MAVROS)
    delayed_state_grounding = TimerAction(
        period=10.0,
        actions=[state_grounding_node],
    )

    # Delayed start for vampire (wait for state grounding)
    delayed_vampire = TimerAction(
        period=12.0,
        actions=[vampire_node],
    )

    # Delayed start for episode manager
    delayed_episode_manager = TimerAction(
        period=15.0,
        actions=[episode_manager_node],
    )

    # Log startup
    startup_log = LogInfo(
        msg=['Flyby F-11 Training Environment Starting...']
    )
    mavros_log = LogInfo(
        msg=['MAVROS connecting to FCU at: ', fcu_url]
    )
    world_log = LogInfo(
        msg=['Loading world config: ', world_config]
    )

    return LaunchDescription([
        # Arguments
        headless_arg,
        sitl_instance_arg,
        world_config_arg,
        enable_vampire_arg,
        fcu_url_arg,
        ontology_path_arg,

        # Logging
        startup_log,
        mavros_log,
        world_log,

        # Nodes (with delays for proper startup sequence)
        delayed_mavros,
        delayed_state_grounding,
        delayed_vampire,
        delayed_episode_manager,
    ])
