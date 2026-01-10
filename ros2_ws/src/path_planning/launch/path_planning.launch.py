"""
Path Planning Launch File

Launches the global planner and trajectory controller nodes.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_path_planning = get_package_share_directory('path_planning')

    # Load parameters from YAML
    config_file = os.path.join(pkg_path_planning, 'config', 'planner_params.yaml')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Global Planner Node
    global_planner_node = Node(
        package='path_planning',
        executable='global_planner_node',
        name='global_planner',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    # Trajectory Controller Node
    trajectory_controller_node = Node(
        package='path_planning',
        executable='trajectory_controller_node',
        name='trajectory_controller',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        global_planner_node,
        trajectory_controller_node,
    ])
