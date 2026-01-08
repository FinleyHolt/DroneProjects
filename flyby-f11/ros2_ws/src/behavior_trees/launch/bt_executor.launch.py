"""
BT Executor Launch File

Launches the Behavior Tree executor node for ISR missions.
Configures the tree file path and loads parameters from YAML config.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_behavior_trees = get_package_share_directory('behavior_trees')

    # Declare launch arguments
    tree_file_arg = DeclareLaunchArgument(
        'tree_file',
        default_value=os.path.join(pkg_behavior_trees, 'trees', 'isr_mission.xml'),
        description='Path to the behavior tree XML file'
    )

    tick_rate_arg = DeclareLaunchArgument(
        'tick_rate_hz',
        default_value='50.0',
        description='BT tick rate in Hz'
    )

    enable_logging_arg = DeclareLaunchArgument(
        'enable_logging',
        default_value='false',
        description='Enable BT cout logger for debugging'
    )

    # Load parameters from YAML
    config_file = os.path.join(pkg_behavior_trees, 'config', 'bt_params.yaml')

    # BT Executor Node
    bt_executor_node = Node(
        package='behavior_trees',
        executable='bt_executor_node',
        name='bt_executor',
        output='screen',
        parameters=[
            config_file,
            {
                'tree_file': LaunchConfiguration('tree_file'),
                'tick_rate_hz': LaunchConfiguration('tick_rate_hz'),
                'enable_logging': LaunchConfiguration('enable_logging'),
            }
        ],
        remappings=[
            # Remap topics if needed
            # ('/flyby/uav_state', '/uav_state'),
        ],
    )

    return LaunchDescription([
        tree_file_arg,
        tick_rate_arg,
        enable_logging_arg,
        bt_executor_node,
    ])
