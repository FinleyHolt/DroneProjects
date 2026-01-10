"""
RL Inference Launch File

Launches the TensorRT-based RL policy inference node.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_rl_inference = get_package_share_directory('rl_inference')

    # Load parameters from YAML
    config_file = os.path.join(pkg_rl_inference, 'config', 'inference_params.yaml')

    # Model paths
    models_dir = os.path.join(pkg_rl_inference, 'models')

    # Declare launch arguments
    search_engine_arg = DeclareLaunchArgument(
        'search_policy_engine',
        default_value=os.path.join(models_dir, 'search_policy_fp16.engine'),
        description='Path to SearchPolicy TensorRT engine'
    )

    dwell_engine_arg = DeclareLaunchArgument(
        'dwell_policy_engine',
        default_value=os.path.join(models_dir, 'dwell_policy_fp16.engine'),
        description='Path to DwellPolicy TensorRT engine'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # RL Inference Node
    rl_inference_node = Node(
        package='rl_inference',
        executable='rl_inference_node',
        name='rl_inference',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'search_policy_engine': LaunchConfiguration('search_policy_engine'),
                'dwell_policy_engine': LaunchConfiguration('dwell_policy_engine'),
            }
        ],
        remappings=[
            # Remap topics if needed
        ],
    )

    return LaunchDescription([
        search_engine_arg,
        dwell_engine_arg,
        use_sim_time_arg,
        rl_inference_node,
    ])
