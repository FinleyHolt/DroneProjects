"""Launch file for depth estimation node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('flyby_depth')

    # Declare launch arguments
    model_size_arg = DeclareLaunchArgument(
        'model_size',
        default_value='small',
        description='DepthAnything V2 model size: small, base, or large'
    )

    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Inference device: cuda or cpu'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='15.0',
        description='Depth map publish rate (Hz)'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )

    rangefinder_topic_arg = DeclareLaunchArgument(
        'rangefinder_topic',
        default_value='/rangefinder/range',
        description='Rangefinder topic'
    )

    # Load parameters from config file
    config_file = os.path.join(pkg_share, 'config', 'depth_params.yaml')

    # Depth estimation node
    depth_node = Node(
        package='flyby_depth',
        executable='depth_estimation_node',
        name='depth_estimation_node',
        output='screen',
        parameters=[
            config_file,
            {
                'model_size': LaunchConfiguration('model_size'),
                'device': LaunchConfiguration('device'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'image_topic': LaunchConfiguration('image_topic'),
                'rangefinder_topic': LaunchConfiguration('rangefinder_topic'),
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    return LaunchDescription([
        model_size_arg,
        device_arg,
        publish_rate_arg,
        image_topic_arg,
        rangefinder_topic_arg,
        depth_node,
    ])
