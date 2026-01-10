"""
Perception launch file for Flyby F-11 UAV.

Launches YOLO detector and perception encoder nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('flyby_perception')

    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'perception.yaml']),
        description='Path to perception config file'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # YOLO Detector Node
    yolo_detector_node = Node(
        package='flyby_perception',
        executable='yolo_detector_node',
        name='yolo_detector_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'image_topic': LaunchConfiguration('image_topic'),
            }
        ],
        remappings=[
            ('/camera/image_raw', LaunchConfiguration('image_topic')),
        ]
    )

    # Perception Encoder Node
    perception_encoder_node = Node(
        package='flyby_perception',
        executable='perception_encoder_node',
        name='perception_encoder_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        config_file_arg,
        image_topic_arg,
        use_sim_time_arg,
        yolo_detector_node,
        perception_encoder_node,
    ])
