"""Launch file for local avoidance node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('local_avoidance')

    # Declare launch arguments
    update_rate_arg = DeclareLaunchArgument(
        'update_rate',
        default_value='20.0',
        description='VFH+ update rate (Hz)'
    )

    safety_radius_arg = DeclareLaunchArgument(
        'safety_radius',
        default_value='5.0',
        description='Minimum obstacle clearance (meters)'
    )

    critical_radius_arg = DeclareLaunchArgument(
        'critical_radius',
        default_value='2.0',
        description='Emergency stop distance (meters)'
    )

    lookahead_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='20.0',
        description='Obstacle detection range (meters)'
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/depth/points',
        description='Input point cloud topic'
    )

    # Load parameters from config file
    config_file = os.path.join(pkg_share, 'config', 'avoidance_params.yaml')

    # Local avoidance node
    avoidance_node = Node(
        package='local_avoidance',
        executable='local_avoidance_node',
        name='local_avoidance_node',
        output='screen',
        parameters=[
            config_file,
            {
                'update_rate': LaunchConfiguration('update_rate'),
                'vfh.safety_radius': LaunchConfiguration('safety_radius'),
                'vfh.critical_radius': LaunchConfiguration('critical_radius'),
                'vfh.lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            }
        ],
        remappings=[
            # Add any topic remappings here if needed
        ]
    )

    return LaunchDescription([
        update_rate_arg,
        safety_radius_arg,
        critical_radius_arg,
        lookahead_arg,
        pointcloud_topic_arg,
        avoidance_node,
    ])
