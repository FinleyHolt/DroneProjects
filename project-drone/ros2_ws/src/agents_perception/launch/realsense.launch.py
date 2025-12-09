#!/usr/bin/env python3
"""
RealSense camera launch file for LLMDrone perception stack.

Launches both T265 (tracking) and D455 (depth) cameras with optimal settings
for autonomous drone operations.

Usage:
    ros2 launch agents_perception realsense.launch.py
    ros2 launch agents_perception realsense.launch.py enable_t265:=false  # D455 only
    ros2 launch agents_perception realsense.launch.py enable_d455:=false  # T265 only
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for RealSense cameras."""

    # Launch arguments
    enable_t265_arg = DeclareLaunchArgument(
        'enable_t265',
        default_value='true',
        description='Enable T265 tracking camera'
    )

    enable_d455_arg = DeclareLaunchArgument(
        'enable_d455',
        default_value='true',
        description='Enable D455 depth camera'
    )

    # T265 Tracking Camera Launch
    t265_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('realsense2_camera'),
            'launch',
            'rs_launch.py'
        ]),
        launch_arguments={
            'camera_name': 't265',
            'device_type': 't265',
            'serial_no': '',  # Auto-detect
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'enable_pose': 'true',
            'enable_fisheye1': 'true',
            'enable_fisheye2': 'true',
            'unite_imu_method': '1',  # Linear interpolation
            'publish_tf': 'true',
            'tf_publish_rate': '200.0',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_t265'))
    )

    # D455 Depth Camera Launch
    d455_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('realsense2_camera'),
            'launch',
            'rs_launch.py'
        ]),
        launch_arguments={
            'camera_name': 'd455',
            'device_type': 'd455',
            'serial_no': '',  # Auto-detect
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'align_depth.enable': 'true',
            'pointcloud.enable': 'true',
            'rgb_camera.profile': '640x480x30',  # Balance quality/performance
            'depth_module.profile': '640x480x30',
            'publish_tf': 'true',
        }.items(),
        condition=IfCondition(LaunchConfiguration('enable_d455'))
    )

    return LaunchDescription([
        enable_t265_arg,
        enable_d455_arg,
        t265_launch,
        d455_launch,
    ])
