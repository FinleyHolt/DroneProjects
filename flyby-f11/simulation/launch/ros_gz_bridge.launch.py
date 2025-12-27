#!/usr/bin/env python3
"""
ROS GZ Bridge Launch File for F-11 ISR Drone

Launches the ros_gz_bridge node to bridge Gazebo topics to ROS 2.

Usage:
    ros2 launch flyby_f11_simulation ros_gz_bridge.launch.py

    # With custom world/model names:
    ros2 launch flyby_f11_simulation ros_gz_bridge.launch.py \
        world_name:=custom_world model_name:=custom_model

    # With custom config file:
    ros2 launch flyby_f11_simulation ros_gz_bridge.launch.py \
        config_file:=/path/to/custom_config.yaml

Topics Bridged (GZ -> ROS):
    /f11/camera/image_raw   - Camera image from ISR payload
    /f11/camera/camera_info - Camera intrinsic parameters
    /f11/imu                - IMU data
    /f11/gps                - GPS NavSat fix

Topics Bridged (ROS -> GZ):
    /f11/gimbal/pitch_cmd   - Gimbal pitch command
    /f11/gimbal/yaw_cmd     - Gimbal yaw command
    /f11/gimbal/roll_cmd    - Gimbal roll command
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    # Get the directory containing this launch file
    # Default config is relative to the simulation directory
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_config = os.path.join(pkg_dir, 'config', 'ros_gz_bridge.yaml')

    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the ros_gz_bridge YAML configuration file'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='flyby_training',
        description='Gazebo world name (used for topic namespacing)'
    )

    model_name_arg = DeclareLaunchArgument(
        'model_name',
        default_value='f11_isr',
        description='Gazebo model name (used for topic namespacing)'
    )

    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    world_name = LaunchConfiguration('world_name')
    model_name = LaunchConfiguration('model_name')

    # ROS GZ Bridge Node
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': config_file,
        }],
    )

    # Log startup information
    startup_log = LogInfo(
        msg=['ROS GZ Bridge starting with config: ', config_file]
    )

    return LaunchDescription([
        # Arguments
        config_file_arg,
        world_name_arg,
        model_name_arg,

        # Logging
        startup_log,

        # Nodes
        ros_gz_bridge_node,
    ])
