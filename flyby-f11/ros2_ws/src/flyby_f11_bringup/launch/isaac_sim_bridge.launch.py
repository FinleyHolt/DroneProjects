"""
Isaac Sim ROS 2 Bridge Launch File for Flyby F-11.

This launch file starts all ROS 2 nodes needed for the autonomy stack
when running in Isaac Sim simulation. The camera and state data come
from Isaac Sim's OmniGraph ROS 2 bridge.

Topics from Isaac Sim (via OmniGraph):
    - /camera/image_raw: RGB camera image
    - /camera/depth: Depth image
    - /camera/camera_info: Camera calibration

Topics published by our nodes:
    - /perception/detections: YOLO detections
    - /perception/encoded: Encoded perception state
    - /uav/state: Unified UAV state
    - /autonomy/behavior_command: Active behavior
    - /autonomy/flight_phase: Current flight phase
    - /cmd_vel: Velocity commands

Usage:
    ros2 launch flyby_f11_bringup isaac_sim_bridge.launch.py

Isaac Sim must be running with the ROS 2 bridge enabled in OmniGraph.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package share directories
    perception_pkg = FindPackageShare('flyby_perception')
    autonomy_pkg = FindPackageShare('flyby_autonomy')
    bringup_pkg = FindPackageShare('flyby_f11_bringup')

    # Arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from Isaac Sim'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Camera image topic from Isaac Sim'
    )

    enable_perception_arg = DeclareLaunchArgument(
        'enable_perception',
        default_value='true',
        description='Enable YOLO perception pipeline'
    )

    enable_autonomy_arg = DeclareLaunchArgument(
        'enable_autonomy',
        default_value='true',
        description='Enable ontology autonomy controller'
    )

    # Include perception launch
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([perception_pkg, 'launch', 'perception.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'image_topic': LaunchConfiguration('image_topic'),
        }.items()
    )

    # Include autonomy launch
    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([autonomy_pkg, 'launch', 'autonomy.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'use_simulation': 'true',
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        image_topic_arg,
        enable_perception_arg,
        enable_autonomy_arg,
        perception_launch,
        autonomy_launch,
    ])
