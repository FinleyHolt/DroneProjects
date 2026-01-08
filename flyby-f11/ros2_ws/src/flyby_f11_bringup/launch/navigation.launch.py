"""
Launch file for complete navigation stack.

Launches:
1. Depth estimation node (DepthAnything V2 + rangefinder scale)
2. Local avoidance node (3D-VFH+)
3. Trajectory controller (with avoidance integration)
4. Global planner (existing)

Usage:
    ros2 launch flyby_f11_bringup navigation.launch.py
    ros2 launch flyby_f11_bringup navigation.launch.py use_sim:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directories
    flyby_depth_share = get_package_share_directory('flyby_depth')
    local_avoidance_share = get_package_share_directory('local_avoidance')
    path_planning_share = get_package_share_directory('path_planning')

    # Declare launch arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation time and parameters'
    )

    depth_model_arg = DeclareLaunchArgument(
        'depth_model',
        default_value='small',
        description='DepthAnything model size: small, base, large'
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

    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='10.0',
        description='Maximum flight velocity (m/s)'
    )

    # Config files
    depth_config = os.path.join(flyby_depth_share, 'config', 'depth_params.yaml')
    avoidance_config = os.path.join(local_avoidance_share, 'config', 'avoidance_params.yaml')

    # Depth estimation node
    depth_node = Node(
        package='flyby_depth',
        executable='depth_estimation_node',
        name='depth_estimation_node',
        output='screen',
        parameters=[
            depth_config,
            {
                'model_size': LaunchConfiguration('depth_model'),
                'device': 'cuda',
                'publish_rate': 15.0,
            }
        ],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/rangefinder/range', '/rangefinder/range'),
        ]
    )

    # Local avoidance node
    avoidance_node = Node(
        package='local_avoidance',
        executable='local_avoidance_node',
        name='local_avoidance_node',
        output='screen',
        parameters=[
            avoidance_config,
            {
                'vfh.safety_radius': LaunchConfiguration('safety_radius'),
                'vfh.critical_radius': LaunchConfiguration('critical_radius'),
                'update_rate': 20.0,
            }
        ],
        remappings=[
            ('/depth/points', '/depth/points'),
            ('/uav/state', '/uav/state'),
        ]
    )

    # Trajectory controller node
    trajectory_node = Node(
        package='path_planning',
        executable='trajectory_controller_node',
        name='trajectory_controller_node',
        output='screen',
        parameters=[
            {
                'max_linear_velocity': LaunchConfiguration('max_velocity'),
                'lookahead_distance': 5.0,
                'xy_goal_tolerance': 2.0,
                'z_goal_tolerance': 1.0,
            }
        ],
        remappings=[
            ('/uav/state', '/uav/state'),
            ('/cmd_vel', '/cmd_vel'),
            ('/avoidance/waypoint_override', '/avoidance/waypoint_override'),
        ]
    )

    # Global planner node
    global_planner_node = Node(
        package='path_planning',
        executable='global_planner_node',
        name='global_planner_node',
        output='screen',
        parameters=[
            {
                'planning_timeout': 2.0,
                'goal_tolerance': 2.0,
            }
        ],
        remappings=[
            ('/uav/state', '/uav/state'),
        ]
    )

    return LaunchDescription([
        # Arguments
        use_sim_arg,
        depth_model_arg,
        safety_radius_arg,
        critical_radius_arg,
        max_velocity_arg,

        # Nodes
        depth_node,
        avoidance_node,
        trajectory_node,
        global_planner_node,
    ])
