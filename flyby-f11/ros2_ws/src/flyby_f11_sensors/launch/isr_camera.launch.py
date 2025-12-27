"""Launch file for F-11 ISR camera payload sensors."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for ISR camera sensors."""
    pkg_share = get_package_share_directory('flyby_f11_sensors')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    enable_limit_warnings = DeclareLaunchArgument(
        'enable_limit_warnings',
        default_value='true',
        description='Enable gimbal limit warnings'
    )

    return LaunchDescription([
        use_sim_time,
        enable_limit_warnings,

        # Gimbal controller node
        Node(
            package='flyby_f11_sensors',
            executable='gimbal_controller_node.py',
            name='gimbal_controller',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'gimbal_params.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'enable_limit_warnings': LaunchConfiguration('enable_limit_warnings')},
            ],
            remappings=[
                # No remappings needed - uses standard F-11 topics
            ]
        ),

        # ISR camera node
        Node(
            package='flyby_f11_sensors',
            executable='isr_camera_node.py',
            name='isr_camera',
            output='screen',
            parameters=[
                os.path.join(pkg_share, 'config', 'camera_params.yaml'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                # No remappings needed - uses standard F-11 topics
            ]
        ),
    ])
