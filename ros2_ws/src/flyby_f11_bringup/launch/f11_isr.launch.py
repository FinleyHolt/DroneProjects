"""
Launch F-11 ISR camera payload sensors only.

This launch file starts the sensor nodes without Gazebo simulation.
Use this when:
- Gazebo is already running
- Testing sensor nodes independently
- Running on real hardware (future)

Nodes started:
- gimbal_controller: 3-axis gimbal control with ontology limits
- isr_camera: Camera interface and state publishing
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for F-11 ISR payload."""
    sensors_share = get_package_share_directory('flyby_f11_sensors')

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

    republish_images = DeclareLaunchArgument(
        'republish_images',
        default_value='true',
        description='Republish camera images to ROS 2 topic'
    )

    # Include ISR camera sensors launch
    isr_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_share, 'launch', 'isr_camera.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'enable_limit_warnings': LaunchConfiguration('enable_limit_warnings'),
        }.items()
    )

    return LaunchDescription([
        use_sim_time,
        enable_limit_warnings,
        republish_images,
        isr_sensors,
    ])
