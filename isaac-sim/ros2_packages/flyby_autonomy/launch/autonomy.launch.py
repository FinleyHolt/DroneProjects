"""
Autonomy launch file for Flyby F-11 UAV.

Launches ontology controller and state estimator nodes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('flyby_autonomy')

    # Arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'autonomy.yaml']),
        description='Path to autonomy config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    use_simulation_arg = DeclareLaunchArgument(
        'use_simulation',
        default_value='true',
        description='Enable simulation mode'
    )

    # State Estimator Node
    state_estimator_node = Node(
        package='flyby_autonomy',
        executable='state_estimator_node',
        name='state_estimator_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_simulation': LaunchConfiguration('use_simulation'),
            }
        ]
    )

    # Ontology Controller Node
    ontology_controller_node = Node(
        package='flyby_autonomy',
        executable='ontology_controller_node',
        name='ontology_controller_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        use_simulation_arg,
        state_estimator_node,
        ontology_controller_node,
    ])
