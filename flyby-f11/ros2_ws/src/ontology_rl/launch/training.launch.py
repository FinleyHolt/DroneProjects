"""
Launch file for RL training environment with simulation.

This launch file starts:
1. Gazebo simulation with training arena world
2. F-11 UAV model (if available)
3. Vampire bridge for ontology queries
4. Mission orchestrator
5. Gym environment node

Usage:
    ros2 launch ontology_rl training.launch.py
    ros2 launch ontology_rl training.launch.py env_id:=FlybyBehaviorSelector-v0
    ros2 launch ontology_rl training.launch.py headless:=true
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for training environment."""

    # Package directories
    ontology_rl_share = get_package_share_directory('ontology_rl')

    # Try to find other packages (may not exist yet)
    try:
        bringup_share = get_package_share_directory('flyby_f11_bringup')
        has_bringup = True
    except Exception:
        bringup_share = None
        has_bringup = False

    try:
        vampire_share = get_package_share_directory('vampire_bridge')
        has_vampire = True
    except Exception:
        vampire_share = None
        has_vampire = False

    try:
        orchestrator_share = get_package_share_directory('mission_orchestrator')
        has_orchestrator = True
    except Exception:
        orchestrator_share = None
        has_orchestrator = False

    # Launch arguments
    env_id_arg = DeclareLaunchArgument(
        'env_id',
        default_value='FlybyMissionPlanner-v0',
        description='Gymnasium environment ID'
    )

    max_steps_arg = DeclareLaunchArgument(
        'max_steps',
        default_value='1000',
        description='Maximum steps per episode'
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run simulation in headless mode (no GUI)'
    )

    auto_reset_arg = DeclareLaunchArgument(
        'auto_reset',
        default_value='true',
        description='Automatically reset environment on episode end'
    )

    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='true',
        description='Launch Gazebo simulation'
    )

    use_vampire_arg = DeclareLaunchArgument(
        'use_vampire',
        default_value='true',
        description='Launch Vampire theorem prover bridge'
    )

    use_orchestrator_arg = DeclareLaunchArgument(
        'use_orchestrator',
        default_value='false',
        description='Launch mission orchestrator'
    )

    # Launch configurations
    env_id = LaunchConfiguration('env_id')
    max_steps = LaunchConfiguration('max_steps')
    headless = LaunchConfiguration('headless')
    auto_reset = LaunchConfiguration('auto_reset')
    use_sim = LaunchConfiguration('use_sim')
    use_vampire = LaunchConfiguration('use_vampire')
    use_orchestrator = LaunchConfiguration('use_orchestrator')

    # Gym environment node
    gym_env_node = Node(
        package='ontology_rl',
        executable='gym_env_node',
        name='gym_env',
        output='screen',
        parameters=[{
            'env_id': env_id,
            'max_steps': max_steps,
            'auto_reset': auto_reset,
            'publish_rate': 10.0,
        }],
    )

    # Build launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(env_id_arg)
    ld.add_action(max_steps_arg)
    ld.add_action(headless_arg)
    ld.add_action(auto_reset_arg)
    ld.add_action(use_sim_arg)
    ld.add_action(use_vampire_arg)
    ld.add_action(use_orchestrator_arg)

    # Include simulation launch if available
    if has_bringup:
        sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, 'launch', 'simulation.launch.py')
            ),
            condition=IfCondition(use_sim),
        )
        ld.add_action(sim_launch)

    # Vampire bridge node (if available)
    if has_vampire:
        vampire_node = Node(
            package='vampire_bridge',
            executable='vampire_node',
            name='vampire_bridge',
            output='screen',
            condition=IfCondition(use_vampire),
        )
        ld.add_action(vampire_node)

    # Mission orchestrator (if available)
    if has_orchestrator:
        orchestrator_node = Node(
            package='mission_orchestrator',
            executable='mission_manager_node',
            name='mission_manager',
            output='screen',
            condition=IfCondition(use_orchestrator),
        )
        ld.add_action(orchestrator_node)

    # Delay gym env node to allow simulation to start
    delayed_gym_node = TimerAction(
        period=2.0,  # Wait 2 seconds for simulation
        actions=[gym_env_node],
    )
    ld.add_action(delayed_gym_node)

    return ld
