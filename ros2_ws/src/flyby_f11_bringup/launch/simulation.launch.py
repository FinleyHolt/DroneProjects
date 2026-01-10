"""
Launch F-11 simulation with ISR camera payload.

This launch file starts:
1. Gazebo simulation with flyby_training world
2. F-11 ISR camera model spawn
3. Gazebo-ROS bridges for sensors
4. ISR sensor nodes (gimbal controller, camera interface)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for F-11 simulation."""
    # Get package directories
    bringup_share = get_package_share_directory('flyby_f11_bringup')
    sensors_share = get_package_share_directory('flyby_f11_sensors')

    # Get project root for simulation assets
    # Assumes standard flyby-f11 project structure
    project_root = os.path.dirname(os.path.dirname(os.path.dirname(bringup_share)))
    simulation_dir = os.path.join(project_root, 'simulation')
    worlds_dir = os.path.join(simulation_dir, 'worlds')
    models_dir = os.path.join(simulation_dir, 'models')

    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    headless = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo in headless mode'
    )

    world = DeclareLaunchArgument(
        'world',
        default_value='flyby_training.sdf',
        description='Gazebo world file'
    )

    spawn_model = DeclareLaunchArgument(
        'spawn_model',
        default_value='true',
        description='Spawn F-11 model in Gazebo'
    )

    model_name = DeclareLaunchArgument(
        'model_name',
        default_value='f11_isr_camera',
        description='F-11 model variant to spawn'
    )

    spawn_x = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_z = DeclareLaunchArgument('spawn_z', default_value='0.25')

    # Set Gazebo resource paths
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    gz_resource_path = f"{models_dir}:{gz_resource_path}"

    # Gazebo simulation command
    gz_args = ['-r']  # Run immediately
    gz_cmd = ['gz', 'sim'] + gz_args + [
        PathJoinSubstitution([worlds_dir, LaunchConfiguration('world')])
    ]

    gazebo = ExecuteProcess(
        cmd=gz_cmd,
        output='screen',
        additional_env={'GZ_SIM_RESOURCE_PATH': gz_resource_path},
    )

    # Spawn F-11 model (delayed to let Gazebo start)
    spawn_entity = TimerAction(
        period=3.0,  # Wait for Gazebo to initialize
        actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/flyby_training/create',
                    '--reqtype', 'gz.msgs.EntityFactory',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req',
                    PythonExpression([
                        '"sdf_filename: \\"',
                        LaunchConfiguration('model_name'),
                        '\\" pose: {position: {x: ',
                        LaunchConfiguration('spawn_x'),
                        ', y: ',
                        LaunchConfiguration('spawn_y'),
                        ', z: ',
                        LaunchConfiguration('spawn_z'),
                        '}}"'
                    ])
                ],
                output='screen',
                condition=IfCondition(LaunchConfiguration('spawn_model'))
            )
        ]
    )

    # Gazebo-ROS bridge for sensor data
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            # Camera image
            '/f11_isr/camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            # IMU
            '/world/flyby_training/model/f11_isr_camera/link/imu_link/sensor/imu_sensor/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            # GPS
            '/world/flyby_training/model/f11_isr_camera/link/gps_link/sensor/gps_sensor/navsat@sensor_msgs/msg/NavSatFix@gz.msgs.NavSat',
            # Gimbal joints (ROS to Gazebo)
            '/f11_isr/gimbal/yaw@std_msgs/msg/Float64@gz.msgs.Double',
            '/f11_isr/gimbal/pitch@std_msgs/msg/Float64@gz.msgs.Double',
            '/f11_isr/gimbal/roll@std_msgs/msg/Float64@gz.msgs.Double',
            # Clock
            '/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock',
        ],
    )

    # Include ISR sensor nodes
    isr_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sensors_share, 'launch', 'isr_camera.launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items()
    )

    return LaunchDescription([
        # Launch arguments
        use_sim_time,
        headless,
        world,
        spawn_model,
        model_name,
        spawn_x,
        spawn_y,
        spawn_z,

        # Start Gazebo
        gazebo,

        # Spawn F-11 model
        spawn_entity,

        # Start Gazebo-ROS bridge (delayed to let model spawn)
        TimerAction(period=5.0, actions=[gz_bridge]),

        # Start sensor nodes (delayed to let bridges initialize)
        TimerAction(period=6.0, actions=[isr_sensors]),
    ])
