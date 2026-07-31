import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                             SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')

    # Charging Station (Home) is the mission's start/end pose. Update these
    # defaults once you know the exact pose you want the robot to start at.
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    tb3_model = 'burger'
    tb3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # ---------- 1. Warehouse world (Gazebo + world bridge) ----------
    warehouse_world_dir = get_package_share_directory('warehouse_world')

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(warehouse_world_dir, 'launch',
                         'warehouse_storage_launch.launch.py'))
    )
    # ---------- 2. Robot description (for TF / RViz visualization only) ----------
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_gazebo_dir, 'launch', 'robot_state_publisher.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # ---------- 3. Spawn the real SDF model (has DiffDrive + JointStatePublisher plugins) ----------
    model_sdf_path = os.path.join(
        tb3_gazebo_dir, 'models', f'turtlebot3_{tb3_model}', 'model.sdf')

    spawn_tb3 = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', tb3_model,
            '-file', model_sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
        ],
        output='screen'
    )

    # ---------- 4. Use turtlebot3_gazebo's own bridge config (matches the SDF plugin topics) ----------
    bridge_params = os.path.join(
        tb3_gazebo_dir, 'params', f'turtlebot3_{tb3_model}_bridge.yaml')

    tb3_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen'
    )

    # Spawn the robot 5 seconds after launch starts, so Gazebo is fully up
    # before we try to spawn into it.
    spawn_robot_group = TimerAction(
        period=5.0,
        actions=[
            robot_state_publisher,
            spawn_tb3,
            tb3_bridge,
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', tb3_model),
        world,
        spawn_robot_group,
    ])
