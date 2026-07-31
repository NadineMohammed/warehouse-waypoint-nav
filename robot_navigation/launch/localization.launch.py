import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    map_yaml = LaunchConfiguration('map')

    amcl_params = os.path.join(
        get_package_share_directory('robot_navigation'), 'config', 'amcl.yaml')

    default_map = os.path.join(
        get_package_share_directory('robot_navigation'), 'maps', 'warehouse_map.yaml')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}]
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params, {'use_sim_time': use_sim_time}]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'bond_timeout': 30.0,
            'node_names': ['map_server', 'amcl']
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='True'),
        DeclareLaunchArgument('map', default_value=default_map,
                               description='Full path to the map yaml'),
        map_server,
        amcl,
        lifecycle_manager,
    ])