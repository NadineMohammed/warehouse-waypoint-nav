from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    mission_node = Node(
        package='warehouse_waypoints',
        executable='waypoint_mission_node',
        name='waypoint_mission_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([mission_node])
