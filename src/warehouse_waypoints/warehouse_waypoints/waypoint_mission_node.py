#!/usr/bin/env python3
"""
Autonomous warehouse waypoint mission.

- Publishes a visualization_msgs/MarkerArray on /waypoint_markers with one
  sphere + text label per named location (Home, Loading, Storage, Shipping).
- The currently active Nav2 goal is GREEN, every other marker stays BLUE.
- Runs the mission: Home -> Loading (wait 30s) -> Storage -> Shipping -> Home.
- Waits for each Nav2 result before sending the next goal.
- If any goal fails, the mission stops immediately and the failed
  location is reported.
"""

import math
import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from visualization_msgs.msg import Marker, MarkerArray


def yaw_to_quaternion(yaw_deg):
    yaw = math.radians(yaw_deg)
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def load_waypoints(navigator):
    # Prefer an installed share-dir copy; fall back to the source config/
    # folder so the node also works from a plain `python3` run.
    try:
        share_dir = get_package_share_directory('warehouse_waypoints')
        yaml_path = os.path.join(share_dir, 'config', 'waypoints.yaml')
        if not os.path.exists(yaml_path):
            raise FileNotFoundError
    except Exception:
        yaml_path = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            'config', 'waypoints.yaml')

    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    poses = {}
    for name, wp in data['waypoints'].items():
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.pose.position.x = float(wp['x'])
        ps.pose.position.y = float(wp['y'])
        ps.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(float(wp.get('yaw_deg', 0.0)))
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        poses[name] = {'pose': ps, 'label': wp.get('label', name)}

    mission_order = data.get('mission_order', ['loading', 'storage', 'shipping', 'home'])
    wait_at = data.get('wait_at', {})
    return poses, mission_order, wait_at


def build_marker_array(poses, active_name, clock):
    stamp = clock.now().to_msg()
    marker_array = MarkerArray()
    idx = 0
    for name, info in poses.items():
        is_active = (name == active_name)
        color = (0.0, 1.0, 0.0, 1.0) if is_active else (0.0, 0.3, 1.0, 1.0)

        sphere = Marker()
        sphere.header.frame_id = 'map'
        sphere.header.stamp = stamp
        sphere.ns = 'waypoints'
        sphere.id = idx
        idx += 1
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose = info['pose'].pose
        sphere.pose.position.z = 0.25
        sphere.scale.x = 0.35
        sphere.scale.y = 0.35
        sphere.scale.z = 0.35
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = color
        marker_array.markers.append(sphere)

        text = Marker()
        text.header.frame_id = 'map'
        text.header.stamp = stamp
        text.ns = 'waypoint_labels'
        text.id = idx
        idx += 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = info['pose'].pose
        text.pose.position.z = 0.7
        text.scale.z = 0.3
        text.color.r, text.color.g, text.color.b, text.color.a = (1.0, 1.0, 1.0, 1.0)
        text.text = info['label']
        marker_array.markers.append(text)

    return marker_array


def main():
    rclpy.init()
    navigator = BasicNavigator()

    poses, mission_order, wait_at = load_waypoints(navigator)

    marker_pub = navigator.create_publisher(MarkerArray, '/waypoint_markers', 10)

    def publish_markers(active_name):
        marker_array = build_marker_array(poses, active_name, navigator.get_clock())
        marker_pub.publish(marker_array)

    navigator.get_logger().info('Waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active()

    # Show all waypoints with none active yet, mission starts at Home.
    publish_markers(None)
    time.sleep(1.0)

    navigator.get_logger().info('Mission started at Charging Station (Home).')

    for name in mission_order:
        if name not in poses:
            navigator.get_logger().error(f'Unknown waypoint "{name}" in mission_order, aborting.')
            break

        label = poses[name]['label']
        navigator.get_logger().info(f'Navigating to {label} ({name})...')
        publish_markers(name)

        goal_pose = poses[name]['pose']
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        navigator.goToPose(goal_pose)

        while not navigator.isTaskComplete():
            time.sleep(0.2)

        result = navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(f'Reached {label}.')
        else:
            status = {
                TaskResult.CANCELED: 'CANCELED',
                TaskResult.FAILED: 'FAILED',
            }.get(result, 'UNKNOWN')
            navigator.get_logger().error(
                f'Goal to {label} ({name}) did NOT succeed. Status: {status}. '
                f'Stopping mission and reporting failed location: {label}.'
            )
            break
        wait_seconds = wait_at.get(name)
        if wait_seconds:
            navigator.get_logger().info(f'Waiting {wait_seconds:.0f} seconds at {label}...')
            time.sleep(float(wait_seconds))
    else:
        # Loop completed fully without a break -> mission fully succeeded.
        publish_markers('home')
        navigator.get_logger().info('Mission complete: robot returned to Charging Station (Home).')

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
