#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String


# Must match the names used in waypoint_mission.py
WAYPOINTS = [
    {'name': 'Home',             'x': 0.000,  'y': 0.000,  'yaw': 0.000, 'wait': 0},
    {'name': 'Loading Station',  'x': 7.663,  'y': -5.210, 'yaw': -0.260, 'wait': 30},
    {'name': 'Storage Area',     'x': 14.302, 'y': -5.372, 'yaw': 0.127, 'wait': 0},
    {'name': 'Shipping Station', 'x': 20.626, 'y': 1.807,  'yaw': 1.392, 'wait': 0},
]

BLUE = (0.0, 0.4, 1.0)
GREEN = (0.0, 1.0, 0.0)


class WaypointMarkers(Node):
    def __init__(self):
        super().__init__('waypoint_markers')

        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self._pub = self.create_publisher(MarkerArray, '/waypoint_markers', qos)
        self._sub = self.create_subscription(
            String, '/mission_status', self.status_callback, 10
        )

        self.active_waypoint = 'Home'

        # publish once immediately so markers show up even before mission starts
        self.publish_markers()

        # also republish periodically as a safety net (e.g. late RViz subscribers)
        self._timer = self.create_timer(2.0, self.publish_markers)

    def status_callback(self, msg):
        if msg.data != self.active_waypoint:
            self.active_waypoint = msg.data
            self.get_logger().info(f'Active waypoint changed to: {self.active_waypoint}')
            self.publish_markers()

    def publish_markers(self):
        marker_array = MarkerArray()

        for i, wp in enumerate(WAYPOINTS):
            is_active = (wp['name'] == self.active_waypoint)
            color = GREEN if is_active else BLUE

            # Sphere marker for the location itself
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = self.get_clock().now().to_msg()
            sphere.ns = 'waypoints'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wp['x']
            sphere.pose.position.y = wp['y']
            sphere.pose.position.z = 0.15
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.3
            sphere.scale.y = 0.3
            sphere.scale.z = 0.3
            sphere.color.r, sphere.color.g, sphere.color.b = color
            sphere.color.a = 1.0
            marker_array.markers.append(sphere)

            # Text label above the sphere
            text = Marker()
            text.header.frame_id = 'map'
            text.header.stamp = self.get_clock().now().to_msg()
            text.ns = 'waypoint_labels'
            text.id = i + 100
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = wp['x']
            text.pose.position.y = wp['y']
            text.pose.position.z = 0.6
            text.pose.orientation.w = 1.0
            text.scale.z = 0.3
            text.color.r, text.color.g, text.color.b = color
            text.color.a = 1.0
            text.text = wp['name']
            marker_array.markers.append(text)

        self._pub.publish(marker_array)


def main():
    rclpy.init()
    node = WaypointMarkers()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
