#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.duration import Duration

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


# ---- Behavior toggle -------------------------------------------------
# 'stop'  -> stop the whole mission and report which goal failed
# 'skip'  -> log the failure, skip that waypoint, continue to the next
ON_FAILURE = 'skip'
# -----------------------------------------------------------------------

WAYPOINTS = [
    {'name': 'Home',             'x': 0.000,  'y': 0.000,  'yaw': 0.000,  'wait': 1},
    {'name': 'Loading Station',  'x': 12.274, 'y': -0.140, 'yaw': -0.142, 'wait': 30},
    {'name': 'Storage Area',     'x': 13.133, 'y': 4.731,  'yaw': 0.975,  'wait': 1},
    {'name': 'Shipping Station', 'x': 5.650,  'y': 3.961,  'yaw': -2.962, 'wait': 1},
    {'name': 'Home',             'x': 0.000,  'y': 0.000,  'yaw': 0.000,  'wait': 1}
]


def yaw_to_quaternion(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class WaypointMission(Node):
    def __init__(self):
        super().__init__('waypoint_mission')
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._status_pub = self.create_publisher(String, '/mission_status', 10)
        self.failed_waypoints = []

    def publish_status(self, active_name):
        msg = String()
        msg.data = active_name
        self._status_pub.publish(msg)

    def send_goal(self, waypoint):
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        self._client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint['x']
        goal_msg.pose.pose.position.y = waypoint['y']
        qz, qw = yaw_to_quaternion(waypoint['yaw'])
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw

        self.get_logger().info(
            f"Sending goal: {waypoint['name']} "
            f"(x={waypoint['x']}, y={waypoint['y']}, yaw={waypoint['yaw']})"
        )

        # Tell the marker publisher this is now the active goal
        self.publish_status(waypoint['name'])

        send_future = self._client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"Goal to {waypoint['name']} was REJECTED by server")
            return False

        self.get_logger().info(f"Goal accepted, navigating to {waypoint['name']}...")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        # status code 4 = SUCCEEDED (per action_msgs/msg/GoalStatus)
        succeeded = (result.status == 4)

        if succeeded:
            self.get_logger().info(f"Reached {waypoint['name']} successfully.")
        else:
            self.get_logger().error(
                f"FAILED to reach {waypoint['name']} "
                f"(status code={result.status}, x={waypoint['x']}, y={waypoint['y']})"
            )
            self.failed_waypoints.append(waypoint['name'])

        return succeeded

    def feedback_callback(self, feedback_msg):
        remaining = feedback_msg.feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {remaining:.2f} m')

    def run_mission(self):
        self.get_logger().info('=== Starting Warehouse Waypoint Mission ===')

        for waypoint in WAYPOINTS:
            success = self.send_goal(waypoint)

            if not success:
                if ON_FAILURE == 'stop':
                    self.get_logger().error(
                        f"Mission STOPPED. Failed at: {waypoint['name']} "
                        f"(x={waypoint['x']}, y={waypoint['y']})"
                    )
                    self.publish_status('FAILED')
                    return
                else:  # 'skip'
                    self.get_logger().warn(
                        f"Skipping {waypoint['name']} and continuing to next waypoint."
                    )
                    continue

            # Wait at this waypoint if required (e.g. Loading Station -> 30s)
            if waypoint['wait'] > 0:
                self.get_logger().info(
                    f"Waiting {waypoint['wait']} seconds at {waypoint['name']}..."
                )
                time.sleep(waypoint['wait'])

        self.get_logger().info('=== Mission Complete ===')
        if self.failed_waypoints:
            self.get_logger().warn(
                f"Completed with failures at: {', '.join(self.failed_waypoints)}"
            )
        else:
            self.get_logger().info('All waypoints reached successfully.')

        self.publish_status('Home')  # mission ends back at Home = active marker


def main():
    rclpy.init()
    node = WaypointMission()
    node.run_mission()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
