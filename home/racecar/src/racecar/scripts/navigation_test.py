#!/usr/bin/env python3
import json
import math
import os
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def yaw_to_quaternion(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


class NavigationTest(Node):
    def __init__(self):
        super().__init__("navigation_test")
        self.declare_parameter("test_points", [[2.0, 0.0, 0.0]])
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("wait_time", 3.0)
        self.declare_parameter("nav_server_timeout", 20.0)
        self.declare_parameter("publish_initial_pose", True)
        self.declare_parameter("initial_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("initial_cov_xy", 0.1)
        self.declare_parameter("initial_cov_yaw", 0.2)

        self.waypoints_file = str(self.get_parameter("waypoints_file").value).strip()
        self.test_points = self.load_test_points()
        self.wait_time = float(self.get_parameter("wait_time").value)
        self.nav_server_timeout = float(self.get_parameter("nav_server_timeout").value)
        self.publish_initial_pose_enabled = bool(self.get_parameter("publish_initial_pose").value)
        self.initial_pose = self.get_parameter("initial_pose").value
        self.initial_cov_xy = float(self.get_parameter("initial_cov_xy").value)
        self.initial_cov_yaw = float(self.get_parameter("initial_cov_yaw").value)

        qos_profile = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos_profile)
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.current_point_index = 0
        self.delay_timer = self.create_timer(2.0, self.start_navigation_test)
        self.get_logger().info("navigation_test started, points=%d" % len(self.test_points))

    def load_test_points(self):
        if self.waypoints_file:
            if not os.path.exists(self.waypoints_file):
                raise FileNotFoundError("waypoints_file not found: %s" % self.waypoints_file)
            with open(self.waypoints_file, "r", encoding="utf-8") as f:
                data = json.load(f)
            if not isinstance(data, list) or not data:
                raise ValueError("waypoints_file must be a non-empty JSON list")
            points = []
            for idx, item in enumerate(data):
                if isinstance(item, dict):
                    points.append([float(item["x"]), float(item["y"]), float(item.get("yaw", 0.0))])
                elif isinstance(item, list) and len(item) >= 3:
                    points.append([float(item[0]), float(item[1]), float(item[2])])
                else:
                    raise ValueError("invalid waypoint at index %d: %r" % (idx, item))
            self.get_logger().info("loaded points from %s" % self.waypoints_file)
            return points
        return self.get_parameter("test_points").value

    def schedule_once(self, delay_sec: float, callback):
        holder = {"timer": None}

        def wrapped():
            if holder["timer"] is not None:
                holder["timer"].cancel()
            callback()

        holder["timer"] = self.create_timer(delay_sec, wrapped)
        return holder["timer"]

    def start_navigation_test(self):
        self.delay_timer.cancel()
        if self.publish_initial_pose_enabled:
            init = self.initial_pose
            if not isinstance(init, list) or len(init) < 3:
                self.get_logger().error("initial_pose must be [x, y, yaw]")
                return
            self.publish_initial_pose(float(init[0]), float(init[1]), float(init[2]))
            self.schedule_once(2.0, self.navigate_to_next_point)
            return
        self.navigate_to_next_point()

    def publish_initial_pose(self, x: float, y: float, yaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(yaw)
        cov = [0.0] * 36
        cov[0] = self.initial_cov_xy
        cov[7] = self.initial_cov_xy
        cov[35] = self.initial_cov_yaw
        msg.pose.covariance = cov
        deadline = time.time() + 3.0
        while self.initial_pose_pub.get_subscription_count() <= 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        for _ in range(3):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(
            "published initial pose x=%.3f y=%.3f yaw=%.3f subs=%d"
            % (x, y, yaw, self.initial_pose_pub.get_subscription_count())
        )

    def navigate_to_next_point(self):
        if self.current_point_index >= len(self.test_points):
            self.get_logger().info("navigation_test finished")
            return

        point = self.test_points[self.current_point_index]
        if not isinstance(point, list) or len(point) < 3:
            self.get_logger().error("invalid point at index %d: %r" % (self.current_point_index, point))
            return

        if not self.nav_client.wait_for_server(timeout_sec=self.nav_server_timeout):
            self.get_logger().error("navigate_to_pose action server unavailable")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(point[0])
        goal_msg.pose.pose.position.y = float(point[1])
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation = yaw_to_quaternion(float(point[2]))

        self.get_logger().info(
            "send goal %d/%d -> x=%.3f y=%.3f yaw=%.3f"
            % (
                self.current_point_index + 1,
                len(self.test_points),
                float(point[0]),
                float(point[1]),
                float(point[2]),
            )
        )
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error("goal send failed: %s" % exc)
            return

        if not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return

        self.get_logger().info("goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:
            self.get_logger().error("goal result failed: %s" % exc)
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("goal %d reached" % (self.current_point_index + 1))
        else:
            self.get_logger().warning(
                "goal %d finished with status=%d" % (self.current_point_index + 1, status)
            )

        self.current_point_index += 1
        self.schedule_once(self.wait_time, self.navigate_to_next_point)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("navigation_test interrupted")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
