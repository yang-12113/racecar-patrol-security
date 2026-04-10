#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def yaw_to_quaternion(yaw: float):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    return qz, qw


class InitialPosePublisher(Node):
    def __init__(self, args):
        super().__init__("publish_initial_pose_once")
        self.args = args
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE
        self.pub = self.create_publisher(PoseWithCovarianceStamped, args.topic, qos)

    def make_msg(self) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame
        msg.pose.pose.position.x = float(self.args.x)
        msg.pose.pose.position.y = float(self.args.y)
        msg.pose.pose.position.z = 0.0
        qz, qw = yaw_to_quaternion(float(self.args.yaw))
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        cov = [0.0] * 36
        cov[0] = float(self.args.cov_xy)
        cov[7] = float(self.args.cov_xy)
        cov[35] = float(self.args.cov_yaw)
        msg.pose.covariance = cov
        return msg

    def publish_burst(self):
        deadline = time.time() + max(0.0, float(self.args.wait_subs_timeout))
        while self.pub.get_subscription_count() <= 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.05)

        msg = self.make_msg()
        for _ in range(max(1, int(self.args.repeat))):
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(float(self.args.gap))
        self.get_logger().info(
            "published initial pose: x=%.3f y=%.3f yaw=%.3f frame=%s subs=%d"
            % (self.args.x, self.args.y, self.args.yaw, self.args.frame, self.pub.get_subscription_count())
        )


def parse_args():
    ap = argparse.ArgumentParser(description="Publish /initialpose for Nav2/AMCL.")
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--yaw", type=float, required=True)
    ap.add_argument("--frame", default="map")
    ap.add_argument("--topic", default="/initialpose")
    ap.add_argument("--repeat", type=int, default=5)
    ap.add_argument("--gap", type=float, default=0.2)
    ap.add_argument("--cov-xy", type=float, default=0.1)
    ap.add_argument("--cov-yaw", type=float, default=0.2)
    ap.add_argument("--wait-subs-timeout", type=float, default=3.0)
    return ap.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = InitialPosePublisher(args)
    try:
        node.publish_burst()
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
