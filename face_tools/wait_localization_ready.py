#!/usr/bin/env python3
import argparse
import json
import os
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class LocalizationReadyWaiter(Node):
    def __init__(self, args):
        super().__init__("wait_localization_ready")
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_amcl_pose = None
        self.last_amcl_pose_ts = 0.0
        self.create_subscription(PoseWithCovarianceStamped, args.amcl_pose_topic, self.on_amcl_pose, 10)

    def on_amcl_pose(self, msg):
        self.last_amcl_pose = msg
        self.last_amcl_pose_ts = time.time()

    def is_transform_ready(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                self.args.map_frame,
                self.args.base_frame,
                rclpy.time.Time(),
            )
            return {
                "ok": True,
                "x": round(float(trans.transform.translation.x), 3),
                "y": round(float(trans.transform.translation.y), 3),
                "z": round(float(trans.transform.translation.z), 3),
                "frame": f"{self.args.map_frame}->{self.args.base_frame}",
            }
        except TransformException as exc:
            return {"ok": False, "error": str(exc)}

    def is_amcl_pose_ready(self):
        if self.last_amcl_pose is None:
            return {"ok": False, "error": "no amcl pose yet"}
        age = time.time() - self.last_amcl_pose_ts
        if age > self.args.pose_stale_timeout:
            return {"ok": False, "error": f"amcl pose stale: {age:.2f}s"}
        pose = self.last_amcl_pose.pose.pose
        return {
            "ok": True,
            "x": round(float(pose.position.x), 3),
            "y": round(float(pose.position.y), 3),
            "qx": round(float(pose.orientation.x), 4),
            "qy": round(float(pose.orientation.y), 4),
            "qz": round(float(pose.orientation.z), 4),
            "qw": round(float(pose.orientation.w), 4),
        }


def parse_args():
    ap = argparse.ArgumentParser(description="Wait until AMCL pose and map->base transform are ready.")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--amcl-pose-topic", default="/amcl_pose")
    ap.add_argument("--timeout", type=float, default=25.0)
    ap.add_argument("--check-interval", type=float, default=0.2)
    ap.add_argument("--pose-stale-timeout", type=float, default=2.0)
    ap.add_argument("--republish-interval", type=float, default=6.0)
    ap.add_argument("--publish-script", default="/root/face_tools/publish_initial_pose.py")
    ap.add_argument("--publish-python", default="/usr/bin/python3")
    ap.add_argument("--publish-wait-subs-timeout", type=float, default=12.0)
    ap.add_argument("--x", type=float, default=None)
    ap.add_argument("--y", type=float, default=None)
    ap.add_argument("--yaw", type=float, default=None)
    return ap.parse_args()


def maybe_publish_initial_pose(args):
    if args.x is None or args.y is None or args.yaw is None:
        return {"ok": False, "skipped": True}
    if not os.path.exists(args.publish_script):
        return {"ok": False, "error": f"publish script not found: {args.publish_script}"}

    cmd = [
        args.publish_python,
        args.publish_script,
        "--x",
        str(args.x),
        "--y",
        str(args.y),
        "--yaw",
        str(args.yaw),
        "--wait-subs-timeout",
        str(args.publish_wait_subs_timeout),
    ]
    try:
        completed = subprocess.run(cmd, check=True, capture_output=True, text=True)
        return {
            "ok": True,
            "cmd": cmd,
            "stdout": completed.stdout.strip(),
            "stderr": completed.stderr.strip(),
        }
    except subprocess.CalledProcessError as exc:
        return {
            "ok": False,
            "cmd": cmd,
            "returncode": exc.returncode,
            "stdout": (exc.stdout or "").strip(),
            "stderr": (exc.stderr or "").strip(),
        }


def main():
    args = parse_args()
    rclpy.init()
    node = LocalizationReadyWaiter(args)
    deadline = time.time() + args.timeout
    last_publish_ts = 0.0
    publish_attempts = 0
    last_publish_result = None
    result = {"overall_ok": False, "transform": None, "amcl_pose": None, "publish_attempts": 0}

    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=args.check_interval)

            now = time.time()
            if (
                args.x is not None
                and args.y is not None
                and args.yaw is not None
                and (publish_attempts == 0 or (now - last_publish_ts) >= args.republish_interval)
            ):
                last_publish_result = maybe_publish_initial_pose(args)
                last_publish_ts = now
                publish_attempts += 1

            tf_result = node.is_transform_ready()
            pose_result = node.is_amcl_pose_ready()
            result = {
                "overall_ok": bool(tf_result.get("ok")) and bool(pose_result.get("ok")),
                "transform": tf_result,
                "amcl_pose": pose_result,
                "publish_attempts": publish_attempts,
                "last_publish_result": last_publish_result,
            }
            if result["overall_ok"]:
                print(json.dumps(result, ensure_ascii=False))
                return

        print(json.dumps(result, ensure_ascii=False))
        raise SystemExit(2)
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
