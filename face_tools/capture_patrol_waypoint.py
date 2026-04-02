#!/usr/bin/env python3
import argparse
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointCapture(Node):
    def __init__(self, args):
        super().__init__("capture_patrol_waypoint")
        self.args = args
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def lookup_pose(self):
        deadline = time.time() + float(self.args.timeout)
        last_exc = None
        while time.time() < deadline:
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.args.map_frame,
                    self.args.base_frame,
                    rclpy.time.Time(),
                )
                t = trans.transform.translation
                q = trans.transform.rotation
                yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
                return {
                    "x": round(float(t.x), 3),
                    "y": round(float(t.y), 3),
                    "yaw": round(float(yaw), 3),
                }
            except TransformException as exc:
                last_exc = exc
                rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"failed to get transform {self.args.map_frame}->{self.args.base_frame}: {last_exc}")


def load_waypoints(path: str):
    if not os.path.exists(path):
        return []
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, list):
        raise ValueError(f"waypoint file is not a JSON list: {path}")
    return data


def save_waypoints(path: str, items):
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(items, f, ensure_ascii=False, indent=2)
        f.write("\n")


def parse_args():
    ap = argparse.ArgumentParser(description="Capture current map pose and append it to a patrol waypoint JSON file.")
    ap.add_argument("--out", required=True, help="Output JSON file")
    ap.add_argument("--name", default="", help="Waypoint name")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--timeout", type=float, default=8.0)
    ap.add_argument("--replace", action="store_true", help="Overwrite output file with only this waypoint")
    return ap.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = WaypointCapture(args)
    try:
        pose = node.lookup_pose()
        name = args.name.strip() or f"wp_{int(time.time())}"
        item = {
            "name": name,
            "x": pose["x"],
            "y": pose["y"],
            "yaw": pose["yaw"],
        }
        if args.replace:
            items = [item]
        else:
            items = load_waypoints(args.out)
            items.append(item)
        save_waypoints(args.out, items)
        print(json.dumps({"saved": args.out, "waypoint": item}, ensure_ascii=False))
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
