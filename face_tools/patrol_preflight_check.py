#!/usr/bin/env python3
import argparse
import ast
import json
import os
import time
from pathlib import Path

import rclpy
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from rclpy.qos import qos_profile_sensor_data


class PatrolPreflight(Node):
    def __init__(self, args):
        super().__init__("patrol_preflight_check")
        self.args = args
        self.last_scan = None
        self.last_scan_ts = 0.0
        self.create_subscription(LaserScan, args.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_client = ActionClient(self, NavigateToPose, args.navigate_action)

    def on_scan(self, msg):
        self.last_scan = msg
        self.last_scan_ts = time.time()

    def wait_scan(self):
        deadline = time.time() + self.args.timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_scan is not None and (time.time() - self.last_scan_ts) <= self.args.scan_stale_timeout:
                msg = self.last_scan
                finite = [float(r) for r in msg.ranges if r == r and r > max(0.02, float(msg.range_min))]
                return {
                    "ok": True,
                    "frame": msg.header.frame_id,
                    "range_count": len(msg.ranges),
                    "min_range": min(finite) if finite else None,
                }
        return {"ok": False, "error": "no fresh /scan received"}

    def check_tf(self):
        deadline = time.time() + self.args.timeout
        last_exc = None
        while time.time() < deadline:
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
                last_exc = exc
                rclpy.spin_once(self, timeout_sec=0.1)
        return {"ok": False, "error": str(last_exc)}

    def check_nav_action(self):
        ok = self.nav_client.wait_for_server(timeout_sec=self.args.timeout)
        if ok:
            return {"ok": True}
        return {"ok": False, "error": "navigate_to_pose action server unavailable"}


def parse_args():
    ap = argparse.ArgumentParser(description="Check scan, TF, Nav2 action server and waypoint file before patrol test.")
    ap.add_argument("--waypoints-file", default="")
    ap.add_argument("--map-yaml", default="")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--navigate-action", default="navigate_to_pose")
    ap.add_argument("--timeout", type=float, default=8.0)
    ap.add_argument("--scan-stale-timeout", type=float, default=1.0)
    return ap.parse_args()


def validate_waypoint_items(items):
    if not isinstance(items, list) or not items:
        return False, "waypoints file is empty or not a JSON list", 0
    for idx, item in enumerate(items):
        if isinstance(item, dict):
            try:
                float(item["x"])
                float(item["y"])
                float(item.get("yaw", 0.0))
            except Exception:
                return False, f"invalid dict waypoint at index {idx}: {item!r}", len(items)
            continue
        if isinstance(item, list) and len(item) >= 3:
            try:
                float(item[0])
                float(item[1])
                float(item[2])
            except Exception:
                return False, f"invalid list waypoint at index {idx}: {item!r}", len(items)
            continue
        return False, f"unsupported waypoint at index {idx}: {item!r}", len(items)
    return True, "", len(items)


def load_simple_map_yaml(path: str):
    info = {}
    with open(path, "r", encoding="utf-8") as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, value = line.split(":", 1)
            info[key.strip()] = value.strip()
    image = info.get("image", "").strip().strip("'\"")
    resolution = float(info["resolution"])
    origin = ast.literal_eval(info["origin"])
    negate = int(info.get("negate", "0"))
    occupied_thresh = float(info.get("occupied_thresh", "0.65"))
    free_thresh = float(info.get("free_thresh", "0.25"))
    return {
        "image": image,
        "resolution": resolution,
        "origin": origin,
        "negate": negate,
        "occupied_thresh": occupied_thresh,
        "free_thresh": free_thresh,
    }


def read_pnm_token(f):
    token = bytearray()
    in_comment = False
    while True:
        ch = f.read(1)
        if not ch:
            break
        if in_comment:
            if ch in b"\r\n":
                in_comment = False
            continue
        if ch == b"#":
            in_comment = True
            continue
        if ch.isspace():
            if token:
                break
            continue
        token.extend(ch)
    if not token:
        return None
    return bytes(token)


def read_pgm_size_and_pixel(path: str, px: int, py: int):
    with open(path, "rb") as f:
        magic = read_pnm_token(f)
        if magic not in (b"P5", b"P2"):
            raise ValueError(f"unsupported pgm format: {magic!r}")

        width = int(read_pnm_token(f))
        height = int(read_pnm_token(f))
        maxval = int(read_pnm_token(f))

        if px < 0 or py < 0 or px >= width or py >= height:
            return width, height, None

        if magic == b"P5":
            bpp = 1 if maxval < 256 else 2
            offset = py * width * bpp + px * bpp
            blob = f.read()
            if bpp == 1:
                value = blob[offset]
            else:
                value = int.from_bytes(blob[offset : offset + 2], "big")
            return width, height, value

        values = []
        rest = f.read().split()
        for token in rest:
            if token.startswith(b"#"):
                continue
            values.append(int(token))
        value = values[py * width + px]
        return width, height, value


def waypoint_cell_status(map_info, image_path: Path, waypoint):
    width = None
    height = None
    wx = float(waypoint["x"])
    wy = float(waypoint["y"])
    res = float(map_info["resolution"])
    origin = map_info["origin"]
    mx = int((wx - float(origin[0])) / res)
    my_from_bottom = int((wy - float(origin[1])) / res)

    width, height, _ = read_pgm_size_and_pixel(str(image_path), 0, 0)
    my = height - 1 - my_from_bottom
    width, height, value = read_pgm_size_and_pixel(str(image_path), mx, my)
    if value is None:
        return {
            "name": waypoint.get("name", ""),
            "x": wx,
            "y": wy,
            "status": "out_of_bounds",
            "pixel": [mx, my],
        }

    occ = float(value) / 255.0 if int(map_info["negate"]) else (255.0 - float(value)) / 255.0
    if occ >= float(map_info["occupied_thresh"]):
        status = "occupied"
    elif occ <= float(map_info["free_thresh"]):
        status = "free"
    else:
        status = "unknown"
    return {
        "name": waypoint.get("name", ""),
        "x": wx,
        "y": wy,
        "status": status,
        "pixel": [mx, my],
        "occupancy": round(occ, 3),
    }


def validate_waypoints_against_map(waypoints_file: str, map_yaml: str):
    if not map_yaml:
        return {"ok": True, "skipped": True}
    if not os.path.exists(map_yaml):
        return {"ok": False, "error": f"map yaml not found: {map_yaml}"}

    map_info = load_simple_map_yaml(map_yaml)
    map_yaml_path = Path(map_yaml)
    image_path = (map_yaml_path.parent / map_info["image"]).resolve()
    if not image_path.exists():
        return {"ok": False, "error": f"map image not found: {image_path}"}

    with open(waypoints_file, "r", encoding="utf-8") as f:
        items = json.load(f)

    rows = []
    bad = []
    for idx, item in enumerate(items):
        wp = item if isinstance(item, dict) else {"name": f"wp_{idx:02d}", "x": item[0], "y": item[1], "yaw": item[2]}
        row = waypoint_cell_status(map_info, image_path, wp)
        rows.append(row)
        if row["status"] in ("occupied", "out_of_bounds"):
            bad.append(row)

    out = {"ok": len(bad) == 0, "count": len(rows), "rows": rows}
    if bad:
        out["error"] = "one or more waypoints are occupied or outside the map"
    return out


def main():
    args = parse_args()
    result = {
        "waypoints": {"ok": True},
        "map_waypoints": {"ok": True},
        "scan": None,
        "tf": None,
        "nav_action": None,
        "overall_ok": False,
    }

    if args.waypoints_file:
        if not os.path.exists(args.waypoints_file):
            result["waypoints"] = {"ok": False, "error": f"waypoints file not found: {args.waypoints_file}"}
        else:
            try:
                with open(args.waypoints_file, "r", encoding="utf-8") as f:
                    data = json.load(f)
                ok, error, count = validate_waypoint_items(data)
                result["waypoints"] = {"ok": ok, "count": count}
                if not ok:
                    result["waypoints"]["error"] = error
            except Exception as exc:
                result["waypoints"] = {"ok": False, "error": str(exc)}

        if result["waypoints"].get("ok", False) and args.map_yaml:
            try:
                result["map_waypoints"] = validate_waypoints_against_map(args.waypoints_file, args.map_yaml)
            except Exception as exc:
                result["map_waypoints"] = {"ok": False, "error": str(exc)}

    rclpy.init()
    node = PatrolPreflight(args)
    try:
        result["scan"] = node.wait_scan()
        result["tf"] = node.check_tf()
        result["nav_action"] = node.check_nav_action()
        result["overall_ok"] = all(
            item.get("ok", False)
            for item in [result["waypoints"], result["map_waypoints"], result["scan"], result["tf"], result["nav_action"]]
        )
        print(json.dumps(result, ensure_ascii=False))
        if not result["overall_ok"]:
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
