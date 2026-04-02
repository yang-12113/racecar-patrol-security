#!/usr/bin/env python3
import argparse
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class FaceFollowController(Node):
    def __init__(self, args):
        super().__init__("face_follow_controller")
        self.args = args
        self.pub = self.create_publisher(Twist, "/car_cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, args.scan_topic, self.on_scan, 10)
        self.last_state = None
        self.last_log = 0.0
        self.last_publish = None
        self.last_scan_ts = 0.0
        self.last_scan_info = None
        self.last_obstacle_mode = ""
        self.timer = self.create_timer(1.0 / max(1.0, float(args.rate)), self.on_timer)
        self.get_logger().info(
            f"Face follow controller started: state_file={args.state_file}, "
            f"target_mode={args.target_mode}, target_name={args.target_name}, scan_topic={args.scan_topic}"
        )

    def publish_cmd(self, speed, angle):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(angle)
        self.pub.publish(msg)
        self.last_publish = (float(speed), float(angle))

    def publish_neutral(self):
        self.publish_cmd(self.args.neutral_speed, self.args.neutral_angle)

    def publish_neutral_burst(self, repeat=5, gap=0.05):
        for _ in range(max(1, int(repeat))):
            try:
                self.publish_neutral()
            except Exception:
                return
            if gap > 0:
                time.sleep(gap)

    def load_state(self):
        path = self.args.state_file
        if not os.path.exists(path):
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return None

    def sector_values(self, msg, start_deg, end_deg):
        start_rad = math.radians(start_deg)
        end_rad = math.radians(end_deg)
        vals = []
        angle = float(msg.angle_min)
        for r in msg.ranges:
            if start_rad <= angle <= end_rad and math.isfinite(r):
                rr = float(r)
                if rr >= max(0.02, float(msg.range_min)) and rr <= float(msg.range_max):
                    vals.append(rr)
            angle += float(msg.angle_increment)
        return vals

    def robust_clearance(self, values):
        if not values:
            return float("inf")
        clipped = sorted(min(v, self.args.obstacle_consider_range) for v in values)
        return float(clipped[len(clipped) // 2])

    def on_scan(self, msg):
        front = self.sector_values(msg, -self.args.obstacle_front_deg / 2.0, self.args.obstacle_front_deg / 2.0)
        left = self.sector_values(msg, self.args.obstacle_side_inner_deg, self.args.obstacle_side_outer_deg)
        right = self.sector_values(msg, -self.args.obstacle_side_outer_deg, -self.args.obstacle_side_inner_deg)
        self.last_scan_info = {
            "front_min": min(front) if front else float("inf"),
            "left_min": min(left) if left else float("inf"),
            "right_min": min(right) if right else float("inf"),
            "left_clear": self.robust_clearance(left),
            "right_clear": self.robust_clearance(right),
        }
        self.last_scan_ts = time.time()

    def compute_cmd(self, state):
        width = max(1, int(state.get("width", self.args.image_width)))
        cx = float(state.get("cx", width / 2.0))
        ex = (cx - width / 2.0) / max(1.0, width / 2.0)
        area_ratio = float(state.get("area_ratio", 0.0))

        if abs(ex) < self.args.deadband:
            ex = 0.0

        angle = self.args.neutral_angle + self.args.steer_sign * self.args.steer_gain * ex
        angle = clamp(angle, self.args.min_angle, self.args.max_angle)

        speed = self.args.neutral_speed
        if area_ratio < self.args.desired_area - self.args.area_tol:
            gap = (self.args.desired_area - self.args.area_tol) - area_ratio
            speed += self.args.speed_gain * gap / max(1e-6, self.args.desired_area)
        elif area_ratio > self.args.desired_area + self.args.area_tol:
            gap = area_ratio - (self.args.desired_area + self.args.area_tol)
            speed -= self.args.speed_gain * gap / max(1e-6, self.args.desired_area)

        if abs(ex) > self.args.turn_slow_band:
            speed -= self.args.turn_slow_down

        speed = clamp(speed, self.args.min_speed, self.args.max_speed)
        return float(speed), float(angle), float(ex), float(area_ratio)

    def apply_obstacle_response(self, speed, angle):
        if not self.args.obstacle_enabled:
            return float(speed), float(angle), None

        if (time.time() - self.last_scan_ts) > self.args.scan_timeout or not self.last_scan_info:
            if self.args.allow_stale_scan_motion:
                return float(speed), float(angle), {"mode": "scan_stale_ignore"}
            return float(self.args.neutral_speed), float(self.args.neutral_angle), {"mode": "scan_stale_stop"}

        info = self.last_scan_info
        front_min = float(info["front_min"])
        left_clear = float(info["left_clear"])
        right_clear = float(info["right_clear"])

        steer_bias = 0.0
        steer_dir = ""
        if left_clear > (right_clear + self.args.obstacle_clear_margin):
            steer_bias = -self.args.obstacle_steer_bias
            steer_dir = "left"
        elif right_clear > (left_clear + self.args.obstacle_clear_margin):
            steer_bias = self.args.obstacle_steer_bias
            steer_dir = "right"

        mode = "clear"
        out_speed = float(speed)
        out_angle = float(angle)

        if front_min <= self.args.obstacle_stop_range:
            if steer_dir and max(left_clear, right_clear) >= self.args.obstacle_turn_clearance:
                out_speed = min(out_speed, self.args.obstacle_avoid_speed)
                out_angle = clamp(self.args.neutral_angle + steer_bias, self.args.min_angle, self.args.max_angle)
                mode = f"avoid_{steer_dir}"
            else:
                out_speed = self.args.neutral_speed
                out_angle = self.args.neutral_angle
                mode = "blocked_stop"
        elif front_min <= self.args.obstacle_slow_range:
            out_speed = min(out_speed, self.args.obstacle_slow_speed)
            if steer_dir:
                out_angle = clamp(out_angle + steer_bias * 0.6, self.args.min_angle, self.args.max_angle)
                mode = f"slow_bias_{steer_dir}"
            else:
                mode = "slow"

        return float(out_speed), float(out_angle), {
            "mode": mode,
            "front_min": front_min,
            "left_clear": left_clear,
            "right_clear": right_clear,
        }

    def on_timer(self):
        now = time.time()
        state = self.load_state()
        if not state or not state.get("active", False):
            self.publish_neutral()
            return

        state_ts = float(state.get("ts", 0.0))
        if now - state_ts > self.args.timeout:
            self.publish_neutral()
            return

        name = str(state.get("name", "unknown"))
        if self.args.target_mode == "owner" and self.args.target_name and name != self.args.target_name:
            self.publish_neutral()
            return
        if self.args.target_mode == "unknown" and name != "unknown":
            self.publish_neutral()
            return

        speed, angle, ex, area_ratio = self.compute_cmd(state)
        speed, angle, obstacle_info = self.apply_obstacle_response(speed, angle)
        self.publish_cmd(speed, angle)

        if now - self.last_log >= 1.0:
            self.last_log = now
            obs_text = ""
            if obstacle_info:
                obs_mode = obstacle_info.get("mode", "")
                if obs_mode:
                    obs_text = (
                        f" obs={obs_mode} front={obstacle_info.get('front_min', float('inf')):.2f}"
                        f" left={obstacle_info.get('left_clear', float('inf')):.2f}"
                        f" right={obstacle_info.get('right_clear', float('inf')):.2f}"
                    )
                    self.last_obstacle_mode = obs_mode
            self.get_logger().info(
                f"follow name={name} id={state.get('track_id', -1)} ex={ex:.3f} area={area_ratio:.3f} "
                f"cmd=({speed:.1f}, {angle:.1f}){obs_text}"
            )


def parse_args():
    ap = argparse.ArgumentParser(description="Read face-follow state and publish /car_cmd_vel.")
    ap.add_argument("--state-file", default="/tmp/face_follow_state.json")
    ap.add_argument("--target-mode", default="owner", choices=["owner", "unknown", "any"])
    ap.add_argument("--target-name", default="owner")
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--timeout", type=float, default=1.0)
    ap.add_argument("--scan-timeout", type=float, default=0.8)
    ap.add_argument("--allow-stale-scan-motion", action="store_true")
    ap.add_argument("--image-width", type=int, default=640)
    ap.add_argument("--neutral-speed", type=float, default=1500.0)
    ap.add_argument("--neutral-angle", type=float, default=90.0)
    ap.add_argument("--min-speed", type=float, default=1512.0)
    ap.add_argument("--max-speed", type=float, default=1532.0)
    ap.add_argument("--speed-gain", type=float, default=55.0)
    ap.add_argument("--desired-area", type=float, default=0.16)
    ap.add_argument("--area-tol", type=float, default=0.03)
    ap.add_argument("--steer-gain", type=float, default=32.0)
    ap.add_argument("--steer-sign", type=float, default=1.0)
    ap.add_argument("--deadband", type=float, default=0.04)
    ap.add_argument("--turn-slow-band", type=float, default=0.20)
    ap.add_argument("--turn-slow-down", type=float, default=4.0)
    ap.add_argument("--min-angle", type=float, default=45.0)
    ap.add_argument("--max-angle", type=float, default=135.0)
    ap.add_argument("--disable-obstacle-avoidance", action="store_false", dest="obstacle_enabled")
    ap.set_defaults(obstacle_enabled=True)
    ap.add_argument("--obstacle-front-deg", type=float, default=40.0)
    ap.add_argument("--obstacle-side-inner-deg", type=float, default=25.0)
    ap.add_argument("--obstacle-side-outer-deg", type=float, default=75.0)
    ap.add_argument("--obstacle-stop-range", type=float, default=0.65)
    ap.add_argument("--obstacle-slow-range", type=float, default=1.00)
    ap.add_argument("--obstacle-turn-clearance", type=float, default=0.90)
    ap.add_argument("--obstacle-clear-margin", type=float, default=0.15)
    ap.add_argument("--obstacle-consider-range", type=float, default=2.0)
    ap.add_argument("--obstacle-slow-speed", type=float, default=1516.0)
    ap.add_argument("--obstacle-avoid-speed", type=float, default=1512.0)
    ap.add_argument("--obstacle-steer-bias", type=float, default=18.0)
    return ap.parse_args()


def main():
    args = parse_args()
    node = None
    rclpy.init()
    node = FaceFollowController(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                if rclpy.ok():
                    node.publish_neutral_burst()
            except Exception:
                pass
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
