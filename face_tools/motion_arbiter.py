#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


MODE_SAFE_STOP = "SAFE_STOP"
MODE_FALLBACK_LOCAL = "FALLBACK_LOCAL"
MODE_TRACK_INTRUDER = "TRACK_INTRUDER"
MODE_RETURN_TO_ROUTE = "RETURN_TO_ROUTE"
MODE_PATROL = "PATROL"
MODE_SUSPECT_CONFIRM = "SUSPECT_CONFIRM"


def monotonic_ms() -> int:
    return time.perf_counter_ns() // 1_000_000


def make_neutral(speed: float, angle: float) -> Twist:
    msg = Twist()
    msg.linear.x = float(speed)
    msg.angular.z = float(angle)
    return msg


class MotionArbiter(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("motion_arbiter")
        self.args = args
        self.pub = self.create_publisher(Twist, args.output_topic, 10)
        self.create_subscription(Twist, args.nav_topic, self.on_nav_cmd, 10)
        self.create_subscription(Twist, args.follow_topic, self.on_follow_cmd, 10)
        self.create_subscription(Twist, args.fallback_topic, self.on_fallback_cmd, 10)
        self.create_subscription(String, args.mode_topic, self.on_mode, 10)

        self.nav_cmd = make_neutral(args.neutral_speed, args.neutral_angle)
        self.follow_cmd = make_neutral(args.neutral_speed, args.neutral_angle)
        self.fallback_cmd = make_neutral(args.neutral_speed, args.neutral_angle)
        self.mode = MODE_SAFE_STOP
        self.last_mode_update_ms = 0
        self.last_follow_update_ms = 0
        self.last_nav_update_ms = 0
        self.last_fallback_update_ms = 0
        self.last_state_file_update_ms = 0
        self.lock_until_ms = 0
        self.last_published = None
        self.timer = self.create_timer(1.0 / max(1.0, float(args.rate)), self.on_timer)
        self.get_logger().info(
            f"motion arbiter started: output={args.output_topic}, mode_topic={args.mode_topic}, "
            f"state_file={args.state_file}, bridge_status={args.bridge_status_file}"
        )

    def on_nav_cmd(self, msg: Twist) -> None:
        self.nav_cmd = msg
        self.last_nav_update_ms = monotonic_ms()

    def on_follow_cmd(self, msg: Twist) -> None:
        self.follow_cmd = msg
        self.last_follow_update_ms = monotonic_ms()

    def on_fallback_cmd(self, msg: Twist) -> None:
        self.fallback_cmd = msg
        self.last_fallback_update_ms = monotonic_ms()

    def on_mode(self, msg: String) -> None:
        raw = str(msg.data or "").strip()
        mode = raw
        if raw.startswith("{"):
            try:
                mode = str(json.loads(raw).get("mode", raw))
            except Exception:
                mode = raw
        mode = mode.strip() or MODE_SAFE_STOP
        if mode != self.mode:
            self.lock_until_ms = monotonic_ms() + int(self.args.switch_lock_ms)
        self.mode = mode
        self.last_mode_update_ms = monotonic_ms()

    def load_json(self, path: str) -> dict | None:
        if not path or not os.path.exists(path):
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return None

    def current_pc_state(self, now_ms: int) -> tuple[dict | None, dict | None]:
        state = self.load_json(self.args.state_file)
        if state is not None:
            self.last_state_file_update_ms = now_ms
        bridge = self.load_json(self.args.bridge_status_file)
        return state, bridge

    def guard_mode_from_bridge(self, bridge: dict | None) -> str | None:
        if bridge is None:
            return MODE_SAFE_STOP
        age_ms = bridge.get("message_age_ms")
        fresh = bool(bridge.get("fresh", False))
        if fresh:
            return None
        if age_ms is not None and int(age_ms) > int(self.args.fallback_timeout_ms):
            return MODE_FALLBACK_LOCAL
        return MODE_SAFE_STOP

    def requested_mode(self, now_ms: int, state: dict | None) -> str:
        # Patrol supervisor is the preferred authority when present.
        if self.last_mode_update_ms > 0 and (now_ms - self.last_mode_update_ms) <= int(self.args.mode_timeout_ms):
            return self.mode or MODE_SAFE_STOP
        # Follow-only PC LED mode has no supervisor; fall back to PC state mode.
        if state is not None and bool(state.get("fresh", False)):
            return str(state.get("mode", MODE_SAFE_STOP))
        return MODE_SAFE_STOP

    def publish_twist(self, msg: Twist) -> None:
        self.pub.publish(msg)
        self.last_published = (float(msg.linear.x), float(msg.angular.z))

    def publish_neutral(self) -> None:
        self.publish_twist(make_neutral(self.args.neutral_speed, self.args.neutral_angle))

    def choose_active_cmd(self, now_ms: int) -> Twist:
        state, bridge = self.current_pc_state(now_ms)
        guard_mode = self.guard_mode_from_bridge(bridge)
        effective_mode = guard_mode or self.requested_mode(now_ms, state)

        if effective_mode in {MODE_SAFE_STOP, ""}:
            return make_neutral(self.args.neutral_speed, self.args.neutral_angle)

        if effective_mode == MODE_FALLBACK_LOCAL:
            if (now_ms - self.last_fallback_update_ms) <= int(self.args.cmd_timeout_ms):
                return self.fallback_cmd
            return make_neutral(self.args.neutral_speed, self.args.neutral_angle)

        if effective_mode == MODE_SUSPECT_CONFIRM:
            return make_neutral(self.args.neutral_speed, self.args.neutral_angle)

        if effective_mode == MODE_TRACK_INTRUDER:
            if state is None or not bool(state.get("fresh", False)) or not bool(state.get("active", False)):
                return make_neutral(self.args.neutral_speed, self.args.neutral_angle)
            if (now_ms - self.last_follow_update_ms) <= int(self.args.cmd_timeout_ms):
                return self.follow_cmd
            return make_neutral(self.args.neutral_speed, self.args.neutral_angle)

        if effective_mode in {MODE_PATROL, MODE_RETURN_TO_ROUTE}:
            if (now_ms - self.last_nav_update_ms) <= int(self.args.cmd_timeout_ms):
                return self.nav_cmd
            return make_neutral(self.args.neutral_speed, self.args.neutral_angle)

        return make_neutral(self.args.neutral_speed, self.args.neutral_angle)

    def on_timer(self) -> None:
        now_ms = monotonic_ms()
        if now_ms < self.lock_until_ms:
            self.publish_neutral()
            return
        self.publish_twist(self.choose_active_cmd(now_ms))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Single cmd_vel publisher and motion arbiter for PC-led follow mode.")
    parser.add_argument("--output-topic", default="/car_cmd_vel")
    parser.add_argument("--nav-topic", default="/nav_cmd_vel")
    parser.add_argument("--follow-topic", default="/security/follow_cmd_vel")
    parser.add_argument("--fallback-topic", default="/security/fallback_cmd_vel")
    parser.add_argument("--mode-topic", default="/security/motion_mode")
    parser.add_argument("--state-file", default="/tmp/pc_vision_state.json")
    parser.add_argument("--bridge-status-file", default="/tmp/pc_vision_bridge_status.json")
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--cmd-timeout-ms", type=int, default=300)
    parser.add_argument("--mode-timeout-ms", type=int, default=1000)
    parser.add_argument("--fallback-timeout-ms", type=int, default=2000)
    parser.add_argument("--switch-lock-ms", type=int, default=500)
    parser.add_argument("--neutral-speed", type=float, default=1500.0)
    parser.add_argument("--neutral-angle", type=float, default=90.0)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    node = MotionArbiter(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_neutral()
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
