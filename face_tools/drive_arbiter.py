#!/usr/bin/env python3
import argparse
import copy
import errno
import hashlib
import json
import os
import signal
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


ARBITER_SCHEMA_VERSION = 1

STATE_PATROL = "PATROL"
STATE_ALERT_TRACK = "ALERT_TRACK"
STATE_RETURN_TO_ROUTE = "RETURN_TO_ROUTE"
STATE_SAFE_STOP = "SAFE_STOP"
VALID_STATES = {STATE_PATROL, STATE_ALERT_TRACK, STATE_RETURN_TO_ROUTE, STATE_SAFE_STOP}

ACTION_OUTPUT_NEUTRAL = "OUTPUT_NEUTRAL"
ACTION_FORWARD_NAV = "FORWARD_NAV"
ACTION_FORWARD_TRACK = "FORWARD_TRACK"
VALID_ARBITER_ACTIONS = {ACTION_OUTPUT_NEUTRAL, ACTION_FORWARD_NAV, ACTION_FORWARD_TRACK}

REASON_NONE = "NONE"
REASON_STARTUP_LOCK = "STARTUP_LOCK"
REASON_MODE_SWITCH_LOCK = "MODE_SWITCH_LOCK"
REASON_SOURCE_SWITCH_LOCK = "SOURCE_SWITCH_LOCK"
REASON_SAFE_STOP_MODE = "SAFE_STOP_MODE"
REASON_MODE_TIMEOUT = "MODE_TIMEOUT"
REASON_NAV_TIMEOUT = "NAV_TIMEOUT"
REASON_TRACK_TIMEOUT = "TRACK_TIMEOUT"
REASON_INVALID_MODE = "INVALID_MODE"
REASON_NO_FRESH_NAV_CMD = "NO_FRESH_NAV_CMD"
REASON_NO_FRESH_TRACK_CMD = "NO_FRESH_TRACK_CMD"
REASON_PATROL_ACTIVE = "PATROL_ACTIVE"
REASON_RETURN_ACTIVE = "RETURN_ACTIVE"
REASON_TRACK_ACTIVE = "TRACK_ACTIVE"
REASON_UPSTREAM_EXIT = "UPSTREAM_EXIT"
REASON_MANUAL_STOP = "MANUAL_STOP"
VALID_ARBITER_REASONS = {
    REASON_NONE,
    REASON_STARTUP_LOCK,
    REASON_MODE_SWITCH_LOCK,
    REASON_SOURCE_SWITCH_LOCK,
    REASON_SAFE_STOP_MODE,
    REASON_MODE_TIMEOUT,
    REASON_NAV_TIMEOUT,
    REASON_TRACK_TIMEOUT,
    REASON_INVALID_MODE,
    REASON_NO_FRESH_NAV_CMD,
    REASON_NO_FRESH_TRACK_CMD,
    REASON_PATROL_ACTIVE,
    REASON_RETURN_ACTIVE,
    REASON_TRACK_ACTIVE,
    REASON_UPSTREAM_EXIT,
    REASON_MANUAL_STOP,
}


def monotonic_ms():
    return int(time.monotonic_ns() // 1_000_000)


def canonical_json_bytes(data):
    return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def apply_checksum(data):
    payload = copy.deepcopy(data)
    payload.setdefault("meta", {})
    payload["meta"]["checksum_sha256"] = ""
    payload["meta"]["checksum_sha256"] = hashlib.sha256(canonical_json_bytes(payload)).hexdigest()
    return payload


def verify_checksum(data):
    try:
        meta = data.get("meta", {})
        expected = str(meta.get("checksum_sha256", "")).strip()
        if not expected:
            return False
        payload = copy.deepcopy(data)
        payload.setdefault("meta", {})
        payload["meta"]["checksum_sha256"] = ""
        actual = hashlib.sha256(canonical_json_bytes(payload)).hexdigest()
        return actual == expected
    except Exception:
        return False


def atomic_write_json(path, data):
    if not path:
        return
    out_dir = os.path.dirname(path) or "."
    os.makedirs(out_dir, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".drive_arbiter_", suffix=".json", dir=out_dir)
    try:
        with os.fdopen(fd, "wb") as f:
            raw = canonical_json_bytes(apply_checksum(data))
            f.write(raw)
            f.flush()
            os.fsync(f.fileno())
        try:
            os.replace(tmp_path, path)
        except OSError as exc:
            if exc.errno == errno.EXDEV:
                raise RuntimeError(f"rename across filesystems is not allowed: tmp={tmp_path} dst={path}") from exc
            raise
    finally:
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except OSError:
                pass


def append_jsonl(path, data):
    if not path:
        return
    out_dir = os.path.dirname(path) or "."
    os.makedirs(out_dir, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(data, ensure_ascii=False, sort_keys=True) + "\n")


def make_qos_profile(name):
    value = str(name or "reliable").strip().lower()
    reliability = ReliabilityPolicy.RELIABLE
    if value in ("best_effort", "best-effort", "bestavailable", "best_available"):
        reliability = ReliabilityPolicy.BEST_EFFORT
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=reliability,
        durability=DurabilityPolicy.VOLATILE,
    )


class DriveArbiter(Node):
    def __init__(self, args):
        super().__init__("drive_arbiter")
        self.args = args
        control_qos = make_qos_profile(args.control_qos_profile)
        state_qos = make_qos_profile(args.state_qos_profile)

        self.pub = self.create_publisher(Twist, args.output_topic, control_qos)
        self.state_pub = self.create_publisher(String, args.state_topic, state_qos)
        self.nav_sub = self.create_subscription(Twist, args.nav_topic, self.on_nav_cmd, control_qos)
        self.track_sub = self.create_subscription(Twist, args.track_topic, self.on_track_cmd, control_qos)
        self.mode_sub = None
        if args.mode_topic:
            self.mode_sub = self.create_subscription(String, args.mode_topic, self.on_mode_msg, state_qos)

        self.last_nav_cmd = None
        self.last_nav_ts_ms = 0
        self.last_track_cmd = None
        self.last_track_ts_ms = 0

        self.mode_payload = None
        self.mode_status = "MISSING"
        self.mode_reason = REASON_NONE
        self.mode_update_ms = 0
        self.mode_seq = -1

        self.current_mode = str(args.default_mode)
        self.output_seq = 0
        self.last_signature = None
        self.lock_until_ms = monotonic_ms() + int(max(0.0, float(args.startup_lock)) * 1000.0)
        self.lock_reason = REASON_STARTUP_LOCK
        self.last_action = ACTION_OUTPUT_NEUTRAL
        self.last_reason = REASON_STARTUP_LOCK
        self.last_source_action = ACTION_OUTPUT_NEUTRAL
        self.timer = self.create_timer(1.0 / max(1.0, float(args.rate)), self.on_timer)
        self.publish_protocol_state(
            mode=self.current_mode,
            action=ACTION_OUTPUT_NEUTRAL,
            reason=REASON_STARTUP_LOCK,
            cmd=self.make_cmd(self.args.neutral_speed, self.args.neutral_angle),
        )
        self.get_logger().info(
            "drive arbiter started: output=%s nav=%s track=%s mode_topic=%s mode_state_file=%s state_topic=%s"
            % (
                args.output_topic,
                args.nav_topic,
                args.track_topic,
                args.mode_topic,
                args.mode_state_file,
                args.state_topic,
            )
        )

    def make_cmd(self, speed, angle):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(angle)
        return msg

    def publish_cmd(self, msg):
        self.pub.publish(msg)

    def publish_neutral(self):
        cmd = self.make_cmd(self.args.neutral_speed, self.args.neutral_angle)
        self.publish_cmd(cmd)
        return cmd

    def publish_neutral_burst(self, repeat=5, gap=0.05, reason=REASON_MANUAL_STOP):
        cmd = self.make_cmd(self.args.neutral_speed, self.args.neutral_angle)
        for _ in range(max(1, int(repeat))):
            self.publish_cmd(cmd)
            self.publish_protocol_state(
                mode=STATE_SAFE_STOP,
                action=ACTION_OUTPUT_NEUTRAL,
                reason=reason,
                cmd=cmd,
                force_event=True,
            )
            if gap > 0:
                time.sleep(gap)

    def on_nav_cmd(self, msg):
        self.last_nav_cmd = msg
        self.last_nav_ts_ms = monotonic_ms()

    def on_track_cmd(self, msg):
        self.last_track_cmd = msg
        self.last_track_ts_ms = monotonic_ms()

    def on_mode_msg(self, msg):
        now_ms = monotonic_ms()
        try:
            payload = json.loads(msg.data)
            schema_version = int(payload.get("schema_version", -1))
            if schema_version != ARBITER_SCHEMA_VERSION:
                raise ValueError(f"schema_version={schema_version}")
            seq = int(payload.get("seq", -1))
            mode = str(payload.get("mode", payload.get("state", ""))).strip()
            if mode not in VALID_STATES:
                raise ValueError(f"invalid mode {mode!r}")
            self.mode_payload = payload
            self.mode_status = "OK"
            self.mode_reason = str(payload.get("reason", REASON_NONE))
            self.mode_update_ms = now_ms
            self.mode_seq = seq
        except Exception as exc:
            self.mode_payload = None
            self.mode_status = "INVALID"
            self.mode_reason = REASON_INVALID_MODE
            self.mode_update_ms = now_ms
            self.get_logger().warning("invalid drive_mode payload: %s" % exc)

    def read_mode_file(self):
        path = self.args.mode_state_file
        if not path:
            return None, None
        if not os.path.exists(path):
            return None, "missing"
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception:
            return None, "invalid"

        if isinstance(data, dict) and "schema_version" in data and isinstance(data.get("meta"), dict):
            if int(data.get("schema_version", -1)) != ARBITER_SCHEMA_VERSION:
                return None, "invalid"
            if not verify_checksum(data):
                return None, "invalid"

        mode = str(data.get("mode", data.get("state", ""))).strip()
        if mode not in VALID_STATES:
            return None, "invalid"

        ts_ms = None
        if "ts_monotonic_ms" in data:
            ts_ms = int(data.get("ts_monotonic_ms", 0))
        elif isinstance(data.get("meta"), dict) and "write_ts_monotonic_ms" in data["meta"]:
            ts_ms = int(data["meta"].get("write_ts_monotonic_ms", 0))
        if ts_ms is None or ts_ms <= 0:
            return None, "invalid"
        return {"mode": mode, "ts_monotonic_ms": ts_ms}, "file"

    def age_ms(self, last_ms):
        if last_ms <= 0:
            return None
        return max(0, monotonic_ms() - int(last_ms))

    def enter_lock(self, reason):
        self.lock_reason = str(reason)
        self.lock_until_ms = monotonic_ms() + int(max(0.0, float(self.args.lock_duration)) * 1000.0)

    def resolve_mode(self):
        now_ms = monotonic_ms()
        if self.mode_sub is not None:
            age = None if self.mode_update_ms <= 0 else max(0, now_ms - self.mode_update_ms)
            if self.mode_status == "INVALID" and age is not None and age <= int(self.args.timeout_mode_ms):
                return STATE_SAFE_STOP, REASON_INVALID_MODE
            if self.mode_status == "OK" and self.mode_payload is not None and age is not None and age <= int(self.args.timeout_mode_ms):
                return str(self.mode_payload.get("mode", self.mode_payload.get("state", self.args.default_mode))), REASON_NONE
            file_payload, file_status = self.read_mode_file()
            if file_payload is not None:
                age = max(0, now_ms - int(file_payload["ts_monotonic_ms"]))
                if age <= int(self.args.timeout_mode_ms):
                    return str(file_payload["mode"]), REASON_NONE
            return STATE_SAFE_STOP, REASON_MODE_TIMEOUT

        file_payload, file_status = self.read_mode_file()
        if file_payload is not None:
            age = max(0, now_ms - int(file_payload["ts_monotonic_ms"]))
            if age <= int(self.args.timeout_mode_ms):
                return str(file_payload["mode"]), REASON_NONE
            return STATE_SAFE_STOP, REASON_MODE_TIMEOUT

        return self.args.default_mode, REASON_NONE

    def publish_protocol_state(self, mode, action, reason, cmd, force_event=False):
        self.output_seq += 1
        now_ms = monotonic_ms()
        nav_age_ms = self.age_ms(self.last_nav_ts_ms)
        track_age_ms = self.age_ms(self.last_track_ts_ms)
        payload = {
            "schema_version": ARBITER_SCHEMA_VERSION,
            "ts_monotonic_ms": int(now_ms),
            "seq": int(self.output_seq),
            "mode": str(mode),
            "arbiter_action": str(action),
            "arbiter_reason": str(reason),
            "lock_active": bool(now_ms < self.lock_until_ms),
            "lock_remaining_ms": max(0, int(self.lock_until_ms - now_ms)),
            "nav_age_ms": -1 if nav_age_ms is None else int(nav_age_ms),
            "track_age_ms": -1 if track_age_ms is None else int(track_age_ms),
            "cmd": {
                "linear_x": float(cmd.linear.x),
                "angular_z": float(cmd.angular.z),
            },
            "meta": {
                "checksum_sha256": "",
            },
        }
        payload = apply_checksum(payload)
        atomic_write_json(self.args.status_file, payload)

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, sort_keys=True)
        self.state_pub.publish(msg)

        signature = (
            payload["mode"],
            payload["arbiter_action"],
            payload["arbiter_reason"],
            payload["lock_active"],
        )
        if force_event or signature != self.last_signature:
            append_jsonl(
                self.args.event_log,
                {
                    "schema_version": ARBITER_SCHEMA_VERSION,
                    "ts_monotonic_ms": int(payload["ts_monotonic_ms"]),
                    "seq": int(payload["seq"]),
                    "mode": str(payload["mode"]),
                    "arbiter_action": str(payload["arbiter_action"]),
                    "arbiter_reason": str(payload["arbiter_reason"]),
                    "lock_active": bool(payload["lock_active"]),
                    "lock_remaining_ms": int(payload["lock_remaining_ms"]),
                    "nav_age_ms": int(payload["nav_age_ms"]),
                    "track_age_ms": int(payload["track_age_ms"]),
                    "cmd": payload["cmd"],
                },
            )
            self.last_signature = signature

        self.last_action = str(action)
        self.last_reason = str(reason)

    def on_timer(self):
        now_ms = monotonic_ms()
        desired_mode, mode_reason = self.resolve_mode()
        if desired_mode not in VALID_STATES:
            desired_mode = STATE_SAFE_STOP
            mode_reason = REASON_INVALID_MODE

        if desired_mode != self.current_mode:
            self.current_mode = desired_mode
            self.enter_lock(REASON_MODE_SWITCH_LOCK)

        if now_ms < self.lock_until_ms:
            cmd = self.publish_neutral()
            self.publish_protocol_state(
                mode=self.current_mode,
                action=ACTION_OUTPUT_NEUTRAL,
                reason=self.lock_reason,
                cmd=cmd,
            )
            return

        if mode_reason == REASON_MODE_TIMEOUT:
            self.current_mode = STATE_SAFE_STOP
            cmd = self.publish_neutral()
            self.publish_protocol_state(
                mode=self.current_mode,
                action=ACTION_OUTPUT_NEUTRAL,
                reason=REASON_MODE_TIMEOUT,
                cmd=cmd,
            )
            return

        if mode_reason == REASON_INVALID_MODE:
            self.current_mode = STATE_SAFE_STOP
            cmd = self.publish_neutral()
            self.publish_protocol_state(
                mode=self.current_mode,
                action=ACTION_OUTPUT_NEUTRAL,
                reason=REASON_INVALID_MODE,
                cmd=cmd,
            )
            return

        if self.current_mode == STATE_SAFE_STOP:
            cmd = self.publish_neutral()
            self.publish_protocol_state(
                mode=self.current_mode,
                action=ACTION_OUTPUT_NEUTRAL,
                reason=REASON_SAFE_STOP_MODE,
                cmd=cmd,
            )
            return

        if self.current_mode in (STATE_PATROL, STATE_RETURN_TO_ROUTE):
            nav_age = self.age_ms(self.last_nav_ts_ms)
            if self.last_nav_cmd is None:
                self.current_mode = STATE_SAFE_STOP
                cmd = self.publish_neutral()
                self.publish_protocol_state(
                    mode=self.current_mode,
                    action=ACTION_OUTPUT_NEUTRAL,
                    reason=REASON_NO_FRESH_NAV_CMD,
                    cmd=cmd,
                )
                return
            if nav_age is None or nav_age > int(self.args.timeout_nav_ms):
                self.current_mode = STATE_SAFE_STOP
                cmd = self.publish_neutral()
                self.publish_protocol_state(
                    mode=self.current_mode,
                    action=ACTION_OUTPUT_NEUTRAL,
                    reason=REASON_NAV_TIMEOUT,
                    cmd=cmd,
                )
                return
            action = ACTION_FORWARD_NAV
            reason = REASON_PATROL_ACTIVE if self.current_mode == STATE_PATROL else REASON_RETURN_ACTIVE
            self.publish_cmd(self.last_nav_cmd)
            self.publish_protocol_state(mode=self.current_mode, action=action, reason=reason, cmd=self.last_nav_cmd)
            self.last_source_action = action
            return

        if self.current_mode == STATE_ALERT_TRACK:
            track_age = self.age_ms(self.last_track_ts_ms)
            if self.last_track_cmd is None:
                self.current_mode = STATE_SAFE_STOP
                cmd = self.publish_neutral()
                self.publish_protocol_state(
                    mode=self.current_mode,
                    action=ACTION_OUTPUT_NEUTRAL,
                    reason=REASON_NO_FRESH_TRACK_CMD,
                    cmd=cmd,
                )
                return
            if track_age is None or track_age > int(self.args.timeout_track_ms):
                self.current_mode = STATE_SAFE_STOP
                cmd = self.publish_neutral()
                self.publish_protocol_state(
                    mode=self.current_mode,
                    action=ACTION_OUTPUT_NEUTRAL,
                    reason=REASON_TRACK_TIMEOUT,
                    cmd=cmd,
                )
                return
            action = ACTION_FORWARD_TRACK
            self.publish_cmd(self.last_track_cmd)
            self.publish_protocol_state(
                mode=self.current_mode,
                action=action,
                reason=REASON_TRACK_ACTIVE,
                cmd=self.last_track_cmd,
            )
            self.last_source_action = action
            return

        self.current_mode = STATE_SAFE_STOP
        cmd = self.publish_neutral()
        self.publish_protocol_state(
            mode=self.current_mode,
            action=ACTION_OUTPUT_NEUTRAL,
            reason=REASON_INVALID_MODE,
            cmd=cmd,
        )


def parse_args():
    ap = argparse.ArgumentParser(description="Single-output drive arbiter for patrol and track control.")
    ap.add_argument("--output-topic", default="/car_cmd_vel")
    ap.add_argument("--nav-topic", default="/nav_cmd_vel")
    ap.add_argument("--track-topic", default="/track_cmd_vel")
    ap.add_argument("--mode-topic", default="/drive_mode")
    ap.add_argument("--mode-state-file", default="")
    ap.add_argument("--state-topic", default="/drive_arbiter/state")
    ap.add_argument("--status-file", default="/tmp/drive_arbiter_state.json")
    ap.add_argument("--event-log", default="/tmp/drive_arbiter_events.jsonl")
    ap.add_argument("--default-mode", default=STATE_SAFE_STOP, choices=sorted(VALID_STATES))
    ap.add_argument("--timeout-mode-ms", type=int, default=1000)
    ap.add_argument("--timeout-nav-ms", type=int, default=600)
    ap.add_argument("--timeout-track-ms", type=int, default=400)
    ap.add_argument("--neutral-speed", type=float, default=1500.0)
    ap.add_argument("--neutral-angle", type=float, default=90.0)
    ap.add_argument("--lock-duration", type=float, default=0.45)
    ap.add_argument("--startup-lock", type=float, default=0.45)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--control-qos-profile", default="reliable")
    ap.add_argument("--state-qos-profile", default="reliable")
    return ap.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = DriveArbiter(args)
    stop_requested = {"value": False}

    def handle_term(signum, frame):
        stop_requested["value"] = True
        try:
            node.publish_neutral_burst(reason=REASON_MANUAL_STOP)
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    signal.signal(signal.SIGTERM, handle_term)
    signal.signal(signal.SIGINT, handle_term)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.publish_neutral_burst(reason=REASON_UPSTREAM_EXIT if not stop_requested["value"] else REASON_MANUAL_STOP)
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
