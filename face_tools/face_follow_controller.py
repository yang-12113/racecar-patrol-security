#!/usr/bin/env python3
import argparse
import copy
import errno
import hashlib
import json
import math
import os
import tempfile
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


FOLLOW_CONTROLLER_SCHEMA_VERSION = 1
FOLLOW_INPUT_SCHEMA_VERSION = 1
PC_VISION_SOURCE = "pc_vision_pipeline"
FALLBACK_LOCAL_SOURCE = "fallback_local_detector"

ACTION_MOVE_FORWARD = "MOVE_FORWARD"
ACTION_TURN_ONLY = "TURN_ONLY"
ACTION_HOLD_NEUTRAL = "HOLD_NEUTRAL"

STOP_NONE = "NONE"
STOP_TARGET_INACTIVE = "TARGET_INACTIVE"
STOP_CONTROL_NOT_OK = "CONTROL_NOT_OK"
STOP_TARGET_LOST = "TARGET_LOST"
STOP_TARGET_STALE = "TARGET_STALE"
STOP_TARGET_NOT_MATCHED = "TARGET_NOT_MATCHED"
STOP_TARGET_TIMEOUT = "TARGET_TIMEOUT"
STOP_TARGET_MODE_MISMATCH = "TARGET_MODE_MISMATCH"
STOP_UNKNOWN_NO_FACE_EVIDENCE = "UNKNOWN_NO_FACE_EVIDENCE"
STOP_STATE_READ_ERROR = "STATE_READ_ERROR"
STOP_STATE_SCHEMA_MISMATCH = "STATE_SCHEMA_MISMATCH"
STOP_STATE_CHECKSUM_ERROR = "STATE_CHECKSUM_ERROR"
STOP_STATE_SEQ_STALLED = "STATE_SEQ_STALLED"
STOP_OBSTACLE_BLOCKED = "OBSTACLE_BLOCKED"
STOP_SCAN_STALE = "SCAN_STALE"
STOP_MANUAL_STOP = "MANUAL_STOP"

VALID_CONTROL_ACTIONS = {
    ACTION_MOVE_FORWARD,
    ACTION_TURN_ONLY,
    ACTION_HOLD_NEUTRAL,
}
VALID_STOP_REASONS = {
    STOP_NONE,
    STOP_TARGET_INACTIVE,
    STOP_CONTROL_NOT_OK,
    STOP_TARGET_LOST,
    STOP_TARGET_STALE,
    STOP_TARGET_NOT_MATCHED,
    STOP_TARGET_TIMEOUT,
    STOP_TARGET_MODE_MISMATCH,
    STOP_UNKNOWN_NO_FACE_EVIDENCE,
    STOP_STATE_READ_ERROR,
    STOP_STATE_SCHEMA_MISMATCH,
    STOP_STATE_CHECKSUM_ERROR,
    STOP_STATE_SEQ_STALLED,
    STOP_OBSTACLE_BLOCKED,
    STOP_SCAN_STALE,
    STOP_MANUAL_STOP,
}


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def monotonic_ms():
    return int(time.monotonic_ns() // 1_000_000)


def canonical_json_bytes(data):
    return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def apply_checksum(payload):
    out = copy.deepcopy(payload)
    out.setdefault("meta", {})
    out["meta"]["checksum_sha256"] = ""
    checksum = hashlib.sha256(canonical_json_bytes(out)).hexdigest()
    payload = copy.deepcopy(payload)
    payload.setdefault("meta", {})
    payload["meta"]["checksum_sha256"] = checksum
    return payload


def verify_checksum(payload):
    try:
        expected = str(payload["meta"]["checksum_sha256"])
    except Exception:
        return False
    probe = copy.deepcopy(payload)
    probe.setdefault("meta", {})
    probe["meta"]["checksum_sha256"] = ""
    actual = hashlib.sha256(canonical_json_bytes(probe)).hexdigest()
    return actual == expected


def atomic_write_json(path, data, prefix=".follow_controller_", suffix=".json"):
    if not path:
        return
    tmp_dir = os.path.dirname(path) or "."
    os.makedirs(tmp_dir, exist_ok=True)
    payload = apply_checksum(data)
    fd, tmp_path = tempfile.mkstemp(prefix=prefix, suffix=suffix, dir=tmp_dir)
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(canonical_json_bytes(payload))
            f.flush()
            os.fsync(f.fileno())
        try:
            os.replace(tmp_path, path)
        except OSError as exc:
            if exc.errno == errno.EXDEV:
                raise RuntimeError(f"atomic rename failed across filesystems for {path}") from exc
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
    name = str(name or "reliable").strip().lower()
    reliability = ReliabilityPolicy.RELIABLE
    if name in {"best_effort", "best-available", "best_available"}:
        reliability = ReliabilityPolicy.BEST_EFFORT
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.VOLATILE,
        reliability=reliability,
    )


class FaceFollowController(Node):
    def __init__(self, args):
        super().__init__("face_follow_controller")
        self.args = args
        cmd_qos = make_qos_profile(args.control_qos_profile)
        state_qos = make_qos_profile(args.state_qos_profile)
        self.pub = self.create_publisher(Twist, args.cmd_topic, cmd_qos)
        self.state_pub = self.create_publisher(String, args.state_topic, state_qos)
        self.scan_sub = self.create_subscription(LaserScan, args.scan_topic, self.on_scan, 10)
        self.last_log = 0.0
        self.last_publish = None
        self.last_scan_ts = 0.0
        self.last_scan_info = None
        self.last_input_seq = None
        self.last_input_seq_change_ms = 0
        self.output_seq = 0
        self.invalid_state_count = 0
        self.last_state_signature = None
        self.last_stop_reason = STOP_NONE
        self.last_control_action = ACTION_HOLD_NEUTRAL
        self.timer = self.create_timer(1.0 / max(1.0, float(args.rate)), self.on_timer)
        self.get_logger().info(
            f"Face follow controller started: state_file={args.state_file}, "
            f"target_mode={args.target_mode}, target_name={args.target_name}, "
            f"scan_topic={args.scan_topic}, cmd_topic={args.cmd_topic}, state_topic={args.state_topic}"
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

    def load_payload(self):
        path = self.args.state_file
        if not os.path.exists(path):
            return None, STOP_STATE_READ_ERROR
        try:
            with open(path, "r", encoding="utf-8") as f:
                payload = json.load(f)
        except Exception:
            return None, STOP_STATE_READ_ERROR

        if not isinstance(payload, dict):
            return None, STOP_STATE_READ_ERROR
        if str(payload.get("source", "")) in {PC_VISION_SOURCE, FALLBACK_LOCAL_SOURCE}:
            return payload, STOP_NONE
        if int(payload.get("schema_version", -1)) != FOLLOW_INPUT_SCHEMA_VERSION:
            return None, STOP_STATE_SCHEMA_MISMATCH
        if not verify_checksum(payload):
            return None, STOP_STATE_CHECKSUM_ERROR
        return payload, STOP_NONE

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
        near_area = self.args.desired_area + self.args.area_tol
        far_area = self.args.desired_area - self.args.area_tol
        if area_ratio < far_area:
            gap = far_area - area_ratio
            speed = self.args.neutral_speed + self.args.speed_gain * gap / max(1e-6, self.args.desired_area)
        else:
            speed = self.args.neutral_speed

        if abs(ex) > self.args.turn_slow_band:
            speed -= self.args.turn_slow_down

        speed = clamp(speed, self.args.neutral_speed, self.args.max_speed)
        return float(speed), float(angle), float(ex), float(area_ratio), float(near_area)

    def parse_target_snapshot(self, payload):
        if not payload:
            return {}
        source = str(payload.get("source", ""))
        if source in {PC_VISION_SOURCE, FALLBACK_LOCAL_SOURCE}:
            return {
                "track_id": payload.get("track_id"),
                "name": str(payload.get("identity", "")),
                "bbox_xyxy_norm": payload.get("bbox_xyxy_norm", []),
                "confidence": float(payload.get("confidence", 0.0)),
                "area_ratio": float(payload.get("distance_proxy", 0.0)),
                "matched_this_frame": bool(payload.get("active", False) and payload.get("fresh", False)),
                "face_evidence_state": str(payload.get("face_evidence_state", "none")),
            }
        control_target = payload.get("control_target") or {}
        snapshot = {
            "track_id": control_target.get("track_id"),
            "name": control_target.get("name", ""),
            "bbox_xyxy_norm": control_target.get("bbox_xyxy_norm", [0, 0, 0, 0]),
            "confidence": float(control_target.get("confidence", 0.0)),
            "area_ratio": float(control_target.get("area_ratio", 0.0)),
            "matched_this_frame": bool(control_target.get("matched_this_frame", False)),
        }
        return snapshot

    def evaluate_control_target(self, payload, now_ms):
        if payload is None:
            return None, STOP_STATE_READ_ERROR, 0

        source = str(payload.get("source", ""))
        if source == PC_VISION_SOURCE:
            seq = int(payload.get("seq", -1))
            fresh_age_ms = int(payload.get("message_age_ms", 0) or 0)

            if self.last_input_seq is None or seq != self.last_input_seq:
                self.last_input_seq = seq
                self.last_input_seq_change_ms = now_ms
            elif bool(payload.get("active", False)) and (now_ms - self.last_input_seq_change_ms) > int(self.args.seq_stall_timeout_ms):
                return None, STOP_STATE_SEQ_STALLED, fresh_age_ms

            if not bool(payload.get("active", False)):
                return None, STOP_TARGET_INACTIVE, fresh_age_ms
            if not bool(payload.get("fresh", False)):
                return None, STOP_TARGET_TIMEOUT, fresh_age_ms

            name = str(payload.get("identity", "none"))
            face_state = str(payload.get("face_evidence_state", "none"))
            if self.args.target_mode == "owner":
                expected = self.args.target_name or "owner"
                if name != expected:
                    return None, STOP_TARGET_MODE_MISMATCH, fresh_age_ms
            elif self.args.target_mode == "unknown":
                if name != "unknown":
                    return None, STOP_TARGET_MODE_MISMATCH, fresh_age_ms
                if (
                    not self.args.allow_unknown_without_face_evidence
                    and face_state not in {"weak_unknown", "confirmed_unknown"}
                ):
                    return None, STOP_UNKNOWN_NO_FACE_EVIDENCE, fresh_age_ms
            elif self.args.target_mode == "any" and name == "none" and source != FALLBACK_LOCAL_SOURCE:
                return None, STOP_TARGET_MODE_MISMATCH, fresh_age_ms

            width = max(1, int(payload.get("width", self.args.image_width)))
            height = max(1, int(payload.get("height", self.args.image_height)))
            cx_norm = float(payload.get("cx_norm", 0.5))
            cy_norm = float(payload.get("cy_norm", 0.5))
            state = {
                "track_id": payload.get("track_id"),
                "name": name,
                "cx": cx_norm * width,
                "cy": cy_norm * height,
                "width": width,
                "height": height,
                "area_ratio": float(payload.get("distance_proxy", 0.0)),
                "confidence": float(payload.get("confidence", 0.0)),
                "bbox_xyxy_norm": [],
                "ts_monotonic_ms": int(payload.get("recv_monotonic_ms", now_ms)),
                "seq": seq,
                "face_evidence_state": face_state,
            }
            return state, STOP_NONE, fresh_age_ms

        if source == FALLBACK_LOCAL_SOURCE:
            recv_monotonic_ms = int(payload.get("recv_monotonic_ms", 0) or 0)
            if recv_monotonic_ms > 0:
                fresh_age_ms = max(0, int(now_ms - recv_monotonic_ms))
            else:
                fresh_age_ms = int(payload.get("message_age_ms", 0) or 0)

            if not bool(payload.get("active", False)):
                return None, STOP_TARGET_INACTIVE, fresh_age_ms
            if fresh_age_ms > int(self.args.local_state_fresh_timeout_ms):
                return None, STOP_TARGET_TIMEOUT, fresh_age_ms

            name = str(payload.get("identity", "none"))
            face_state = str(payload.get("face_evidence_state", "none"))
            if self.args.target_mode == "owner":
                expected = self.args.target_name or "owner"
                if name != expected:
                    return None, STOP_TARGET_MODE_MISMATCH, fresh_age_ms
            elif self.args.target_mode == "unknown":
                if name != "unknown":
                    return None, STOP_TARGET_MODE_MISMATCH, fresh_age_ms
                if (
                    not self.args.allow_unknown_without_face_evidence
                    and face_state not in {"weak_unknown", "confirmed_unknown"}
                ):
                    return None, STOP_UNKNOWN_NO_FACE_EVIDENCE, fresh_age_ms

            width = max(1, int(payload.get("width", self.args.image_width)))
            height = max(1, int(payload.get("height", self.args.image_height)))
            cx_norm = float(payload.get("cx_norm", 0.5))
            cy_norm = float(payload.get("cy_norm", 0.5))
            state = {
                "track_id": payload.get("track_id"),
                "name": name,
                "cx": cx_norm * width,
                "cy": cy_norm * height,
                "width": width,
                "height": height,
                "area_ratio": float(payload.get("distance_proxy", 0.0)),
                "confidence": float(payload.get("confidence", 0.0)),
                "bbox_xyxy_norm": [],
                "ts_monotonic_ms": recv_monotonic_ms if recv_monotonic_ms > 0 else now_ms,
                "seq": int(payload.get("seq", -1)),
                "face_evidence_state": face_state,
            }
            return state, STOP_NONE, fresh_age_ms

        meta = payload.get("meta") or {}
        control_target = payload.get("control_target") or {}
        seq = int(meta.get("seq", -1))
        write_ts_ms = int(meta.get("write_ts_monotonic_ms", 0))
        fresh_age_ms = max(0, int(now_ms - write_ts_ms))

        if self.last_input_seq is None or seq != self.last_input_seq:
            self.last_input_seq = seq
            self.last_input_seq_change_ms = now_ms
        elif (
            bool(control_target.get("active", False))
            and (now_ms - self.last_input_seq_change_ms) > int(self.args.seq_stall_timeout_ms)
        ):
            return None, STOP_STATE_SEQ_STALLED, fresh_age_ms

        if not bool(control_target.get("active", False)):
            return None, STOP_TARGET_INACTIVE, fresh_age_ms
        if not bool(control_target.get("control_ok", False)):
            return None, STOP_CONTROL_NOT_OK, fresh_age_ms
        if int(control_target.get("lost", 0)) != 0:
            return None, STOP_TARGET_LOST, fresh_age_ms
        if bool(control_target.get("stale", False)):
            return None, STOP_TARGET_STALE, fresh_age_ms
        if not bool(control_target.get("matched_this_frame", False)):
            return None, STOP_TARGET_NOT_MATCHED, fresh_age_ms
        if fresh_age_ms > int(self.args.fresh_timeout_ms):
            return None, STOP_TARGET_TIMEOUT, fresh_age_ms

        name = str(control_target.get("name", "unknown"))
        if self.args.target_mode == "owner" and self.args.target_name and name != self.args.target_name:
            return None, STOP_TARGET_MODE_MISMATCH, fresh_age_ms
        if self.args.target_mode == "unknown":
            if name != "unknown":
                return None, STOP_TARGET_MODE_MISMATCH, fresh_age_ms
            if (
                not self.args.allow_unknown_without_face_evidence
                and not bool(control_target.get("has_recent_face_evidence", False))
            ):
                return None, STOP_UNKNOWN_NO_FACE_EVIDENCE, fresh_age_ms

        state = {
            "track_id": control_target.get("track_id"),
            "name": name,
            "cx": float(control_target.get("cx", control_target.get("center_x_px", 0.5 * self.args.image_width))),
            "cy": float(control_target.get("cy", 0.0)),
            "width": int(control_target.get("width", self.args.image_width)),
            "height": int(control_target.get("height", 0)),
            "area_ratio": float(control_target.get("area_ratio", 0.0)),
            "confidence": float(control_target.get("confidence", 0.0)),
            "bbox_xyxy_norm": control_target.get("bbox_xyxy_norm", [0, 0, 0, 0]),
            "ts_monotonic_ms": write_ts_ms,
            "seq": seq,
        }
        return state, STOP_NONE, fresh_age_ms

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

    def make_output_payload(self, control_action, stop_reason, fresh_age_ms, target_snapshot, obstacle_info, cmd_speed, cmd_angle):
        self.output_seq += 1
        payload = {
            "schema_version": FOLLOW_CONTROLLER_SCHEMA_VERSION,
            "ts_monotonic_ms": monotonic_ms(),
            "seq": int(self.output_seq),
            "control_action": str(control_action),
            "stop_reason": str(stop_reason),
            "fresh_age_ms": int(fresh_age_ms),
            "cmd": {
                "speed": float(cmd_speed),
                "angle": float(cmd_angle),
            },
            "target_snapshot": target_snapshot or {},
            "obstacle": obstacle_info or {},
            "counters": {
                "invalid_state_count": int(self.invalid_state_count),
            },
        }
        return payload

    def publish_protocol_state(self, payload):
        atomic_write_json(self.args.status_file, payload, prefix=".follow_controller_state_", suffix=".json")
        msg = String()
        msg.data = json.dumps(apply_checksum(payload), ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        self.state_pub.publish(msg)
        signature = (
            payload.get("control_action"),
            payload.get("stop_reason"),
            payload.get("target_snapshot", {}).get("track_id"),
            payload.get("target_snapshot", {}).get("name"),
        )
        if signature != self.last_state_signature:
            append_jsonl(
                self.args.event_log,
                {
                    "schema_version": FOLLOW_CONTROLLER_SCHEMA_VERSION,
                    "ts_monotonic_ms": int(payload["ts_monotonic_ms"]),
                    "seq": int(payload["seq"]),
                    "event": "controller_state_change",
                    "control_action": str(payload.get("control_action")),
                    "stop_reason": str(payload.get("stop_reason")),
                    "target_snapshot": payload.get("target_snapshot", {}),
                    "fresh_age_ms": int(payload.get("fresh_age_ms", 0)),
                },
            )
            self.last_state_signature = signature

    def on_timer(self):
        now_wall = time.time()
        now_ms = monotonic_ms()
        payload, load_reason = self.load_payload()
        if load_reason != STOP_NONE:
            self.invalid_state_count += 1
            self.publish_neutral()
            output = self.make_output_payload(
                ACTION_HOLD_NEUTRAL,
                load_reason,
                0,
                {},
                {},
                self.args.neutral_speed,
                self.args.neutral_angle,
            )
            self.publish_protocol_state(output)
            return

        state, reason, fresh_age_ms = self.evaluate_control_target(payload, now_ms)
        if reason != STOP_NONE:
            self.invalid_state_count += 1
            self.publish_neutral()
            output = self.make_output_payload(
                ACTION_HOLD_NEUTRAL,
                reason,
                fresh_age_ms,
                self.parse_target_snapshot(payload),
                {},
                self.args.neutral_speed,
                self.args.neutral_angle,
            )
            self.publish_protocol_state(output)
            return

        speed, angle, ex, area_ratio, near_area = self.compute_cmd(state)
        speed, angle, obstacle_info = self.apply_obstacle_response(speed, angle)

        if obstacle_info and obstacle_info.get("mode") == "scan_stale_stop":
            control_action = ACTION_HOLD_NEUTRAL
            stop_reason = STOP_SCAN_STALE
            speed = self.args.neutral_speed
            angle = self.args.neutral_angle
        elif obstacle_info and obstacle_info.get("mode") == "blocked_stop":
            control_action = ACTION_HOLD_NEUTRAL
            stop_reason = STOP_OBSTACLE_BLOCKED
            speed = self.args.neutral_speed
            angle = self.args.neutral_angle
        elif speed > (self.args.neutral_speed + 1e-6):
            control_action = ACTION_MOVE_FORWARD
            stop_reason = STOP_NONE
        elif abs(angle - self.args.neutral_angle) > self.args.turn_only_epsilon:
            control_action = ACTION_TURN_ONLY
            stop_reason = STOP_NONE
        else:
            control_action = ACTION_HOLD_NEUTRAL
            stop_reason = STOP_NONE

        self.publish_cmd(speed, angle)
        target_snapshot = {
            "track_id": state.get("track_id"),
            "name": state.get("name", ""),
            "bbox_xyxy_norm": state.get("bbox_xyxy_norm", [0, 0, 0, 0]),
            "confidence": float(state.get("confidence", 0.0)),
            "area_ratio": float(area_ratio),
            "matched_this_frame": True,
        }
        output = self.make_output_payload(
            control_action,
            stop_reason,
            fresh_age_ms,
            target_snapshot,
            obstacle_info,
            speed,
            angle,
        )
        self.publish_protocol_state(output)

        if now_wall - self.last_log >= 1.0:
            self.last_log = now_wall
            obs_text = ""
            if obstacle_info:
                obs_mode = obstacle_info.get("mode", "")
                if obs_mode:
                    obs_text = (
                        f" obs={obs_mode} front={obstacle_info.get('front_min', float('inf')):.2f}"
                        f" left={obstacle_info.get('left_clear', float('inf')):.2f}"
                        f" right={obstacle_info.get('right_clear', float('inf')):.2f}"
                    )
            self.get_logger().info(
                f"action={control_action} stop_reason={stop_reason} name={state.get('name','')} "
                f"id={state.get('track_id', -1)} ex={ex:.3f} area={area_ratio:.3f} near={near_area:.3f} "
                f"cmd=({speed:.1f}, {angle:.1f}) fresh_age_ms={fresh_age_ms}{obs_text}"
            )


def parse_args():
    ap = argparse.ArgumentParser(description="Read face-follow state and publish track commands.")
    ap.add_argument("--state-file", default="/tmp/pc_vision_state.json")
    ap.add_argument("--target-mode", default="owner", choices=["owner", "unknown", "any"])
    ap.add_argument("--target-name", default="owner")
    ap.add_argument("--cmd-topic", default="/security/follow_cmd_vel")
    ap.add_argument("--state-topic", default="/follow_controller/state")
    ap.add_argument("--status-file", default="/tmp/follow_controller_state.json")
    ap.add_argument("--event-log", default="/tmp/follow_controller_events.jsonl")
    ap.add_argument("--control-qos-profile", default="reliable", choices=["reliable", "best_effort", "best_available"])
    ap.add_argument("--state-qos-profile", default="reliable", choices=["reliable", "best_effort", "best_available"])
    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--scan-timeout", type=float, default=0.8)
    ap.add_argument("--allow-stale-scan-motion", action="store_true")
    ap.add_argument("--allow-unknown-without-face-evidence", action="store_true")
    ap.add_argument("--image-width", type=int, default=640)
    ap.add_argument("--image-height", type=int, default=720)
    ap.add_argument("--fresh-timeout-ms", type=int, default=220)
    ap.add_argument("--seq-stall-timeout-ms", type=int, default=180)
    ap.add_argument("--local-state-fresh-timeout-ms", type=int, default=1500)
    ap.add_argument("--neutral-speed", type=float, default=1500.0)
    ap.add_argument("--neutral-angle", type=float, default=90.0)
    ap.add_argument("--max-speed", type=float, default=1532.0)
    ap.add_argument("--speed-gain", type=float, default=55.0)
    ap.add_argument("--desired-area", type=float, default=0.16)
    ap.add_argument("--area-tol", type=float, default=0.03)
    ap.add_argument("--steer-gain", type=float, default=32.0)
    ap.add_argument("--steer-sign", type=float, default=1.0)
    ap.add_argument("--deadband", type=float, default=0.04)
    ap.add_argument("--turn-slow-band", type=float, default=0.20)
    ap.add_argument("--turn-slow-down", type=float, default=4.0)
    ap.add_argument("--turn-only-epsilon", type=float, default=1.0)
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
    except (KeyboardInterrupt, ExternalShutdownException):
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
