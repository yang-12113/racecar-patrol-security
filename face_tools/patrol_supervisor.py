#!/usr/bin/env python3
import argparse
import copy
import errno
import hashlib
import json
import math
import os
import signal
import subprocess
import tempfile
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


STATE_PATROL = "PATROL"
STATE_SUSPECT_CONFIRM = "SUSPECT_CONFIRM"
STATE_TRACK_INTRUDER = "TRACK_INTRUDER"
STATE_RETURN_TO_ROUTE = "RETURN_TO_ROUTE"
STATE_SAFE_STOP = "SAFE_STOP"
SUPERVISOR_SCHEMA_VERSION = 1
PC_VISION_SOURCE = "pc_vision_pipeline"


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float
    name: str


def yaw_to_quaternion(yaw: float) -> Quaternion:
    quat = Quaternion()
    quat.x = 0.0
    quat.y = 0.0
    quat.z = math.sin(yaw / 2.0)
    quat.w = math.cos(yaw / 2.0)
    return quat


def load_waypoints(args) -> List[Waypoint]:
    raw = None
    if args.waypoints_file:
        with open(args.waypoints_file, "r", encoding="utf-8") as f:
            raw = json.load(f)
    elif args.waypoints_json:
        raw = json.loads(args.waypoints_json)
    else:
        raise ValueError("Provide --waypoints-file or --waypoints-json.")

    if not isinstance(raw, list) or not raw:
        raise ValueError("Waypoint list is empty.")

    waypoints: List[Waypoint] = []
    for idx, item in enumerate(raw):
        if isinstance(item, dict):
            x = float(item["x"])
            y = float(item["y"])
            yaw = float(item.get("yaw", 0.0))
            name = str(item.get("name", f"wp_{idx:02d}"))
        elif isinstance(item, list) and len(item) >= 3:
            x = float(item[0])
            y = float(item[1])
            yaw = float(item[2])
            name = f"wp_{idx:02d}"
        else:
            raise ValueError(f"Unsupported waypoint format at index {idx}: {item!r}")
        waypoints.append(Waypoint(x=x, y=y, yaw=yaw, name=name))
    return waypoints


def monotonic_ms() -> int:
    return int(time.monotonic_ns() // 1_000_000)


def canonical_json_bytes(data: dict) -> bytes:
    return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def apply_checksum(data: dict) -> dict:
    payload = copy.deepcopy(data)
    payload.setdefault("meta", {})
    payload["meta"]["checksum_sha256"] = ""
    payload["meta"]["checksum_sha256"] = hashlib.sha256(canonical_json_bytes(payload)).hexdigest()
    return payload


def verify_checksum(data: dict) -> bool:
    try:
        meta = data.get("meta", {})
        expected = str(meta.get("checksum_sha256", "")).strip()
        if not expected:
            return False
        payload = copy.deepcopy(data)
        payload.setdefault("meta", {})
        payload["meta"]["checksum_sha256"] = ""
        return hashlib.sha256(canonical_json_bytes(payload)).hexdigest() == expected
    except Exception:
        return False


def atomic_write_json(path: str, data: dict) -> None:
    if not path:
        return
    tmp_dir = os.path.dirname(path) or "."
    os.makedirs(tmp_dir, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".patrol_supervisor_", suffix=".json", dir=tmp_dir)
    try:
        payload = apply_checksum(data)
        with os.fdopen(fd, "wb") as f:
            f.write(canonical_json_bytes(payload))
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


def append_jsonl(path: str, data: dict) -> None:
    if not path:
        return
    out_dir = os.path.dirname(path) or "."
    os.makedirs(out_dir, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(data, ensure_ascii=False) + "\n")


class PatrolSupervisor(Node):
    def __init__(self, args, waypoints: List[Waypoint]):
        super().__init__("patrol_supervisor")
        self.args = args
        self.waypoints = waypoints

        self.state = STATE_PATROL
        self.current_index = max(0, min(len(self.waypoints) - 1, int(args.start_index)))
        self.last_alert_seen = 0.0
        self.alert_start_ts = 0.0
        self.suspect_start_ts = 0.0
        self.alert_anchor_pose = None
        self.next_dispatch_after = 0.0
        self.last_light = None
        self.last_state_log = ""
        self.last_reason = "startup"
        self.output_seq = 0

        self.light_pub = self.create_publisher(String, args.light_topic, 10)
        self.mode_pub = self.create_publisher(String, args.mode_topic, 10)
        self.nav_client = ActionClient(self, NavigateToPose, args.navigate_action)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_goal_handle = None
        self.nav_goal_index = None
        self.nav_goal_active = False
        self.nav_cancel_requested = False

        self.follow_proc: Optional[subprocess.Popen] = None

        self.timer = self.create_timer(1.0 / max(1.0, float(args.rate)), self.on_timer)
        self.publish_light("green")
        self.get_logger().info(
            "patrol supervisor started: waypoints=%d state_file=%s follow_name=%s "
            "disable_alert=%s max_follow_seconds=%.1f max_follow_distance=%.2f"
            % (
                len(self.waypoints),
                args.state_file,
                args.alert_name,
                bool(args.disable_alert),
                float(args.max_follow_seconds),
                float(args.max_follow_distance),
            )
        )
        append_jsonl(
            self.args.event_log,
            {
                "ts": time.time(),
                "event": "supervisor_start",
                "waypoint_count": int(len(self.waypoints)),
                "alert_name": str(args.alert_name),
                "disable_alert": bool(args.disable_alert),
                "max_follow_seconds": float(args.max_follow_seconds),
                "max_follow_distance": float(args.max_follow_distance),
            },
        )
        self.write_status_file()

    def transition_to(self, new_state: str, reason: str) -> None:
        if self.state == new_state:
            return
        prev_state = self.state
        self.get_logger().info("state %s -> %s: %s" % (self.state, new_state, reason))
        self.last_reason = reason
        self.state = new_state
        append_jsonl(
            self.args.event_log,
            {
                "ts": time.time(),
                "event": "state_transition",
                "from": prev_state,
                "to": new_state,
                "reason": reason,
                "waypoint_index": int(self.current_index),
            },
        )
        self.write_status_file()

    def make_status_payload(self) -> dict:
        self.output_seq += 1
        return {
            "schema_version": SUPERVISOR_SCHEMA_VERSION,
            "ts_monotonic_ms": int(monotonic_ms()),
            "seq": int(self.output_seq),
            "mode": str(self.state),
            "reason": str(self.last_reason),
            "waypoint_index": int(self.current_index),
            "nav_goal_index": -1 if self.nav_goal_index is None else int(self.nav_goal_index),
            "nav_goal_active": bool(self.nav_goal_active),
            "follow_pid": int(self.follow_proc.pid) if self.follow_proc and self.follow_proc.poll() is None else None,
            "light": self.last_light,
            "alert_start_ts_wall": float(self.alert_start_ts),
            "last_alert_seen_wall": float(self.last_alert_seen),
            "alert_anchor_pose": list(self.alert_anchor_pose) if self.alert_anchor_pose is not None else None,
            "meta": {
                "checksum_sha256": "",
            },
        }

    def write_status_file(self) -> None:
        payload = self.make_status_payload()
        if self.args.status_file:
            atomic_write_json(self.args.status_file, payload)
        msg = String()
        msg.data = json.dumps(apply_checksum(payload), ensure_ascii=False, sort_keys=True)
        self.mode_pub.publish(msg)

    def publish_light(self, value: str) -> None:
        if self.last_light == value:
            return
        msg = String()
        msg.data = value
        self.light_pub.publish(msg)
        self.last_light = value
        self.get_logger().info("publish %s=%s" % (self.args.light_topic, value))

    def publish_neutral(self) -> None:
        return None

    def publish_neutral_burst(self, repeat: int = 5, gap: float = 0.05) -> None:
        return None

    def load_follow_state(self) -> Optional[dict]:
        path = self.args.state_file
        if not os.path.exists(path):
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
            if isinstance(data, dict) and "schema_version" in data and isinstance(data.get("meta"), dict):
                if int(data.get("schema_version", -1)) != 1:
                    return None
                if not verify_checksum(data):
                    return None
            return data
        except Exception as exc:
            self.get_logger().warning("failed to load state file: %s" % exc)
            return None

    def is_alert_active(self, state: Optional[dict], now_ts: float) -> bool:
        if self.args.disable_alert:
            return False
        if not state:
            return False
        if str(state.get("source", "")) == PC_VISION_SOURCE:
            if not bool(state.get("active", False)):
                return False
            if not bool(state.get("fresh", False)):
                return False
            age_ms = int(state.get("message_age_ms", 10**9) or 10**9)
            if age_ms > int(self.args.follow_state_timeout * 1000.0):
                return False
            identity = str(state.get("identity", "none"))
            face_state = str(state.get("face_evidence_state", "none"))
            if self.args.alert_name == "unknown":
                return identity == "unknown" and face_state in {"weak_unknown", "confirmed_unknown"}
            if self.args.alert_name == "owner":
                return identity == "owner" and face_state in {"weak_owner", "confirmed_owner"}
            return identity == self.args.alert_name
        if "control_target" in state:
            control_target = state.get("control_target") or {}
            if not control_target.get("active", False):
                return False
            if not control_target.get("control_ok", False):
                return False
            ts_ms = int(control_target.get("ts_monotonic_ms", 0))
            if ts_ms <= 0:
                return False
            if (monotonic_ms() - ts_ms) > int(self.args.follow_state_timeout * 1000.0):
                return False
            name = str(control_target.get("name", ""))
            return name == self.args.alert_name

        if not state.get("active", False):
            return False
        name = str(state.get("name", ""))
        ts = float(state.get("ts", 0.0))
        if now_ts - ts > self.args.follow_state_timeout:
            return False
        return name == self.args.alert_name

    def current_pose_xy(self) -> Optional[tuple]:
        try:
            trans = self.tf_buffer.lookup_transform(
                self.args.map_frame,
                self.args.base_frame,
                rclpy.time.Time(),
            )
            return (
                float(trans.transform.translation.x),
                float(trans.transform.translation.y),
            )
        except TransformException as exc:
            self.get_logger().warning("failed to query robot pose: %s" % exc)
            return None

    def nearest_waypoint_index(self) -> int:
        pose = self.current_pose_xy()
        if pose is None:
            return self.current_index
        rx, ry = pose
        best_idx = self.current_index
        best_dist = None
        for idx, wp in enumerate(self.waypoints):
            dist = (wp.x - rx) ** 2 + (wp.y - ry) ** 2
            if best_dist is None or dist < best_dist:
                best_dist = dist
                best_idx = idx
        return best_idx

    def dispatch_waypoint(self, index: int) -> bool:
        if index < 0 or index >= len(self.waypoints):
            self.enter_safe_stop("invalid waypoint index %s" % index)
            return False

        if not self.nav_client.wait_for_server(timeout_sec=self.args.nav_server_timeout):
            self.enter_safe_stop("navigate_to_pose action server unavailable")
            return False

        wp = self.waypoints[index]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp.x
        goal.pose.pose.position.y = wp.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation = yaw_to_quaternion(wp.yaw)

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(lambda fut, wp_index=index: self.on_goal_response(fut, wp_index))
        self.nav_goal_index = index
        self.nav_cancel_requested = False
        append_jsonl(
            self.args.event_log,
            {
                "ts": time.time(),
                "event": "dispatch_waypoint",
                "waypoint_index": int(index),
                "waypoint_name": wp.name,
                "x": float(wp.x),
                "y": float(wp.y),
                "yaw": float(wp.yaw),
            },
        )
        self.get_logger().info(
            "dispatch waypoint[%d]=%s (%.3f, %.3f, %.3f)"
            % (index, wp.name, wp.x, wp.y, wp.yaw)
        )
        return True

    def on_goal_response(self, future, goal_index: int) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.enter_safe_stop("goal send failed: %s" % exc)
            return

        if not goal_handle.accepted:
            self.enter_safe_stop("goal rejected for waypoint %d" % goal_index)
            return

        self.nav_goal_handle = goal_handle
        self.nav_goal_index = goal_index
        self.nav_goal_active = True
        append_jsonl(
            self.args.event_log,
            {
                "ts": time.time(),
                "event": "goal_accepted",
                "waypoint_index": int(goal_index),
            },
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, wp_index=goal_index: self.on_goal_result(fut, wp_index))
        self.get_logger().info("goal accepted for waypoint %d" % goal_index)

    def on_goal_result(self, future, goal_index: int) -> None:
        self.nav_goal_active = False
        self.nav_goal_handle = None
        try:
            result = future.result()
            status = int(result.status)
        except Exception as exc:
            self.enter_safe_stop("goal result failed: %s" % exc)
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("waypoint %d reached" % goal_index)
            append_jsonl(
                self.args.event_log,
                {
                    "ts": time.time(),
                    "event": "goal_result",
                    "waypoint_index": int(goal_index),
                    "status": "SUCCEEDED",
                },
            )
            if self.state == STATE_PATROL:
                self.current_index = (goal_index + 1) % len(self.waypoints)
                self.next_dispatch_after = time.time() + self.args.waypoint_pause
            return

        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().info("waypoint %d canceled" % goal_index)
            append_jsonl(
                self.args.event_log,
                {
                    "ts": time.time(),
                    "event": "goal_result",
                    "waypoint_index": int(goal_index),
                    "status": "CANCELED",
                },
            )
            return

        if status == GoalStatus.STATUS_ABORTED:
            append_jsonl(
                self.args.event_log,
                {
                    "ts": time.time(),
                    "event": "goal_result",
                    "waypoint_index": int(goal_index),
                    "status": "ABORTED",
                },
            )
            self.enter_safe_stop("waypoint %d aborted" % goal_index)
            return

        self.get_logger().warning("waypoint %d finished with status %d" % (goal_index, status))

    def cancel_nav_goal(self) -> None:
        if not self.nav_goal_handle or not self.nav_goal_active or self.nav_cancel_requested:
            return
        self.nav_cancel_requested = True
        future = self.nav_goal_handle.cancel_goal_async()
        future.add_done_callback(self.on_cancel_done)
        append_jsonl(
            self.args.event_log,
            {
                "ts": time.time(),
                "event": "cancel_nav_goal",
                "waypoint_index": -1 if self.nav_goal_index is None else int(self.nav_goal_index),
            },
        )
        self.get_logger().info("cancel current navigation goal")

    def on_cancel_done(self, future) -> None:
        try:
            future.result()
        except Exception as exc:
            self.get_logger().warning("goal cancel failed: %s" % exc)

    def ensure_follow_controller(self) -> None:
        if self.follow_proc and self.follow_proc.poll() is None:
            return

        cmd = [
            self.args.follow_python,
            self.args.follow_controller_script,
            "--state-file",
            self.args.state_file,
            "--status-file",
            self.args.follow_controller_status_file,
            "--event-log",
            self.args.follow_controller_event_log,
            "--state-topic",
            self.args.follow_controller_state_topic,
            "--cmd-topic",
            self.args.follow_cmd_topic,
            "--target-mode",
            self.args.follow_target_mode,
            "--target-name",
            self.args.follow_target_name,
        ]
        self.get_logger().info("start follow controller: %s" % " ".join(cmd))
        try:
            self.follow_proc = subprocess.Popen(cmd)
            append_jsonl(
                self.args.event_log,
                {
                    "ts": time.time(),
                    "event": "follow_controller_start",
                    "pid": int(self.follow_proc.pid),
                    "target_mode": self.args.follow_target_mode,
                    "target_name": self.args.follow_target_name,
                },
            )
        except Exception as exc:
            self.follow_proc = None
            self.enter_safe_stop("failed to start follow controller: %s" % exc)

    def follow_controller_failed(self) -> bool:
        if not self.follow_proc:
            return False
        code = self.follow_proc.poll()
        if code is None:
            return False
        self.get_logger().error("follow controller exited unexpectedly with code %s" % code)
        append_jsonl(
            self.args.event_log,
            {
                "ts": time.time(),
                "event": "follow_controller_exit",
                "code": int(code),
            },
        )
        self.follow_proc = None
        return True

    def stop_follow_controller(self) -> None:
        if not self.follow_proc:
            return
        if self.follow_proc.poll() is None:
            self.get_logger().info("stop follow controller")
            try:
                self.follow_proc.send_signal(signal.SIGINT)
                self.follow_proc.wait(timeout=self.args.follow_stop_timeout)
            except Exception:
                self.follow_proc.kill()
                try:
                    self.follow_proc.wait(timeout=2.0)
                except Exception:
                    pass
        append_jsonl(
            self.args.event_log,
            {
                "ts": time.time(),
                "event": "follow_controller_stop",
            },
        )
        self.follow_proc = None

    def enter_suspect_confirm(self, reason: str) -> None:
        self.transition_to(STATE_SUSPECT_CONFIRM, reason)
        self.publish_light("red")
        self.cancel_nav_goal()
        self.publish_neutral_burst()
        self.suspect_start_ts = time.time()
        self.last_alert_seen = self.suspect_start_ts

    def enter_track_intruder(self, reason: str) -> None:
        self.transition_to(STATE_TRACK_INTRUDER, reason)
        self.publish_light("red")
        self.cancel_nav_goal()
        self.publish_neutral_burst()
        self.ensure_follow_controller()
        self.last_alert_seen = time.time()
        self.alert_start_ts = self.last_alert_seen
        self.alert_anchor_pose = self.current_pose_xy()

    def enter_return_to_route(self, reason: str) -> None:
        self.transition_to(STATE_RETURN_TO_ROUTE, reason)
        self.stop_follow_controller()
        self.publish_neutral_burst()
        self.publish_light("green")
        self.alert_anchor_pose = None
        self.suspect_start_ts = 0.0
        self.current_index = self.nearest_waypoint_index()
        self.get_logger().info("resume patrol from waypoint index %d" % self.current_index)
        self.next_dispatch_after = time.time() + self.args.resume_pause

    def enter_safe_stop(self, reason: str) -> None:
        self.transition_to(STATE_SAFE_STOP, reason)
        self.cancel_nav_goal()
        self.stop_follow_controller()
        self.publish_light("red")
        self.alert_anchor_pose = None
        self.suspect_start_ts = 0.0
        self.publish_neutral_burst()

    def follow_limits_exceeded(self, now_ts: float) -> Optional[str]:
        if self.args.max_follow_seconds > 0 and self.alert_start_ts > 0:
            if (now_ts - self.alert_start_ts) >= self.args.max_follow_seconds:
                return "follow duration limit exceeded"

        if self.args.max_follow_distance > 0 and self.alert_anchor_pose is not None:
            pose = self.current_pose_xy()
            if pose is not None:
                dx = float(pose[0]) - float(self.alert_anchor_pose[0])
                dy = float(pose[1]) - float(self.alert_anchor_pose[1])
                dist = math.hypot(dx, dy)
                if dist >= self.args.max_follow_distance:
                    return "follow distance limit exceeded"
        return None

    def on_timer(self) -> None:
        now_ts = time.time()
        state = self.load_follow_state()
        alert_active = self.is_alert_active(state, now_ts)

        if self.state == STATE_PATROL:
            self.publish_light("green")
            if alert_active:
                self.enter_suspect_confirm("intruder candidate")
                self.write_status_file()
                return
            if not self.nav_goal_active and now_ts >= self.next_dispatch_after:
                self.dispatch_waypoint(self.current_index)
            self.write_status_file()
            return

        if self.state == STATE_SUSPECT_CONFIRM:
            self.publish_light("red")
            if alert_active:
                self.last_alert_seen = now_ts
                if (now_ts - self.suspect_start_ts) >= float(self.args.suspect_confirm_seconds):
                    self.enter_track_intruder("intruder confirmed")
                    self.write_status_file()
                    return
            elif (now_ts - self.last_alert_seen) >= float(self.args.suspect_lost_timeout):
                self.transition_to(STATE_PATROL, "suspect disappeared")
                self.publish_light("green")
                self.next_dispatch_after = time.time() + self.args.resume_pause
                self.write_status_file()
                return
            self.write_status_file()
            return

        if self.state == STATE_TRACK_INTRUDER:
            self.publish_light("red")
            if self.follow_controller_failed():
                self.enter_safe_stop("follow controller exited unexpectedly")
                self.write_status_file()
                return
            self.ensure_follow_controller()
            leash_reason = self.follow_limits_exceeded(now_ts)
            if leash_reason is not None:
                self.enter_return_to_route(leash_reason)
                self.write_status_file()
                return
            if alert_active:
                self.last_alert_seen = now_ts
            elif (now_ts - self.last_alert_seen) >= self.args.target_lost_timeout:
                self.enter_return_to_route("target lost")
                self.write_status_file()
                return
            self.write_status_file()
            return

        if self.state == STATE_RETURN_TO_ROUTE:
            self.publish_light("green")
            if alert_active:
                self.enter_suspect_confirm("intruder reappeared during return")
                self.write_status_file()
                return
            if not self.nav_goal_active and now_ts >= self.next_dispatch_after:
                if self.dispatch_waypoint(self.current_index):
                    self.transition_to(STATE_PATROL, "resume waypoint dispatched")
            self.write_status_file()
            return

        if self.state == STATE_SAFE_STOP:
            self.publish_light("red")
            self.write_status_file()
            return

    def shutdown(self) -> None:
        self.stop_follow_controller()
        self.publish_neutral_burst()


def parse_args():
    ap = argparse.ArgumentParser(
        description="Patrol supervisor: patrol with Nav2, switch to intruder tracking, then resume route."
    )
    ap.add_argument("--waypoints-file", default="")
    ap.add_argument("--waypoints-json", default="")
    ap.add_argument("--start-index", type=int, default=0)
    ap.add_argument("--rate", type=float, default=5.0)
    ap.add_argument("--navigate-action", default="navigate_to_pose")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--light-topic", default="/light")
    ap.add_argument("--mode-topic", default="/security/motion_mode")
    ap.add_argument("--state-file", default="/tmp/pc_vision_state.json")
    ap.add_argument("--alert-name", default="unknown")
    ap.add_argument("--follow-state-timeout", type=float, default=1.0)
    ap.add_argument("--target-lost-timeout", type=float, default=1.5)
    ap.add_argument("--suspect-confirm-seconds", type=float, default=0.30)
    ap.add_argument("--suspect-lost-timeout", type=float, default=0.50)
    ap.add_argument("--nav-server-timeout", type=float, default=5.0)
    ap.add_argument("--waypoint-pause", type=float, default=0.5)
    ap.add_argument("--resume-pause", type=float, default=0.5)
    ap.add_argument("--follow-python", default="/usr/bin/python3")
    ap.add_argument("--follow-controller-script", default="/root/face_tools/face_follow_controller.py")
    ap.add_argument("--follow-controller-state-topic", default="/follow_controller/state")
    ap.add_argument("--follow-controller-status-file", default="/tmp/follow_controller_state.json")
    ap.add_argument("--follow-controller-event-log", default="/tmp/follow_controller_events.jsonl")
    ap.add_argument("--follow-cmd-topic", default="/security/follow_cmd_vel")
    ap.add_argument("--follow-target-mode", default="unknown")
    ap.add_argument("--follow-target-name", default="")
    ap.add_argument("--follow-stop-timeout", type=float, default=3.0)
    ap.add_argument("--max-follow-seconds", type=float, default=25.0)
    ap.add_argument("--max-follow-distance", type=float, default=3.0)
    ap.add_argument("--status-file", default="/tmp/patrol_supervisor_state.json")
    ap.add_argument("--event-log", default="", help="Optional JSONL file for patrol state transitions and supervisor events.")
    ap.add_argument("--disable-alert", action="store_true", help="Disable intruder-triggered tracking and stay in patrol-only mode.")
    ap.add_argument("--neutral-speed", type=float, default=1500.0)
    ap.add_argument("--neutral-angle", type=float, default=90.0)
    return ap.parse_args()


def main():
    args = parse_args()
    waypoints = load_waypoints(args)
    rclpy.init()
    node = PatrolSupervisor(args, waypoints)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        if "context is not valid" not in str(exc).lower():
            raise
    finally:
        try:
            node.shutdown()
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
