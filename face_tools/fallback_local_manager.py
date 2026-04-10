#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import tempfile
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


MODE_FALLBACK_LOCAL = "FALLBACK_LOCAL"
MODE_SAFE_STOP = "SAFE_STOP"


def monotonic_ms() -> int:
    return time.monotonic_ns() // 1_000_000


def atomic_write_json(path: str, data: dict) -> None:
    if not path:
        return
    out_dir = os.path.dirname(path) or "."
    os.makedirs(out_dir, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".fallback_manager_", suffix=".json", dir=out_dir)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, sort_keys=True)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp_path, path)
    finally:
        if os.path.exists(tmp_path):
            try:
                os.remove(tmp_path)
            except OSError:
                pass


def safe_unlink(path: str) -> None:
    if not path:
        return
    try:
        os.remove(path)
    except FileNotFoundError:
        pass
    except OSError:
        pass


class ManagedProc:
    def __init__(self, name: str, log_path: str) -> None:
        self.name = name
        self.log_path = log_path
        self.proc: Optional[subprocess.Popen] = None

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def pid(self) -> Optional[int]:
        return self.proc.pid if self.running() else None

    def start(self, cmd: list[str]) -> None:
        if self.running():
            return
        os.makedirs(os.path.dirname(self.log_path) or ".", exist_ok=True)
        logf = open(self.log_path, "ab", buffering=0)
        self.proc = subprocess.Popen(
            cmd,
            stdout=logf,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            close_fds=True,
        )

    def stop(self, timeout_s: float = 3.0) -> None:
        if not self.proc:
            return
        proc = self.proc
        self.proc = None
        if proc.poll() is not None:
            return
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except Exception:
            try:
                proc.terminate()
            except Exception:
                pass
        deadline = time.time() + max(0.1, timeout_s)
        while time.time() < deadline:
            if proc.poll() is not None:
                return
            time.sleep(0.1)
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass


class FallbackLocalManager(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("fallback_local_manager")
        self.args = args
        self.mode = MODE_SAFE_STOP
        self.last_mode_ms = 0
        self.phase = "NORMAL"
        self.phase_reason = "startup"
        self.phase_entered_ms = monotonic_ms()
        self.startup_deadline_ms = monotonic_ms() + int(args.startup_grace_ms)
        self.next_probe_ms = 0
        self.probe_started_ms = 0
        self.probe_fresh_since_ms: Optional[int] = None
        self.sender = ManagedProc("sender", args.sender_log)
        self.fallback_detector = ManagedProc("fallback_detector", args.fallback_detector_log)
        self.fallback_controller = ManagedProc("fallback_controller", args.fallback_controller_log)
        self.create_subscription(String, args.mode_topic, self.on_mode, 10)
        self.timer = self.create_timer(1.0 / max(1.0, float(args.rate)), self.on_timer)
        self.ensure_normal_mode(reason="startup")
        self.get_logger().info(
            f"fallback manager started: bridge_status={args.bridge_status_file}, sender_output={args.sender_output_url}, "
            f"fallback_state={args.fallback_state_file}"
        )

    def on_mode(self, msg: String) -> None:
        raw = str(msg.data or "").strip()
        mode = raw
        if raw.startswith("{"):
            try:
                mode = str(json.loads(raw).get("mode", raw))
            except Exception:
                mode = raw
        self.mode = mode.strip() or MODE_SAFE_STOP
        self.last_mode_ms = monotonic_ms()

    def load_json(self, path: str) -> Optional[dict]:
        if not path or not os.path.exists(path):
            return None
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return None

    def build_sender_cmd(self) -> list[str]:
        return [
            self.args.ros_py,
            os.path.join(self.args.face_tools_dir, "car_sender.py"),
            "--output-url",
            self.args.sender_output_url,
            "--sender-mode",
            "framed-mjpg-tcp",
            *self.args.camera_arg,
        ]

    def build_fallback_detector_cmd(self) -> list[str]:
        return [
            "/root/run_yolo_v5_stream.sh",
            "--target-cls",
            "0",
            "--print-every",
            "0",
            "--conf",
            str(self.args.fallback_conf),
            "--state-file",
            self.args.fallback_state_file,
            "--state-source",
            "fallback_local_detector",
            "--stream-port",
            "0",
            *self.args.camera_arg,
        ]

    def build_fallback_controller_cmd(self) -> list[str]:
        return [
            self.args.ros_py,
            os.path.join(self.args.face_tools_dir, "face_follow_controller.py"),
            "--state-file",
            self.args.fallback_state_file,
            "--target-mode",
            "any",
            "--target-name",
            "",
            "--cmd-topic",
            self.args.fallback_cmd_topic,
            "--status-file",
            self.args.fallback_controller_status_file,
            "--event-log",
            self.args.fallback_controller_event_log,
            "--image-width",
            str(self.args.fallback_image_width),
            "--image-height",
            str(self.args.fallback_image_height),
        ]

    def ensure_normal_mode(self, reason: str) -> None:
        self.fallback_controller.stop()
        self.fallback_detector.stop()
        safe_unlink(self.args.fallback_state_file)
        self.sender.start(self.build_sender_cmd())
        self.phase = "NORMAL"
        self.phase_reason = reason
        self.phase_entered_ms = monotonic_ms()
        self.probe_started_ms = 0
        self.probe_fresh_since_ms = None

    def ensure_fallback_mode(self, reason: str) -> None:
        self.sender.stop()
        self.fallback_controller.stop()
        self.fallback_detector.stop()
        safe_unlink(self.args.fallback_state_file)
        self.fallback_detector.start(self.build_fallback_detector_cmd())
        self.fallback_controller.start(self.build_fallback_controller_cmd())
        self.phase = "FALLBACK"
        self.phase_reason = reason
        self.phase_entered_ms = monotonic_ms()
        self.next_probe_ms = monotonic_ms() + int(self.args.probe_interval_s * 1000.0)
        self.probe_started_ms = 0
        self.probe_fresh_since_ms = None

    def ensure_probe_mode(self, reason: str) -> None:
        self.fallback_controller.stop()
        self.fallback_detector.stop()
        safe_unlink(self.args.fallback_state_file)
        self.sender.start(self.build_sender_cmd())
        self.phase = "PROBE"
        self.phase_reason = reason
        self.phase_entered_ms = monotonic_ms()
        self.probe_started_ms = monotonic_ms()
        self.probe_fresh_since_ms = None

    def bridge_health(self) -> tuple[bool, int, str]:
        bridge = self.load_json(self.args.bridge_status_file) or {}
        age = int(bridge.get("message_age_ms", 10**9) or 10**9)
        fresh = bool(bridge.get("fresh", False))
        reason = str(bridge.get("reason", "missing"))
        return fresh, age, reason

    def explicit_fallback_requested(self, now_ms: int) -> bool:
        if not self.last_mode_ms:
            return False
        if (now_ms - self.last_mode_ms) > int(self.args.mode_timeout_ms):
            return False
        return self.mode == MODE_FALLBACK_LOCAL

    def write_status(self, fresh: bool, age_ms: int, bridge_reason: str) -> None:
        fallback_state = self.load_json(self.args.fallback_state_file)
        fallback_state_ready = self.fallback_state_ready()
        fallback_state_age_ms = None
        if isinstance(fallback_state, dict):
            fallback_state_age_ms = int(monotonic_ms() - int(fallback_state.get("recv_monotonic_ms", 0) or 0))
        payload = {
            "phase": self.phase,
            "phase_reason": self.phase_reason,
            "phase_age_ms": int(monotonic_ms() - int(self.phase_entered_ms)),
            "mode": self.mode,
            "sender_pid": self.sender.pid(),
            "fallback_detector_pid": self.fallback_detector.pid(),
            "fallback_controller_pid": self.fallback_controller.pid(),
            "fallback_state_present": bool(fallback_state),
            "fallback_state_ready": bool(fallback_state_ready),
            "fallback_state_age_ms": fallback_state_age_ms,
            "bridge_fresh": bool(fresh),
            "bridge_age_ms": int(age_ms),
            "bridge_reason": bridge_reason,
            "probe_started_ms": int(self.probe_started_ms),
            "next_probe_ms": int(self.next_probe_ms),
            "ts_monotonic_ms": int(monotonic_ms()),
        }
        atomic_write_json(self.args.status_file, payload)

    def fallback_state_ready(self) -> bool:
        payload = self.load_json(self.args.fallback_state_file)
        if not isinstance(payload, dict):
            return False
        if str(payload.get("source", "")) != "fallback_local_detector":
            return False
        recv_ms = int(payload.get("recv_monotonic_ms", 0) or 0)
        age_ms = int(monotonic_ms() - recv_ms)
        return age_ms <= int(self.args.fallback_state_ready_max_age_ms)

    def on_timer(self) -> None:
        now_ms = monotonic_ms()
        fresh, age_ms, bridge_reason = self.bridge_health()
        explicit_fallback = self.explicit_fallback_requested(now_ms)
        fallback_ready = self.fallback_state_ready()
        stale_for_fallback = (now_ms > self.startup_deadline_ms) and (not fresh) and age_ms > int(self.args.fallback_timeout_ms)

        if self.phase == "NORMAL":
            if explicit_fallback or stale_for_fallback:
                self.ensure_fallback_mode("explicit_mode" if explicit_fallback else f"bridge_stale:{bridge_reason}")
        elif self.phase == "FALLBACK":
            fallback_phase_age_ms = now_ms - int(self.phase_entered_ms)
            if (not self.fallback_detector.running()) or (not self.fallback_controller.running()):
                self.ensure_fallback_mode("fallback_proc_restart")
            elif (not fallback_ready) and fallback_phase_age_ms >= int(self.args.fallback_state_stale_timeout_ms):
                self.ensure_fallback_mode("fallback_state_stale_restart")
            elif now_ms >= self.next_probe_ms:
                if bridge_reason == "missing":
                    self.phase_reason = "bridge_missing_hold_fallback"
                    self.next_probe_ms = now_ms + int(self.args.probe_interval_s * 1000.0)
                elif fallback_ready:
                    self.ensure_probe_mode("periodic_probe")
                else:
                    self.phase_reason = "waiting_fallback_state"
                    self.next_probe_ms = now_ms + int(self.args.fallback_ready_retry_ms)
        elif self.phase == "PROBE":
            if fresh and age_ms <= int(self.args.fresh_timeout_ms):
                if self.probe_fresh_since_ms is None:
                    self.probe_fresh_since_ms = now_ms
                elif (now_ms - self.probe_fresh_since_ms) >= int(self.args.recover_stable_ms):
                    self.ensure_normal_mode("bridge_recovered")
            else:
                self.probe_fresh_since_ms = None
                if (now_ms - self.probe_started_ms) >= int(self.args.probe_window_s * 1000.0):
                    self.ensure_fallback_mode(f"probe_failed:{bridge_reason}")

        self.write_status(fresh, age_ms, bridge_reason)

    def shutdown_all(self) -> None:
        self.sender.stop()
        self.fallback_detector.stop()
        self.fallback_controller.stop()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Manage sender/fallback-local detector camera ownership.")
    parser.add_argument("--face-tools-dir", default="/root/face_tools")
    parser.add_argument("--ros-py", default="/usr/bin/python3")
    parser.add_argument("--bridge-status-file", default="/tmp/pc_vision_bridge_status.json")
    parser.add_argument("--status-file", default="/tmp/fallback_local_manager_status.json")
    parser.add_argument("--mode-topic", default="/security/motion_mode")
    parser.add_argument("--mode-timeout-ms", type=int, default=1000)
    parser.add_argument("--sender-output-url", default="tcp://192.168.5.15:5601")
    parser.add_argument("--camera-arg", action="append", default=[])
    parser.add_argument("--sender-log", default="/tmp/car_sender.log")
    parser.add_argument("--fallback-detector-log", default="/tmp/fallback_local_detector.log")
    parser.add_argument("--fallback-controller-log", default="/tmp/fallback_local_controller.log")
    parser.add_argument("--fallback-state-file", default="/tmp/local_yolov5_state.json")
    parser.add_argument("--fallback-controller-status-file", default="/tmp/fallback_follow_controller_state.json")
    parser.add_argument("--fallback-controller-event-log", default="/tmp/fallback_follow_controller_events.jsonl")
    parser.add_argument("--fallback-cmd-topic", default="/security/fallback_cmd_vel")
    parser.add_argument("--fallback-conf", type=float, default=0.25)
    parser.add_argument("--fallback-image-width", type=int, default=640)
    parser.add_argument("--fallback-image-height", type=int, default=480)
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--startup-grace-ms", type=int, default=5000)
    parser.add_argument("--fresh-timeout-ms", type=int, default=150)
    parser.add_argument("--fallback-timeout-ms", type=int, default=2000)
    parser.add_argument("--probe-interval-s", type=float, default=20.0)
    parser.add_argument("--probe-window-s", type=float, default=2.0)
    parser.add_argument("--recover-stable-ms", type=int, default=1000)
    parser.add_argument("--fallback-ready-retry-ms", type=int, default=3000)
    parser.add_argument("--fallback-state-ready-max-age-ms", type=int, default=1000)
    parser.add_argument("--fallback-state-stale-timeout-ms", type=int, default=25000)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    rclpy.init()
    node = FallbackLocalManager(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.shutdown_all()
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


