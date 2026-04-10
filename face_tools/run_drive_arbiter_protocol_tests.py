#!/usr/bin/env python3
import argparse
import copy
import hashlib
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


SCHEMA_VERSION = 1

STATE_PATROL = "PATROL"
STATE_ALERT_TRACK = "ALERT_TRACK"
STATE_RETURN_TO_ROUTE = "RETURN_TO_ROUTE"
STATE_SAFE_STOP = "SAFE_STOP"

ACTION_OUTPUT_NEUTRAL = "OUTPUT_NEUTRAL"
ACTION_FORWARD_NAV = "FORWARD_NAV"
ACTION_FORWARD_TRACK = "FORWARD_TRACK"

REASON_STARTUP_LOCK = "STARTUP_LOCK"
REASON_MODE_SWITCH_LOCK = "MODE_SWITCH_LOCK"
REASON_SAFE_STOP_MODE = "SAFE_STOP_MODE"
REASON_MODE_TIMEOUT = "MODE_TIMEOUT"
REASON_NAV_TIMEOUT = "NAV_TIMEOUT"
REASON_TRACK_TIMEOUT = "TRACK_TIMEOUT"
REASON_NO_FRESH_NAV_CMD = "NO_FRESH_NAV_CMD"
REASON_NO_FRESH_TRACK_CMD = "NO_FRESH_TRACK_CMD"
REASON_PATROL_ACTIVE = "PATROL_ACTIVE"
REASON_RETURN_ACTIVE = "RETURN_ACTIVE"
REASON_TRACK_ACTIVE = "TRACK_ACTIVE"


def monotonic_ms():
    return int(time.monotonic_ns() // 1_000_000)


def canonical_json_bytes(data):
    return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


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


def read_json(path):
    try:
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        if isinstance(data, dict) and "meta" in data:
            if not verify_checksum(data):
                return None
        return data
    except Exception:
        return None


def capture_command_output(argv, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        result = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False, text=True)
        path.write_text(result.stdout, encoding="utf-8")
        return result.returncode
    except Exception as exc:
        path.write_text(f"failed to run {argv!r}: {exc}\n", encoding="utf-8")
        return 1


class ArbiterStimulus(Node):
    def __init__(self, mode_topic, nav_topic, track_topic):
        super().__init__("drive_arbiter_protocol_test_publisher")
        self.mode_pub = self.create_publisher(String, mode_topic, 10)
        self.nav_pub = self.create_publisher(Twist, nav_topic, 10)
        self.track_pub = self.create_publisher(Twist, track_topic, 10)
        self.seq = 0

    def publish_mode(self, mode, reason="test"):
        self.seq += 1
        msg = String()
        msg.data = json.dumps(
            {
                "schema_version": SCHEMA_VERSION,
                "seq": int(self.seq),
                "ts_monotonic_ms": int(monotonic_ms()),
                "mode": str(mode),
                "reason": str(reason),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.mode_pub.publish(msg)

    def publish_cmd(self, which, speed, angle):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(angle)
        if which == "nav":
            self.nav_pub.publish(msg)
        else:
            self.track_pub.publish(msg)


class DriveArbiterProtocolRunner:
    def __init__(self, args):
        self.args = args
        self.base_dir = Path(args.base_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.report_path = self.base_dir / "arbiter_protocol_report.json"
        self.topic_info_path = self.base_dir / "car_cmd_vel_topic_info.txt"
        self.rosbag_dir = Path(args.bag_dir) if args.bag_dir else (self.base_dir / "rosbag")
        self.results = []
        self.rosbag_proc = None

    def case_dir(self, name):
        case_dir = self.base_dir / name
        case_dir.mkdir(parents=True, exist_ok=True)
        return {
            "dir": case_dir,
            "status": case_dir / "drive_arbiter_state.json",
            "events": case_dir / "drive_arbiter_events.jsonl",
            "log": case_dir / "drive_arbiter.log",
        }

    def cleanup_case(self, files):
        for key, path in files.items():
            if key == "dir":
                continue
            try:
                if path.exists():
                    path.unlink()
            except OSError:
                pass

    def spawn_arbiter(self, files, extra_args=None):
        self.cleanup_case(files)
        argv = [
            self.args.python_bin,
            self.args.arbiter_script,
            "--mode-topic",
            self.args.mode_topic,
            "--nav-topic",
            self.args.nav_topic,
            "--track-topic",
            self.args.track_topic,
            "--output-topic",
            self.args.output_topic,
            "--state-topic",
            self.args.state_topic,
            "--status-file",
            str(files["status"]),
            "--event-log",
            str(files["events"]),
            "--timeout-mode-ms",
            str(self.args.timeout_mode_ms),
            "--timeout-nav-ms",
            str(self.args.timeout_nav_ms),
            "--timeout-track-ms",
            str(self.args.timeout_track_ms),
            "--lock-duration",
            str(self.args.lock_duration),
            "--startup-lock",
            str(self.args.startup_lock),
            "--rate",
            str(self.args.rate),
        ]
        if extra_args:
            argv.extend(extra_args)
        with open(files["log"], "wb") as f:
            proc = subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(self.args.startup_wait_sec)
        return proc

    def stop_process(self, proc):
        if proc is None:
            return
        try:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGTERM)
                try:
                    proc.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=2.0)
        except Exception:
            pass

    def start_rosbag(self):
        if not self.args.record_bag:
            return
        try:
            self.rosbag_dir.parent.mkdir(parents=True, exist_ok=True)
            argv = [
                "ros2",
                "bag",
                "record",
                "-o",
                str(self.rosbag_dir),
                str(self.args.output_topic),
                str(self.args.nav_topic),
                str(self.args.track_topic),
                str(self.args.mode_topic),
                str(self.args.state_topic),
            ]
            log_path = self.base_dir / "rosbag_record.log"
            with open(log_path, "wb") as f:
                self.rosbag_proc = subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(1.0)
        except Exception:
            self.rosbag_proc = None

    def stop_rosbag(self):
        if self.rosbag_proc is None:
            return
        self.stop_process(self.rosbag_proc)
        self.rosbag_proc = None

    def spin_tick(self, node, mode=None, cmd=None):
        if mode is not None:
            node.publish_mode(mode)
        if cmd is not None:
            node.publish_cmd(*cmd)
        rclpy.spin_once(node, timeout_sec=0.02)

    def wait_for_state(self, files, timeout_sec, tick=None):
        deadline = time.time() + timeout_sec
        last = None
        while time.time() < deadline:
            if tick is not None:
                tick()
            last = read_json(files["status"])
            if last is not None:
                return last
            time.sleep(0.02)
        return last

    def wait_match(self, files, timeout_sec, *, expected_action, expected_reason, tick=None):
        deadline = time.time() + timeout_sec
        last = None
        while time.time() < deadline:
            if tick is not None:
                tick()
            last = read_json(files["status"])
            if (
                last is not None
                and str(last.get("arbiter_action", "")) == expected_action
                and str(last.get("arbiter_reason", "")) == expected_reason
            ):
                return last
            time.sleep(0.02)
        return last

    def record(self, case, passed, state, files):
        self.results.append(
            {
                "case": case,
                "passed": bool(passed),
                "state": state,
                "status_file": str(files["status"]),
                "event_log": str(files["events"]),
                "log_file": str(files["log"]),
            }
        )

    def run_case(self, case_name, fn):
        files = self.case_dir(case_name)
        proc = None
        try:
            proc = self.spawn_arbiter(files, fn["extra_args"])
            if self.args.capture_topic_info and not self.topic_info_path.exists():
                capture_command_output(["ros2", "topic", "info", str(self.args.output_topic), "--verbose"], self.topic_info_path)
            state, passed = fn["runner"](files)
            self.record(case_name, passed, state, files)
        finally:
            self.stop_process(proc)

    def cases(self, stim):
        return [
            {
                "name": "startup_lock",
                "extra_args": ["--mode-topic", "", "--default-mode", STATE_ALERT_TRACK],
                "runner": lambda files: self._case_startup_lock(files),
            },
            {
                "name": "mode_timeout",
                "extra_args": [],
                "runner": lambda files: self._case_mode_timeout(files),
            },
            {
                "name": "no_fresh_nav_cmd",
                "extra_args": [],
                "runner": lambda files: self._case_no_fresh_nav_cmd(files, stim),
            },
            {
                "name": "nav_timeout",
                "extra_args": [],
                "runner": lambda files: self._case_nav_timeout(files, stim),
            },
            {
                "name": "patrol_active",
                "extra_args": [],
                "runner": lambda files: self._case_patrol_active(files, stim),
            },
            {
                "name": "return_active",
                "extra_args": [],
                "runner": lambda files: self._case_return_active(files, stim),
            },
            {
                "name": "no_fresh_track_cmd",
                "extra_args": [],
                "runner": lambda files: self._case_no_fresh_track_cmd(files, stim),
            },
            {
                "name": "track_timeout",
                "extra_args": [],
                "runner": lambda files: self._case_track_timeout(files, stim),
            },
            {
                "name": "track_active",
                "extra_args": [],
                "runner": lambda files: self._case_track_active(files, stim),
            },
            {
                "name": "safe_stop_mode",
                "extra_args": [],
                "runner": lambda files: self._case_safe_stop_mode(files, stim),
            },
            {
                "name": "mode_switch_lock",
                "extra_args": [],
                "runner": lambda files: self._case_mode_switch_lock(files, stim),
            },
        ]

    def _case_startup_lock(self, files):
        state = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_OUTPUT_NEUTRAL,
            expected_reason=REASON_STARTUP_LOCK,
        )
        return state, state is not None and str(state.get("arbiter_reason")) == REASON_STARTUP_LOCK

    def _case_mode_timeout(self, files):
        state = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_OUTPUT_NEUTRAL,
            expected_reason=REASON_MODE_TIMEOUT,
        )
        return state, state is not None and str(state.get("arbiter_reason")) == REASON_MODE_TIMEOUT

    def _case_no_fresh_nav_cmd(self, files, stim):
        def tick():
            self.spin_tick(stim, mode=STATE_PATROL)

        state = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_OUTPUT_NEUTRAL,
            expected_reason=REASON_NO_FRESH_NAV_CMD,
            tick=tick,
        )
        return state, state is not None and str(state.get("arbiter_reason")) == REASON_NO_FRESH_NAV_CMD

    def _case_nav_timeout(self, files, stim):
        def seed():
            self.spin_tick(stim, mode=STATE_PATROL, cmd=("nav", 1518.0, 90.0))

        active = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_FORWARD_NAV,
            expected_reason=REASON_PATROL_ACTIVE,
            tick=seed,
        )
        if active is None:
            return active, False

        def tick():
            self.spin_tick(stim, mode=STATE_PATROL)

        state = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_OUTPUT_NEUTRAL,
            expected_reason=REASON_NAV_TIMEOUT,
            tick=tick,
        )
        return state, state is not None and str(state.get("arbiter_reason")) == REASON_NAV_TIMEOUT

    def _case_patrol_active(self, files, stim):
        def tick():
            self.spin_tick(stim, mode=STATE_PATROL, cmd=("nav", 1518.0, 88.0))

        state = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_FORWARD_NAV,
            expected_reason=REASON_PATROL_ACTIVE,
            tick=tick,
        )
        return state, state is not None and str(state.get("arbiter_action")) == ACTION_FORWARD_NAV

    def _case_return_active(self, files, stim):
        def tick():
            self.spin_tick(stim, mode=STATE_RETURN_TO_ROUTE, cmd=("nav", 1518.0, 92.0))

        state = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_FORWARD_NAV,
            expected_reason=REASON_RETURN_ACTIVE,
            tick=tick,
        )
        return state, state is not None and str(state.get("arbiter_reason")) == REASON_RETURN_ACTIVE

    def _case_no_fresh_track_cmd(self, files, stim):
        def tick():
            self.spin_tick(stim, mode=STATE_ALERT_TRACK)

        state = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_OUTPUT_NEUTRAL,
            expected_reason=REASON_NO_FRESH_TRACK_CMD,
            tick=tick,
        )
        return state, state is not None and str(state.get("arbiter_reason")) == REASON_NO_FRESH_TRACK_CMD

    def _case_track_timeout(self, files, stim):
        def seed():
            self.spin_tick(stim, mode=STATE_ALERT_TRACK, cmd=("track", 1520.0, 90.0))

        active = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_FORWARD_TRACK,
            expected_reason=REASON_TRACK_ACTIVE,
            tick=seed,
        )
        if active is None:
            return active, False

        def tick():
            self.spin_tick(stim, mode=STATE_ALERT_TRACK)

        state = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_OUTPUT_NEUTRAL,
            expected_reason=REASON_TRACK_TIMEOUT,
            tick=tick,
        )
        return state, state is not None and str(state.get("arbiter_reason")) == REASON_TRACK_TIMEOUT

    def _case_track_active(self, files, stim):
        def tick():
            self.spin_tick(stim, mode=STATE_ALERT_TRACK, cmd=("track", 1516.0, 90.0))

        state = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_FORWARD_TRACK,
            expected_reason=REASON_TRACK_ACTIVE,
            tick=tick,
        )
        return state, state is not None and str(state.get("arbiter_action")) == ACTION_FORWARD_TRACK

    def _case_safe_stop_mode(self, files, stim):
        def tick():
            self.spin_tick(stim, mode=STATE_SAFE_STOP)

        state = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_OUTPUT_NEUTRAL,
            expected_reason=REASON_SAFE_STOP_MODE,
            tick=tick,
        )
        return state, state is not None and str(state.get("arbiter_reason")) == REASON_SAFE_STOP_MODE

    def _case_mode_switch_lock(self, files, stim):
        def start_patrol():
            self.spin_tick(stim, mode=STATE_PATROL, cmd=("nav", 1518.0, 90.0))

        active = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_FORWARD_NAV,
            expected_reason=REASON_PATROL_ACTIVE,
            tick=start_patrol,
        )
        if active is None:
            return active, False

        def switch_mode():
            self.spin_tick(stim, mode=STATE_ALERT_TRACK, cmd=("track", 1516.0, 90.0))

        state = self.wait_match(
            files,
            self.args.case_timeout_sec,
            expected_action=ACTION_OUTPUT_NEUTRAL,
            expected_reason=REASON_MODE_SWITCH_LOCK,
            tick=switch_mode,
        )
        return state, state is not None and bool(state.get("lock_active"))

    def run(self):
        rclpy.init()
        stim = ArbiterStimulus(self.args.mode_topic, self.args.nav_topic, self.args.track_topic)
        self.start_rosbag()
        try:
            for case in self.cases(stim):
                self.run_case(case["name"], case)
                time.sleep(0.2)
        finally:
            self.stop_rosbag()
            try:
                stim.destroy_node()
            except Exception:
                pass
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

        report = {
            "schema_version": SCHEMA_VERSION,
            "ts_monotonic_ms": monotonic_ms(),
            "passed": all(item.get("passed", False) for item in self.results),
            "topic_info_path": str(self.topic_info_path) if self.args.capture_topic_info else "",
            "rosbag_dir": str(self.rosbag_dir) if self.args.record_bag else "",
            "results": self.results,
        }
        self.report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if report["passed"] else 1


def parse_args():
    ap = argparse.ArgumentParser(description="Run drive_arbiter protocol tests.")
    ap.add_argument("--base-dir", default="/tmp/drive_arbiter_test_suite")
    ap.add_argument("--python-bin", default=sys.executable)
    ap.add_argument("--arbiter-script", default="/root/face_tools/drive_arbiter.py")
    ap.add_argument("--mode-topic", default="/test_drive_mode")
    ap.add_argument("--nav-topic", default="/test_nav_cmd_vel")
    ap.add_argument("--track-topic", default="/test_track_cmd_vel")
    ap.add_argument("--output-topic", default="/car_cmd_vel")
    ap.add_argument("--state-topic", default="/drive_arbiter/state")
    ap.add_argument("--startup-lock", type=float, default=0.45)
    ap.add_argument("--lock-duration", type=float, default=0.45)
    ap.add_argument("--timeout-mode-ms", type=int, default=800)
    ap.add_argument("--timeout-nav-ms", type=int, default=500)
    ap.add_argument("--timeout-track-ms", type=int, default=400)
    ap.add_argument("--rate", type=float, default=20.0)
    ap.add_argument("--startup-wait-sec", type=float, default=0.25)
    ap.add_argument("--case-timeout-sec", type=float, default=3.5)
    ap.add_argument("--record-bag", action="store_true")
    ap.add_argument("--bag-dir", default="")
    ap.add_argument("--capture-topic-info", action="store_true")
    return ap.parse_args()


def main():
    args = parse_args()
    runner = DriveArbiterProtocolRunner(args)
    raise SystemExit(runner.run())


if __name__ == "__main__":
    main()
