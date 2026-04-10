#!/usr/bin/env python3
import argparse
import copy
import errno
import hashlib
import json
import os
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path


SCHEMA_VERSION = 1
NEUTRAL_SPEED = 1500.0
NEUTRAL_ANGLE = 90.0

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

ARBITER_FORWARD_TRACK = "FORWARD_TRACK"
ARBITER_OUTPUT_NEUTRAL = "OUTPUT_NEUTRAL"
ARBITER_TRACK_ACTIVE = "TRACK_ACTIVE"
ARBITER_NO_FRESH_TRACK_CMD = "NO_FRESH_TRACK_CMD"


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


def atomic_write_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=".inject_", suffix=".json", dir=str(path.parent))
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


def read_json(path):
    try:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    except Exception:
        return None


def append_jsonl(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n")


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


class InjectionRunner:
    def __init__(self, args):
        self.args = args
        self.base_dir = Path(args.base_dir)
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.python = args.python_bin
        self.controller_script = args.controller_script
        self.arbiter_script = args.arbiter_script
        self.report_path = self.base_dir / "follow_injection_report.json"
        self.topic_info_path = self.base_dir / "car_cmd_vel_topic_info.txt"
        self.rosbag_dir = Path(args.bag_dir) if args.bag_dir else (self.base_dir / "rosbag")
        self.results = []
        self.rosbag_proc = None

    def file_paths(self, name):
        case_dir = self.base_dir / name
        case_dir.mkdir(parents=True, exist_ok=True)
        return {
            "case_dir": case_dir,
            "input_state": case_dir / "face_follow_state.json",
            "ctrl_state": case_dir / "follow_controller_state.json",
            "ctrl_events": case_dir / "follow_controller_events.jsonl",
            "arb_state": case_dir / "drive_arbiter_state.json",
            "arb_events": case_dir / "drive_arbiter_events.jsonl",
            "ctrl_stdout": case_dir / "follow_controller.log",
            "arb_stdout": case_dir / "drive_arbiter.log",
        }

    def cleanup_case_files(self, files):
        for path in files.values():
            if isinstance(path, Path):
                try:
                    if path.exists():
                        if path.is_file():
                            path.unlink()
                except OSError:
                    pass

    def spawn(self, argv, log_path):
        with open(log_path, "wb") as f:
            proc = subprocess.Popen(
                argv,
                stdout=f,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        return proc

    def stop_process(self, proc, label):
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
        except Exception as exc:
            append_jsonl(self.report_path.with_suffix(".jsonl"), {"event": "stop_error", "label": label, "error": str(exc)})

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
                "/car_cmd_vel",
                "/track_cmd_vel",
                "/drive_arbiter/state",
                "/follow_controller/state",
            ]
            log_path = self.base_dir / "rosbag_record.log"
            with open(log_path, "wb") as f:
                self.rosbag_proc = subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
            time.sleep(1.0)
        except Exception as exc:
            append_jsonl(self.report_path.with_suffix(".jsonl"), {"event": "rosbag_start_error", "error": str(exc)})
            self.rosbag_proc = None

    def stop_rosbag(self):
        if self.rosbag_proc is None:
            return
        self.stop_process(self.rosbag_proc, "rosbag")
        self.rosbag_proc = None

    def start_stack(self, files, case):
        self.cleanup_case_files(files)
        target_mode = case.get("controller_mode", "any")
        target_name = case.get("controller_name", "")
        controller = self.spawn(
            [
                self.python,
                self.controller_script,
                "--state-file",
                str(files["input_state"]),
                "--status-file",
                str(files["ctrl_state"]),
                "--event-log",
                str(files["ctrl_events"]),
                "--state-topic",
                "/follow_controller/state",
                "--target-mode",
                str(target_mode),
                "--target-name",
                str(target_name),
                "--cmd-topic",
                "/track_cmd_vel",
                "--disable-obstacle-avoidance",
                "--fresh-timeout-ms",
                str(self.args.fresh_timeout_ms),
                "--seq-stall-timeout-ms",
                str(self.args.seq_stall_timeout_ms),
            ],
            str(files["ctrl_stdout"]),
        )
        arbiter = self.spawn(
            [
                self.python,
                self.arbiter_script,
                "--mode-topic",
                "",
                "--default-mode",
                "ALERT_TRACK",
                "--track-topic",
                "/track_cmd_vel",
                "--output-topic",
                "/car_cmd_vel",
                "--status-file",
                str(files["arb_state"]),
                "--event-log",
                str(files["arb_events"]),
                "--state-topic",
                "/drive_arbiter/state",
                "--timeout-track-ms",
                str(self.args.track_timeout_ms),
            ],
            str(files["arb_stdout"]),
        )
        time.sleep(self.args.startup_wait_sec)
        return controller, arbiter

    def wait_for(self, predicate, timeout_sec):
        deadline = time.time() + timeout_sec
        last = None
        while time.time() < deadline:
            last = predicate()
            if last is not None:
                return last
            time.sleep(0.05)
        return None

    def build_valid_payload(self, seq, *, active=True, control_ok=True, lost=0, stale=False, matched=True, name="owner", area_ratio=0.02, fresh_offset_ms=0):
        now_ms = monotonic_ms() - int(fresh_offset_ms)
        control_target = {
            "active": bool(active),
            "control_ok": bool(control_ok),
            "track_id": 42,
            "name": str(name),
            "render_name": str(name),
            "lost": int(lost),
            "stale": bool(stale),
            "matched_this_frame": bool(matched),
            "has_recent_face_evidence": False if name == "unknown" else True,
            "confidence": 0.95,
            "score": 0.92,
            "area_ratio": float(area_ratio),
            "bbox_xyxy_norm": [0.35, 0.20, 0.65, 0.90],
            "cx": 320.0,
            "cy": 240.0,
            "width": 640,
            "height": 480,
            "seq": int(seq),
            "ts_monotonic_ms": int(now_ms),
        }
        payload = {
            "schema_version": SCHEMA_VERSION,
            "follow_name": "owner",
            "ts_monotonic_ms": int(now_ms),
            "meta": {
                "write_ts_monotonic_ms": int(now_ms),
                "frame_id": 1,
                "seq": int(seq),
                "checksum_sha256": "",
            },
            "render_target": None,
            "control_target": control_target,
        }
        return payload

    def write_case_payload(self, case, files):
        name = case["name"]
        if name == "state_read_error":
            try:
                files["input_state"].unlink(missing_ok=True)
            except TypeError:
                if files["input_state"].exists():
                    files["input_state"].unlink()
            return

        if name == "truncated_json":
            files["input_state"].write_text('{"schema_version": 1, "broken": ', encoding="utf-8")
            return

        if name == "seq_stalled":
            payload = self.build_valid_payload(seq=11, area_ratio=0.02)
            atomic_write_json(files["input_state"], payload)
            return

        payload = self.build_valid_payload(
            seq=case.get("seq", 1),
            area_ratio=case.get("area_ratio", 0.02),
            name=case.get("payload_name", "owner"),
        )

        if name == "schema_mismatch":
            payload["schema_version"] = 99
        elif name == "checksum_error":
            payload = apply_checksum(payload)
            payload["meta"]["checksum_sha256"] = "badchecksum"
            files["input_state"].write_text(json.dumps(payload, ensure_ascii=False, sort_keys=True), encoding="utf-8")
            return
        elif name == "target_inactive":
            payload["control_target"]["active"] = False
            payload["control_target"]["control_ok"] = False
        elif name == "control_not_ok":
            payload["control_target"]["control_ok"] = False
        elif name == "target_lost":
            payload["control_target"]["lost"] = 1
        elif name == "target_stale":
            payload["control_target"]["stale"] = True
        elif name == "target_not_matched":
            payload["control_target"]["matched_this_frame"] = False
        elif name == "target_timeout":
            old_ms = monotonic_ms() - int(self.args.fresh_timeout_ms + 250)
            payload["meta"]["write_ts_monotonic_ms"] = old_ms
            payload["control_target"]["ts_monotonic_ms"] = old_ms
        elif name == "unknown_without_face":
            payload["control_target"]["name"] = "unknown"
            payload["control_target"]["has_recent_face_evidence"] = False
        elif name == "valid_forward":
            pass

        atomic_write_json(files["input_state"], payload)

    def wait_for_case_result(self, case, files):
        expected_stop = case.get("expected_stop_reason")
        expected_action = case.get("expected_action")
        expect_neutral = bool(case.get("expect_neutral", True))
        expect_arbiter_action = case.get("expected_arbiter_action")
        min_speed = case.get("min_speed")

        def probe():
            ctrl = read_json(files["ctrl_state"])
            arb = read_json(files["arb_state"])
            if not ctrl or not arb:
                return None

            ctrl_stop = str(ctrl.get("stop_reason", ""))
            ctrl_action = str(ctrl.get("control_action", ""))
            arb_action = str(arb.get("arbiter_action", ""))
            arb_reason = str(arb.get("arbiter_reason", ""))
            arb_cmd = arb.get("cmd") or {}
            arb_speed = float(arb_cmd.get("linear_x", -1.0))
            arb_angle = float(arb_cmd.get("angular_z", -1.0))

            if expected_stop is not None and ctrl_stop != expected_stop:
                return None
            if expected_action is not None and ctrl_action != expected_action:
                return None
            if expect_arbiter_action is not None and arb_action != expect_arbiter_action:
                return None
            if expect_neutral and not (abs(arb_speed - NEUTRAL_SPEED) < 1e-6 and abs(arb_angle - NEUTRAL_ANGLE) < 1e-6):
                return None
            if min_speed is not None and not (arb_speed >= float(min_speed)):
                return None
            return {
                "controller": ctrl,
                "arbiter": arb,
                "arbiter_reason": arb_reason,
            }

        return self.wait_for(probe, case.get("timeout_sec", self.args.case_timeout_sec))

    def run_case(self, case):
        files = self.file_paths(case["name"])
        controller = arbiter = None
        started_ms = monotonic_ms()
        try:
            controller, arbiter = self.start_stack(files, case)
            if self.args.capture_topic_info and not self.topic_info_path.exists():
                capture_command_output(["ros2", "topic", "info", "/car_cmd_vel", "--verbose"], self.topic_info_path)
            self.write_case_payload(case, files)
            if case["name"] == "seq_stalled":
                time.sleep((self.args.seq_stall_timeout_ms + 120) / 1000.0)
            result = self.wait_for_case_result(case, files)
            passed = result is not None
            record = {
                "case": case["name"],
                "passed": bool(passed),
                "started_ms": int(started_ms),
                "finished_ms": int(monotonic_ms()),
                "expected_stop_reason": case.get("expected_stop_reason"),
                "expected_action": case.get("expected_action"),
                "expected_arbiter_action": case.get("expected_arbiter_action"),
            }
            if result is not None:
                record["controller"] = result["controller"]
                record["arbiter"] = result["arbiter"]
            else:
                record["controller"] = read_json(files["ctrl_state"])
                record["arbiter"] = read_json(files["arb_state"])
                record["follow_log_tail"] = files["ctrl_stdout"].read_text(encoding="utf-8", errors="replace")[-3000:] if files["ctrl_stdout"].exists() else ""
                record["arbiter_log_tail"] = files["arb_stdout"].read_text(encoding="utf-8", errors="replace")[-3000:] if files["arb_stdout"].exists() else ""
            self.results.append(record)
        finally:
            self.stop_process(controller, "controller")
            self.stop_process(arbiter, "arbiter")

    def cases(self):
        return [
            {
                "name": "valid_forward",
                "expected_action": ACTION_MOVE_FORWARD,
                "expected_stop_reason": STOP_NONE,
                "expected_arbiter_action": ARBITER_FORWARD_TRACK,
                "expect_neutral": False,
                "min_speed": 1500.1,
            },
            {"name": "state_read_error", "expected_action": ACTION_HOLD_NEUTRAL, "expected_stop_reason": STOP_STATE_READ_ERROR},
            {"name": "schema_mismatch", "expected_action": ACTION_HOLD_NEUTRAL, "expected_stop_reason": STOP_STATE_SCHEMA_MISMATCH},
            {"name": "checksum_error", "expected_action": ACTION_HOLD_NEUTRAL, "expected_stop_reason": STOP_STATE_CHECKSUM_ERROR},
            {"name": "truncated_json", "expected_action": ACTION_HOLD_NEUTRAL, "expected_stop_reason": STOP_STATE_READ_ERROR},
            {"name": "target_inactive", "expected_action": ACTION_HOLD_NEUTRAL, "expected_stop_reason": STOP_TARGET_INACTIVE},
            {"name": "control_not_ok", "expected_action": ACTION_HOLD_NEUTRAL, "expected_stop_reason": STOP_CONTROL_NOT_OK},
            {"name": "target_lost", "expected_action": ACTION_HOLD_NEUTRAL, "expected_stop_reason": STOP_TARGET_LOST},
            {"name": "target_stale", "expected_action": ACTION_HOLD_NEUTRAL, "expected_stop_reason": STOP_TARGET_STALE},
            {"name": "target_not_matched", "expected_action": ACTION_HOLD_NEUTRAL, "expected_stop_reason": STOP_TARGET_NOT_MATCHED},
            {"name": "target_timeout", "expected_action": ACTION_HOLD_NEUTRAL, "expected_stop_reason": STOP_TARGET_TIMEOUT},
            {
                "name": "target_mode_mismatch",
                "expected_action": ACTION_HOLD_NEUTRAL,
                "expected_stop_reason": STOP_TARGET_MODE_MISMATCH,
                "controller_mode": "owner",
                "controller_name": "owner",
                "payload_name": "guest",
            },
            {
                "name": "unknown_without_face",
                "expected_action": ACTION_HOLD_NEUTRAL,
                "expected_stop_reason": STOP_UNKNOWN_NO_FACE_EVIDENCE,
                "controller_mode": "unknown",
            },
            {"name": "seq_stalled", "expected_action": ACTION_HOLD_NEUTRAL, "expected_stop_reason": STOP_STATE_SEQ_STALLED, "timeout_sec": 4.0},
        ]

    def run(self):
        self.start_rosbag()
        try:
            for case in self.cases():
                self.run_case(case)
        finally:
            self.stop_rosbag()
        summary = {
            "schema_version": SCHEMA_VERSION,
            "ts_monotonic_ms": monotonic_ms(),
            "passed": all(item.get("passed", False) for item in self.results),
            "topic_info_path": str(self.topic_info_path) if self.args.capture_topic_info else "",
            "rosbag_dir": str(self.rosbag_dir) if self.args.record_bag else "",
            "results": self.results,
        }
        self.report_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        print(json.dumps(summary, ensure_ascii=False, indent=2))
        return 0 if summary["passed"] else 1


def parse_args():
    ap = argparse.ArgumentParser(description="Run protocol-level follow state injection tests against controller + arbiter.")
    ap.add_argument("--base-dir", default="/tmp/follow_injection_suite")
    ap.add_argument("--python-bin", default=sys.executable)
    ap.add_argument("--controller-script", default="/root/face_tools/face_follow_controller.py")
    ap.add_argument("--arbiter-script", default="/root/face_tools/drive_arbiter.py")
    ap.add_argument("--startup-wait-sec", type=float, default=1.2)
    ap.add_argument("--case-timeout-sec", type=float, default=3.0)
    ap.add_argument("--fresh-timeout-ms", type=int, default=220)
    ap.add_argument("--seq-stall-timeout-ms", type=int, default=180)
    ap.add_argument("--track-timeout-ms", type=int, default=500)
    ap.add_argument("--record-bag", action="store_true")
    ap.add_argument("--bag-dir", default="")
    ap.add_argument("--capture-topic-info", action="store_true")
    return ap.parse_args()


def main():
    args = parse_args()
    runner = InjectionRunner(args)
    raise SystemExit(runner.run())


if __name__ == "__main__":
    main()
