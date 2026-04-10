#!/usr/bin/env python3
import argparse
import copy
import errno
import hashlib
import json
import os
import tempfile
import time


FOLLOW_INPUT_SCHEMA_VERSION = 1
FOLLOW_CONTROLLER_SCHEMA_VERSION = 1
ARBITER_SCHEMA_VERSION = 1
SUPERVISOR_SCHEMA_VERSION = 1


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


def atomic_write_json(path, data, prefix=".init_protocol_", suffix=".json"):
    if not path:
        return
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    fd, tmp_path = tempfile.mkstemp(prefix=prefix, suffix=suffix, dir=directory)
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


def make_follow_input_state(width, height):
    now_ms = monotonic_ms()
    return {
        "schema_version": FOLLOW_INPUT_SCHEMA_VERSION,
        "follow_name": "",
        "ts_monotonic_ms": int(now_ms),
        "meta": {
            "write_ts_monotonic_ms": int(now_ms),
            "frame_id": 0,
            "seq": 0,
            "checksum_sha256": "",
        },
        "render_target": None,
        "control_target": {
            "active": False,
            "control_ok": False,
            "track_id": None,
            "name": "",
            "lost": 0,
            "stale": False,
            "matched_this_frame": False,
            "has_recent_face_evidence": False,
            "confidence": 0.0,
            "area_ratio": 0.0,
            "bbox_xyxy_norm": [0.0, 0.0, 0.0, 0.0],
            "cx": float(width) / 2.0,
            "cy": float(height) / 2.0,
            "width": int(width),
            "height": int(height),
            "seq": 0,
            "ts_monotonic_ms": int(now_ms),
        },
    }


def make_follow_controller_state():
    now_ms = monotonic_ms()
    return {
        "schema_version": FOLLOW_CONTROLLER_SCHEMA_VERSION,
        "ts_monotonic_ms": int(now_ms),
        "seq": 0,
        "control_action": "HOLD_NEUTRAL",
        "stop_reason": "TARGET_INACTIVE",
        "fresh_age_ms": 0,
        "cmd": {
            "speed": 1500.0,
            "angle": 90.0,
        },
        "target_snapshot": {},
        "obstacle": {},
        "counters": {
            "invalid_state_count": 0,
        },
        "meta": {
            "checksum_sha256": "",
        },
    }


def make_drive_arbiter_state(mode):
    now_ms = monotonic_ms()
    return {
        "schema_version": ARBITER_SCHEMA_VERSION,
        "ts_monotonic_ms": int(now_ms),
        "seq": 0,
        "mode": str(mode),
        "arbiter_action": "OUTPUT_NEUTRAL",
        "arbiter_reason": "MANUAL_STOP",
        "lock_active": False,
        "lock_remaining_ms": 0,
        "nav_age_ms": -1,
        "track_age_ms": -1,
        "cmd": {
            "linear_x": 1500.0,
            "angular_z": 90.0,
        },
        "meta": {
            "checksum_sha256": "",
        },
    }


def make_supervisor_state():
    now_ms = monotonic_ms()
    return {
        "schema_version": SUPERVISOR_SCHEMA_VERSION,
        "ts_monotonic_ms": int(now_ms),
        "seq": 0,
        "mode": "SAFE_STOP",
        "reason": "INIT_IDLE",
        "waypoint_index": 0,
        "nav_goal_index": -1,
        "nav_goal_active": False,
        "follow_pid": None,
        "light": "",
        "alert_start_ts_wall": 0.0,
        "last_alert_seen_wall": 0.0,
        "alert_anchor_pose": None,
        "meta": {
            "checksum_sha256": "",
        },
    }


def parse_args():
    ap = argparse.ArgumentParser(description="Write valid idle protocol state files for local tracking stack startup/cleanup.")
    ap.add_argument("--follow-input-state-file", default="")
    ap.add_argument("--follow-controller-state-file", default="")
    ap.add_argument("--drive-arbiter-state-file", default="")
    ap.add_argument("--supervisor-state-file", default="")
    ap.add_argument("--frame-width", type=int, default=1280)
    ap.add_argument("--frame-height", type=int, default=720)
    ap.add_argument("--arbiter-mode", default="SAFE_STOP")
    return ap.parse_args()


def main():
    args = parse_args()
    if args.follow_input_state_file:
        atomic_write_json(args.follow_input_state_file, make_follow_input_state(args.frame_width, args.frame_height))
    if args.follow_controller_state_file:
        atomic_write_json(args.follow_controller_state_file, make_follow_controller_state())
    if args.drive_arbiter_state_file:
        atomic_write_json(args.drive_arbiter_state_file, make_drive_arbiter_state(args.arbiter_mode))
    if args.supervisor_state_file:
        atomic_write_json(args.supervisor_state_file, make_supervisor_state())


if __name__ == "__main__":
    main()
