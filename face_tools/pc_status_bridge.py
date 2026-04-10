#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
import time
from pathlib import Path
from typing import Any
import zlib


def monotonic_ms() -> int:
    return time.perf_counter_ns() // 1_000_000


def canonical_payload(payload: dict[str, Any]) -> bytes:
    data = dict(payload)
    data.pop("payload_crc32", None)
    return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")


def compute_crc32(payload: dict[str, Any]) -> int:
    return zlib.crc32(canonical_payload(payload)) & 0xFFFFFFFF


def write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False), encoding="utf-8")
    tmp.replace(path)


class PcStatusBridge:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((args.listen_host, args.listen_port))
        self.sock.settimeout(0.1)

        self.state_path = Path(args.state_file)
        self.status_path = Path(args.status_file)
        self.last_session_id: str | None = None
        self.last_seq: int | None = None
        self.last_accept_monotonic_ms: int | None = None
        self.last_packet: dict[str, Any] | None = None
        self.checksum_fail_count = 0
        self.out_of_order_drop_count = 0
        self.decode_error_count = 0
        self.last_log_ts = 0.0

    def _inactive_state(self, now_ms: int, reason: str) -> dict[str, Any]:
        return {
            "active": False,
            "source": "pc_vision_pipeline",
            "session_id": self.last_session_id,
            "seq": self.last_seq,
            "frame_seq": -1,
            "recv_monotonic_ms": now_ms,
            "message_age_ms": None if self.last_accept_monotonic_ms is None else (now_ms - self.last_accept_monotonic_ms),
            "fresh": False,
            "stale_reason": reason,
            "mode": "SAFE_STOP",
            "track_id": -1,
            "identity": "none",
            "face_evidence_state": "none",
            "cx_norm": 0.5,
            "cy_norm": 0.5,
            "distance_proxy": 0.0,
            "confidence": 0.0,
            "top1_score": 0.0,
            "topk_score": 0.0,
            "match_count": 0,
            "checksum_ok": True,
        }

    def _status_payload(self, now_ms: int, reason: str, fresh: bool) -> dict[str, Any]:
        return {
            "last_session_id": self.last_session_id,
            "last_seq": self.last_seq,
            "last_accept_monotonic_ms": self.last_accept_monotonic_ms,
            "message_age_ms": None if self.last_accept_monotonic_ms is None else (now_ms - self.last_accept_monotonic_ms),
            "fresh": fresh,
            "reason": reason,
            "checksum_fail_count": self.checksum_fail_count,
            "out_of_order_drop_count": self.out_of_order_drop_count,
            "decode_error_count": self.decode_error_count,
            "last_packet": self.last_packet,
        }

    def _write_state(self, payload: dict[str, Any]) -> None:
        write_json(self.state_path, payload)

    def _write_status(self, payload: dict[str, Any]) -> None:
        write_json(self.status_path, payload)

    def _accept_packet(self, packet: dict[str, Any], recv_ms: int) -> None:
        session_id = str(packet.get("session_id", ""))
        seq = int(packet.get("seq", -1))

        if int(packet.get("schema_version", -1)) != 1:
            return
        if str(packet.get("source", "")) != "pc_vision_pipeline":
            return
        if not session_id:
            return
        if int(packet.get("payload_crc32", -1)) != compute_crc32(packet):
            self.checksum_fail_count += 1
            return

        if self.last_session_id != session_id:
            self.last_session_id = session_id
            self.last_seq = None

        if self.last_seq is not None and seq <= self.last_seq:
            self.out_of_order_drop_count += 1
            return

        self.last_seq = seq
        self.last_accept_monotonic_ms = recv_ms
        self.last_packet = packet
        state = {
            "active": bool(packet.get("target_present", False)),
            "source": str(packet.get("source", "pc_vision_pipeline")),
            "session_id": session_id,
            "seq": seq,
            "frame_seq": int(packet.get("frame_seq", -1)),
            "recv_monotonic_ms": recv_ms,
            "message_age_ms": 0,
            "fresh": True,
            "stale_reason": "",
            "mode": str(packet.get("mode", "PATROL")),
            "track_id": int(packet.get("target_id", -1)),
            "identity": str(packet.get("identity", "none")),
            "face_evidence_state": str(packet.get("face_evidence_state", "none")),
            "cx_norm": float(packet.get("cx_norm", 0.5)),
            "cy_norm": float(packet.get("cy_norm", 0.5)),
            "distance_proxy": float(packet.get("distance_proxy", 0.0)),
            "confidence": float(packet.get("confidence", 0.0)),
            "top1_score": float(packet.get("top1_score", 0.0)),
            "topk_score": float(packet.get("topk_score", 0.0)),
            "match_count": int(packet.get("match_count", 0)),
            "checksum_ok": True,
        }
        self._write_state(state)
        self._write_status(self._status_payload(recv_ms, "accepted", True))

    def _refresh_files(self, now_ms: int) -> None:
        if self.last_accept_monotonic_ms is None:
            self._write_state(self._inactive_state(now_ms, "waiting_for_first_packet"))
            self._write_status(self._status_payload(now_ms, "waiting_for_first_packet", False))
            return

        age_ms = now_ms - self.last_accept_monotonic_ms
        if age_ms <= int(self.args.fresh_timeout_ms):
            state = self._inactive_state(now_ms, "")
            if self.last_packet is not None:
                state.update(
                    {
                        "active": bool(self.last_packet.get("target_present", False)),
                        "source": str(self.last_packet.get("source", "pc_vision_pipeline")),
                        "session_id": self.last_session_id,
                        "seq": self.last_seq,
                        "frame_seq": int(self.last_packet.get("frame_seq", -1)),
                        "message_age_ms": age_ms,
                        "fresh": True,
                        "stale_reason": "",
                        "mode": str(self.last_packet.get("mode", "PATROL")),
                        "track_id": int(self.last_packet.get("target_id", -1)),
                        "identity": str(self.last_packet.get("identity", "none")),
                        "face_evidence_state": str(self.last_packet.get("face_evidence_state", "none")),
                        "cx_norm": float(self.last_packet.get("cx_norm", 0.5)),
                        "cy_norm": float(self.last_packet.get("cy_norm", 0.5)),
                        "distance_proxy": float(self.last_packet.get("distance_proxy", 0.0)),
                        "confidence": float(self.last_packet.get("confidence", 0.0)),
                        "top1_score": float(self.last_packet.get("top1_score", 0.0)),
                        "topk_score": float(self.last_packet.get("topk_score", 0.0)),
                        "match_count": int(self.last_packet.get("match_count", 0)),
                    }
                )
            self._write_state(state)
            self._write_status(self._status_payload(now_ms, "fresh", True))
            return

        if age_ms <= int(self.args.stale_timeout_ms):
            state = self._inactive_state(now_ms, "stale_display_only")
            if self.last_packet is not None:
                state.update(
                    {
                        "source": str(self.last_packet.get("source", "pc_vision_pipeline")),
                        "session_id": self.last_session_id,
                        "seq": self.last_seq,
                        "frame_seq": int(self.last_packet.get("frame_seq", -1)),
                        "message_age_ms": age_ms,
                        "fresh": False,
                        "stale_reason": "stale_display_only",
                        "mode": str(self.last_packet.get("mode", "PATROL")),
                    }
                )
            self._write_state(state)
            self._write_status(self._status_payload(now_ms, "stale_display_only", False))
            return

        self._write_state(self._inactive_state(now_ms, "message_timeout"))
        self._write_status(self._status_payload(now_ms, "message_timeout", False))

    def loop(self) -> int:
        print(
            f"[BRIDGE] listening udp://{self.args.listen_host}:{self.args.listen_port} "
            f"-> state_file={self.state_path} fresh_timeout_ms={self.args.fresh_timeout_ms} "
            f"stale_timeout_ms={self.args.stale_timeout_ms}"
        )
        while True:
            now_ms = monotonic_ms()
            try:
                payload, _addr = self.sock.recvfrom(65535)
                packet = json.loads(payload.decode("utf-8"))
                self._accept_packet(packet, recv_ms=now_ms)
            except socket.timeout:
                pass
            except Exception as exc:
                self.decode_error_count += 1
                if (time.time() - self.last_log_ts) >= 1.0:
                    print(f"[BRIDGE] decode error: {exc}")
                    self.last_log_ts = time.time()
            self._refresh_files(now_ms)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Receive PC vision UDP status and bridge it into local state files.")
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=8890)
    parser.add_argument("--state-file", default="/tmp/pc_vision_state.json")
    parser.add_argument("--status-file", default="/tmp/pc_vision_bridge_status.json")
    parser.add_argument("--fresh-timeout-ms", type=int, default=150)
    parser.add_argument("--stale-timeout-ms", type=int, default=300)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    bridge = PcStatusBridge(args)
    return bridge.loop()


if __name__ == "__main__":
    raise SystemExit(main())
