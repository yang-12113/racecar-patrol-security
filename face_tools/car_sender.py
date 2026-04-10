#!/usr/bin/env python3
from __future__ import annotations

import argparse
import shutil
import socket
import struct
import subprocess
import time
import zlib
from pathlib import Path

import cv2

FRAME_MAGIC = b"RCMJ"
FRAME_VERSION = 1
FRAME_HEADER_STRUCT = struct.Struct("!4sBHIQHHIIBBH")
FRAME_HEADER_LEN = FRAME_HEADER_STRUCT.size


def monotonic_ms() -> int:
    return time.perf_counter_ns() // 1_000_000


def try_run(cmd: list[str]) -> None:
    try:
        subprocess.run(cmd, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def apply_v4l2_controls(device_path: str, power_line: int, exposure_auto_priority: int) -> None:
    if shutil.which("v4l2-ctl") is None:
        return
    try_run(["v4l2-ctl", "-d", device_path, "-c", f"power_line_frequency={int(power_line)}"])
    try_run(["v4l2-ctl", "-d", device_path, "-c", f"exposure_auto_priority={int(exposure_auto_priority)}"])


def parse_indices(raw: str) -> list[int]:
    return [int(x.strip()) for x in str(raw).split(",") if x.strip()]


def choose_camera(preferred: int, indices: list[int], width: int, height: int, fps: float, fourcc: str) -> tuple[cv2.VideoCapture | None, int | None]:
    candidates = [preferred] + [idx for idx in indices if idx != preferred]
    for camera_idx in candidates:
        cap = cv2.VideoCapture(camera_idx)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        cap.set(cv2.CAP_PROP_FPS, float(fps))
        if fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        ok, frame = cap.read()
        if ok and frame is not None and frame.size > 0:
            return cap, camera_idx
        cap.release()
    return None, None


class FFmpegMpegTsSender:
    def __init__(self, output_url: str, width: int, height: int, fps: float, crf: int, preset: str, ffmpeg_bin: str = "ffmpeg") -> None:
        self.output_url = str(output_url)
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1.0, float(fps))
        self.crf = int(crf)
        self.preset = str(preset)
        self.ffmpeg_bin = str(ffmpeg_bin)
        self.proc: subprocess.Popen[bytes] | None = None
        self._start()

    def _start(self) -> None:
        cmd = [
            self.ffmpeg_bin,
            "-loglevel",
            "error",
            "-nostdin",
            "-fflags",
            "nobuffer",
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{self.width}x{self.height}",
            "-r",
            f"{self.fps:.3f}",
            "-i",
            "-",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            self.preset,
            "-tune",
            "zerolatency",
            "-bf",
            "0",
            "-g",
            str(max(1, int(round(self.fps)))),
            "-keyint_min",
            str(max(1, int(round(self.fps)))),
            "-pix_fmt",
            "yuv420p",
            "-crf",
            str(self.crf),
            "-flush_packets",
            "1",
            "-muxdelay",
            "0",
            "-muxpreload",
            "0",
            "-f",
            "mpegts",
            self.output_url,
        ]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, bufsize=0)

    def close(self) -> None:
        if self.proc is None:
            return
        try:
            if self.proc.stdin is not None:
                self.proc.stdin.close()
        except Exception:
            pass
        try:
            self.proc.terminate()
            self.proc.wait(timeout=1.0)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass
        finally:
            self.proc = None

    def write(self, frame) -> bool:
        if frame is None or frame.size == 0:
            return False
        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_LINEAR)
        if self.proc is None or self.proc.stdin is None:
            self.close()
            self._start()
        try:
            assert self.proc is not None and self.proc.stdin is not None
            self.proc.stdin.write(frame.tobytes())
            return True
        except Exception:
            self.close()
            return False


class DirectCameraMjpegReader:
    def __init__(self, device_path: str, width: int, height: int, fps: float, ffmpeg_bin: str = "ffmpeg") -> None:
        self.device_path = str(device_path)
        self.width = int(width)
        self.height = int(height)
        self.fps = max(1.0, float(fps))
        self.ffmpeg_bin = str(ffmpeg_bin)
        self.proc: subprocess.Popen[bytes] | None = None
        self.buffer = bytearray()

    def start(self) -> None:
        cmd = [
            self.ffmpeg_bin,
            "-hide_banner",
            "-loglevel",
            "error",
            "-nostdin",
            "-f",
            "v4l2",
            "-input_format",
            "mjpeg",
            "-video_size",
            f"{self.width}x{self.height}",
            "-framerate",
            f"{self.fps:.3f}",
            "-i",
            self.device_path,
            "-an",
            "-c:v",
            "copy",
            "-f",
            "mjpeg",
            "-",
        ]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, stdin=subprocess.DEVNULL, bufsize=0)
        self.buffer = bytearray()

    def close(self) -> None:
        if self.proc is None:
            return
        try:
            if self.proc.stdout is not None:
                self.proc.stdout.close()
        except Exception:
            pass
        try:
            self.proc.terminate()
            self.proc.wait(timeout=1.0)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass
        finally:
            self.proc = None

    def alive(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def next_jpeg(self) -> bytes | None:
        if self.proc is None or self.proc.stdout is None:
            return None
        while True:
            soi = self.buffer.find(b"\xff\xd8")
            if soi >= 0:
                if soi > 0:
                    del self.buffer[:soi]
                eoi = self.buffer.find(b"\xff\xd9", 2)
                if eoi >= 0:
                    jpeg = bytes(self.buffer[: eoi + 2])
                    del self.buffer[: eoi + 2]
                    return jpeg
            chunk = self.proc.stdout.read(65536)
            if not chunk:
                return None
            self.buffer.extend(chunk)


class FramedMjpgTcpSender:
    def __init__(self, output_url: str, connect_timeout: float = 2.0) -> None:
        self.output_url = str(output_url)
        self.connect_timeout = float(connect_timeout)
        self.sock: socket.socket | None = None
        self.seq = 0
        self.host, self.port = self._parse_output(self.output_url)

    @staticmethod
    def _parse_output(output_url: str) -> tuple[str, int]:
        if "://" in output_url:
            _, rest = output_url.split("://", 1)
        else:
            rest = output_url
        host, port = rest.rsplit(":", 1)
        return host, int(port)

    def connect(self) -> None:
        self.close()
        self.sock = socket.create_connection((self.host, self.port), timeout=self.connect_timeout)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def close(self) -> None:
        if self.sock is None:
            return
        try:
            self.sock.close()
        except Exception:
            pass
        finally:
            self.sock = None

    def send_jpeg(self, jpeg: bytes, width: int, height: int, quality: int = 90, flags: int = 0) -> bool:
        if not jpeg:
            return False
        try:
            if self.sock is None:
                self.connect()
        except Exception:
            self.close()
            return False
        self.seq += 1
        frame_ts_ms = monotonic_ms()
        crc32 = zlib.crc32(jpeg) & 0xFFFFFFFF
        header = FRAME_HEADER_STRUCT.pack(
            FRAME_MAGIC,
            FRAME_VERSION,
            FRAME_HEADER_LEN,
            int(self.seq),
            int(frame_ts_ms),
            int(width),
            int(height),
            int(len(jpeg)),
            int(crc32),
            int(max(0, min(100, quality))),
            int(flags),
            0,
        )
        try:
            assert self.sock is not None
            self.sock.sendall(header)
            self.sock.sendall(jpeg)
            return True
        except Exception:
            self.close()
            return False


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Low-latency camera sender for the PC-led vision pipeline.")
    parser.add_argument("--output-url", default="tcp://192.168.5.15:5601")
    parser.add_argument("--sender-mode", choices=["framed-mjpg-tcp", "mjpeg-tcp-copy", "x264-udp"], default="framed-mjpg-tcp")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--camera-scan", default="0,1,2,3")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--camera-fourcc", default="MJPG")
    parser.add_argument("--camera-power-line", type=int, default=1)
    parser.add_argument("--camera-exposure-auto-priority", type=int, default=0)
    parser.add_argument("--crf", type=int, default=20)
    parser.add_argument("--preset", default="ultrafast")
    parser.add_argument("--ffmpeg-bin", default="ffmpeg")
    parser.add_argument("--reconnect-delay", type=float, default=0.5)
    parser.add_argument("--status-every", type=float, default=3.0)
    return parser


def run_framed_mjpg(args: argparse.Namespace) -> int:
    scan_indices = parse_indices(args.camera_scan)
    reader: DirectCameraMjpegReader | None = None
    sender = FramedMjpgTcpSender(args.output_url)
    active_index = None
    frames = 0
    last_status = time.time()
    try:
        while True:
            if reader is None:
                active_index = None
                for camera_idx in [args.camera] + [idx for idx in scan_indices if idx != args.camera]:
                    device_path = f"/dev/video{camera_idx}"
                    if not Path(device_path).exists():
                        continue
                    apply_v4l2_controls(device_path, args.camera_power_line, args.camera_exposure_auto_priority)
                    probe = DirectCameraMjpegReader(device_path=device_path, width=args.width, height=args.height, fps=args.camera_fps, ffmpeg_bin=args.ffmpeg_bin)
                    probe.start()
                    time.sleep(1.0)
                    if probe.alive():
                        reader = probe
                        active_index = camera_idx
                        print(f"[SENDER] using {device_path} -> {args.output_url} mode=framed-mjpg-tcp")
                        break
                    probe.close()
                if reader is None:
                    print("[SENDER] framed-mjpg camera open failed, retrying...")
                    time.sleep(args.reconnect_delay)
                    continue

            jpeg = reader.next_jpeg() if reader is not None else None
            if not jpeg:
                print("[SENDER] camera mjpg stream stalled, reopening camera")
                if reader is not None:
                    reader.close()
                reader = None
                time.sleep(args.reconnect_delay)
                continue

            if not sender.send_jpeg(jpeg, args.width, args.height):
                print("[SENDER] tcp send failed, reconnecting socket")
                time.sleep(args.reconnect_delay)
                continue

            frames += 1
            now = time.time()
            if (now - last_status) >= max(0.5, float(args.status_every)):
                print(f"[SENDER] camera=/dev/video{active_index} frames={frames} output={args.output_url} mode=framed-mjpg-tcp")
                last_status = now
    finally:
        sender.close()
        if reader is not None:
            reader.close()

    return 0


def run_mjpeg_copy(args: argparse.Namespace) -> int:
    scan_indices = parse_indices(args.camera_scan)
    reader: DirectCameraMjpegReader | None = None
    sender = FramedMjpgTcpSender(args.output_url)
    active_index = None
    frames = 0
    last_status = time.time()
    try:
        while True:
            if reader is None:
                active_index = None
                for camera_idx in [args.camera] + [idx for idx in scan_indices if idx != args.camera]:
                    device_path = f"/dev/video{camera_idx}"
                    if not Path(device_path).exists():
                        continue
                    apply_v4l2_controls(device_path, args.camera_power_line, args.camera_exposure_auto_priority)
                    probe = DirectCameraMjpegReader(device_path=device_path, width=args.width, height=args.height, fps=args.camera_fps, ffmpeg_bin=args.ffmpeg_bin)
                    probe.start()
                    time.sleep(1.0)
                    if probe.alive():
                        reader = probe
                        active_index = camera_idx
                        print(f"[SENDER] using {device_path} -> {args.output_url} mode=mjpeg-tcp-copy")
                        break
                    probe.close()
                if reader is None:
                    print("[SENDER] direct camera sender failed to start, retrying...")
                    time.sleep(args.reconnect_delay)
                    continue

            jpeg = reader.next_jpeg() if reader is not None else None
            if not jpeg:
                print("[SENDER] direct mjpg stream stalled, reopening camera")
                if reader is not None:
                    reader.close()
                reader = None
                time.sleep(args.reconnect_delay)
                continue

            if not sender.send_jpeg(jpeg, args.width, args.height):
                print("[SENDER] tcp send failed, reconnecting socket")
                time.sleep(args.reconnect_delay)
                continue

            frames += 1
            now = time.time()
            if (now - last_status) >= max(0.5, float(args.status_every)):
                print(f"[SENDER] camera=/dev/video{active_index} frames={frames} output={args.output_url} mode=mjpeg-tcp-copy")
                last_status = now
    finally:
        sender.close()
        if reader is not None:
            reader.close()

    return 0


def run_x264_udp(args: argparse.Namespace) -> int:
    scan_indices = parse_indices(args.camera_scan)
    cap = None
    active_index = None
    sender = FFmpegMpegTsSender(
        args.output_url,
        width=args.width,
        height=args.height,
        fps=args.camera_fps,
        crf=args.crf,
        preset=args.preset,
        ffmpeg_bin=args.ffmpeg_bin,
    )

    frames = 0
    last_status = time.time()
    try:
        while True:
            if cap is None:
                cap, active_index = choose_camera(args.camera, scan_indices, args.width, args.height, args.camera_fps, args.camera_fourcc)
                if cap is None:
                    print("[SENDER] camera open failed, retrying...")
                    time.sleep(args.reconnect_delay)
                    continue
                device_path = f"/dev/video{active_index}"
                apply_v4l2_controls(device_path, args.camera_power_line, args.camera_exposure_auto_priority)
                print(f"[SENDER] using {device_path} -> {args.output_url} mode=x264-udp")

            ok, frame = cap.read()
            if not ok or frame is None or frame.size == 0:
                print("[SENDER] capture failed, reopening camera")
                cap.release()
                cap = None
                time.sleep(args.reconnect_delay)
                continue

            if not sender.write(frame):
                print("[SENDER] ffmpeg write failed, restarting sender")
                sender.close()
                sender = FFmpegMpegTsSender(
                    args.output_url,
                    width=args.width,
                    height=args.height,
                    fps=args.camera_fps,
                    crf=args.crf,
                    preset=args.preset,
                    ffmpeg_bin=args.ffmpeg_bin,
                )
                time.sleep(0.1)
                continue

            frames += 1
            now = time.time()
            if (now - last_status) >= max(0.5, float(args.status_every)):
                print(f"[SENDER] camera=/dev/video{active_index} frames={frames} output={args.output_url} mode=x264-udp")
                last_status = now
    finally:
        if cap is not None:
            cap.release()
        sender.close()

    return 0


def main() -> int:
    args = build_parser().parse_args()
    if args.sender_mode == "framed-mjpg-tcp":
        return run_framed_mjpg(args)
    if args.sender_mode == "mjpeg-tcp-copy":
        return run_mjpeg_copy(args)
    return run_x264_udp(args)


if __name__ == "__main__":
    raise SystemExit(main())
