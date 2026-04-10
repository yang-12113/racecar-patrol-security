#!/usr/bin/env python3
#coding:UTF-8

import math
import time
from typing import Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
import serial
from serial import SerialException

C = 227.45 / 1000
FRAME_LEN = 7
HEADER = b"\x01\x03\x02"
ANGLE_REQUEST = bytes.fromhex("01 03 00 03 00 01 74 0A")
POSITION_REQUEST = bytes.fromhex("01 03 00 00 00 01 84 0A")


def modbus_crc(payload: bytes) -> int:
    crc = 0xFFFF
    for byte in payload:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def signed_int16(value: int) -> int:
    if value >= 0x8000:
        value -= 0x10000
    return value


def wrap_position_delta(delta: int) -> int:
    if delta > 32767:
        delta -= 65536
    elif delta < -32768:
        delta += 65536
    return delta


class EncoderNode(Node):
    def __init__(self):
        super().__init__("encoder_vel")
        self.pub = self.create_publisher(Odometry, "encoder", 10)
        self.odom_sub = self.create_subscription(Odometry, "encoder_imu_odom", self._odom_callback, 10)
        self.port = self.declare_parameter("serial_port", "/dev/encoder").value
        self.baud = self.declare_parameter("baud_rate", 57600).value
        self.k = self.declare_parameter("k", 1).value
        self.read_timeout_sec = float(self.declare_parameter("read_timeout_sec", 0.06).value)
        self.timer_period_sec = float(self.declare_parameter("timer_period_sec", 0.02).value)
        self.keepalive_period_sec = float(self.declare_parameter("keepalive_period_sec", 0.10).value)
        self.max_sample_dt_sec = float(self.declare_parameter("max_sample_dt_sec", 0.20).value)
        self.motion_diag_speed_threshold = float(self.declare_parameter("motion_diag_speed_threshold", 0.05).value)
        self.motion_diag_pose_epsilon = float(self.declare_parameter("motion_diag_pose_epsilon", 0.02).value)
        self.motion_diag_window_sec = float(self.declare_parameter("motion_diag_window_sec", 0.50).value)
        self.ser = None
        self._last_open_attempt = 0.0
        self._last_warn = {}
        self._last_publish_time = 0.0
        self._last_speed_sample_time = None
        self._last_position = None
        self._last_direction = 1
        self._has_good_sample = False
        self._latest_odom_pose = None
        self._latest_odom_rx_time = None
        self._motion_diag_anchor_pose = None
        self._motion_diag_start_time = None
        self._connect_serial()
        self.timer = self.create_timer(self.timer_period_sec, self.timer_callback)

    def _warn_throttled(self, key, message, period=1.0):
        now = time.monotonic()
        last = self._last_warn.get(key, 0.0)
        if now - last >= period:
            self._last_warn[key] = now
            self.get_logger().warning(message)

    def _close_serial(self):
        if self.ser is None:
            return
        try:
            if self.ser.is_open:
                self.ser.close()
        except SerialException:
            pass
        self.ser = None

    def _connect_serial(self):
        if self.ser is not None and self.ser.is_open:
            return True

        now = time.monotonic()
        if now - self._last_open_attempt < 1.0:
            return False

        self._last_open_attempt = now
        self._close_serial()
        serial_kwargs = {
            "timeout": 0.01,
            "write_timeout": 0.05,
            "inter_byte_timeout": 0.01,
        }
        try:
            try:
                self.ser = serial.Serial(
                    self.port,
                    self.baud,
                    exclusive=True,
                    **serial_kwargs,
                )
            except TypeError:
                self.ser = serial.Serial(
                    self.port,
                    self.baud,
                    **serial_kwargs,
                )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"Serial port {self.port} opened: {self.ser.is_open}")
            return True
        except (SerialException, ValueError) as exc:
            self._warn_throttled("open", f"encoder serial reopen failed: {exc}")
            self.ser = None
            return False

    def _odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if not math.isfinite(x) or not math.isfinite(y):
            return
        self._latest_odom_pose = (x, y)
        self._latest_odom_rx_time = time.monotonic()

    def _publish_velocity(self, vel: float):
        pub_vel = Odometry()
        pub_vel.header.frame_id = "odom"
        pub_vel.child_frame_id = "base_footprint"
        pub_vel.header.stamp = self.get_clock().now().to_msg()
        pub_vel.twist.twist.linear.x = vel
        self.pub.publish(pub_vel)
        self._last_publish_time = time.monotonic()

    def _reset_motion_diagnostic(self):
        self._motion_diag_anchor_pose = None
        self._motion_diag_start_time = None

    def _check_motion_consistency(self, vel: float):
        if abs(vel) < self.motion_diag_speed_threshold:
            self._reset_motion_diagnostic()
            return

        now = time.monotonic()
        if self._latest_odom_pose is None or self._latest_odom_rx_time is None:
            return

        if now - self._latest_odom_rx_time > self.motion_diag_window_sec:
            self._warn_throttled(
                "odom_diag_stale",
                "encoder speed is non-zero but encoder_imu_odom has not updated recently; check IMU/odom fusion",
            )
            self._reset_motion_diagnostic()
            return

        if self._motion_diag_anchor_pose is None or self._motion_diag_start_time is None:
            self._motion_diag_anchor_pose = self._latest_odom_pose
            self._motion_diag_start_time = now
            return

        moved = math.hypot(
            self._latest_odom_pose[0] - self._motion_diag_anchor_pose[0],
            self._latest_odom_pose[1] - self._motion_diag_anchor_pose[1],
        )
        elapsed = now - self._motion_diag_start_time
        if moved > self.motion_diag_pose_epsilon:
            self._motion_diag_anchor_pose = self._latest_odom_pose
            self._motion_diag_start_time = now
            return

        if elapsed >= self.motion_diag_window_sec:
            self._warn_throttled(
                "odom_diag_stuck",
                f"encoder speed {vel:.3f} m/s but encoder_imu_odom moved only {moved:.3f} m in "
                f"{elapsed:.2f}s; check IMU/odom fusion chain",
            )
            self._motion_diag_anchor_pose = self._latest_odom_pose
            self._motion_diag_start_time = now

    def _publish_keepalive_if_needed(self):
        if not self._has_good_sample:
            self._warn_throttled(
                "wait_first_sample",
                "encoder not ready yet, skip zero-velocity keepalive until first valid sample",
                period=2.0,
            )
            return
        now = time.monotonic()
        if now - self._last_publish_time < self.keepalive_period_sec:
            return
        self._reset_motion_diagnostic()
        self._warn_throttled(
            "keepalive",
            "encoder input unstable, publish zero-velocity keepalive to protect TF",
            period=2.0,
        )
        self._publish_velocity(0.0)

    def _read_response_frame(self) -> Optional[bytes]:
        deadline = time.monotonic() + self.read_timeout_sec
        frame = bytearray()
        while time.monotonic() < deadline:
            need = FRAME_LEN - len(frame)
            try:
                chunk = self.ser.read(need)
            except (SerialException, OSError) as exc:
                self._warn_throttled("io", f"encoder serial I/O error: {exc}")
                self._close_serial()
                return None

            if not chunk:
                continue

            frame.extend(chunk)
            if len(frame) >= len(HEADER):
                if frame[:len(HEADER)] != HEADER:
                    header_index = frame.find(HEADER, 1)
                    if header_index >= 0:
                        del frame[:header_index]
                    else:
                        frame.clear()
                    continue

            if len(frame) >= FRAME_LEN:
                return bytes(frame[:FRAME_LEN])

        if frame:
            self._warn_throttled("short", f"encoder short frame: expected 7 bytes, got {len(frame)}")
        else:
            self._warn_throttled("timeout", "encoder response timeout")
        return None

    def _query_register(self, request_bytes: bytes, label: str) -> Optional[bytes]:
        if not self._connect_serial():
            return None

        for attempt in range(2):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(request_bytes)
                self.ser.flush()
            except (SerialException, OSError) as exc:
                self._warn_throttled("io", f"encoder serial I/O error: {exc}")
                self._close_serial()
                return None

            frame = self._read_response_frame()
            if frame is None:
                continue

            crc_expected = frame[5] | (frame[6] << 8)
            crc_actual = modbus_crc(frame[:5])
            if crc_actual != crc_expected:
                self._warn_throttled(
                    f"crc_{label}",
                    f"encoder {label} CRC mismatch: expected 0x{crc_expected:04X}, got 0x{crc_actual:04X}",
                )
                continue
            return frame

        self._warn_throttled(f"decode_{label}", f"encoder {label} frame decode failed, skip this cycle")
        return None

    def _decode_signed_register(self, frame: bytes) -> int:
        return signed_int16((frame[3] << 8) | frame[4])

    def _decode_position_register(self, frame: bytes) -> int:
        return (frame[3] << 8) | frame[4]

    def _direction_from_position(self, position: int, fallback_ticks: int) -> int:
        if self._last_position is None:
            self._last_position = position
            if fallback_ticks < 0:
                self._last_direction = -1
            elif fallback_ticks > 0:
                self._last_direction = 1
            return self._last_direction

        delta = wrap_position_delta(position - self._last_position)
        self._last_position = position
        if delta > 0:
            self._last_direction = 1
        elif delta < 0:
            self._last_direction = -1
        elif fallback_ticks < 0:
            self._last_direction = -1
        elif fallback_ticks > 0:
            self._last_direction = 1
        return self._last_direction

    def _sample_dt(self) -> float:
        now = time.monotonic()
        if self._last_speed_sample_time is None:
            self._last_speed_sample_time = now
            return self.timer_period_sec

        dt = now - self._last_speed_sample_time
        self._last_speed_sample_time = now
        if dt <= 0.0 or dt > self.max_sample_dt_sec:
            self._warn_throttled("dt", f"encoder sample dt abnormal: {dt:.3f}s, fallback to {self.timer_period_sec:.3f}s")
            return self.timer_period_sec
        return dt

    def timer_callback(self):
        angle_frame = self._query_register(ANGLE_REQUEST, "angle")
        if angle_frame is None:
            self._reset_motion_diagnostic()
            self._publish_keepalive_if_needed()
            return

        position_frame = self._query_register(POSITION_REQUEST, "position")
        if position_frame is None:
            self._reset_motion_diagnostic()
            self._publish_keepalive_if_needed()
            return

        angle_ticks = self._decode_signed_register(angle_frame)
        position = self._decode_position_register(position_frame)
        direction = self._direction_from_position(position, angle_ticks)
        dt = self._sample_dt()
        vel = 3.57 * angle_ticks * C / 1024.0 / dt * self.k * 0.25 * direction

        if not math.isfinite(vel):
            self._warn_throttled("vel", "encoder velocity became invalid, publish keepalive instead")
            self._reset_motion_diagnostic()
            self._publish_keepalive_if_needed()
            return

        self._has_good_sample = True
        self._check_motion_consistency(vel)
        self._publish_velocity(vel)

    def destroy_node(self):
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
