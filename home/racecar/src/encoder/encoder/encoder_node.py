#!/usr/bin/env python3
#coding:UTF-8

import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
import serial
from serial import SerialException

Bytenum_vel = 0
Bytenum_dir = 0
last_pose = 0
C = 227.45 / 1000


def DueVelData(inputdata):
    global Bytenum_vel

    for data in inputdata:
        if data == 0x01 and Bytenum_vel == 0:
            Bytenum_vel = 1
            continue
        if data == 0x03 and Bytenum_vel == 1:
            Bytenum_vel = 2
            continue
        if data == 0x02 and Bytenum_vel == 2:
            Bytenum_vel = 3
            continue
        if Bytenum_vel == 3:
            data_high = data
            Bytenum_vel = 4
            continue
        if Bytenum_vel == 4:
            data_low = data
            Bytenum_vel = 0
            angle_vel = data_high * 256 + data_low

            if angle_vel >= 32768:
                angle_vel -= 65536

            return float(angle_vel)

    return None


def DueDirData(inputdata):
    global Bytenum_dir
    global last_pose

    for data in inputdata:
        if data == 0x01 and Bytenum_dir == 0:
            Bytenum_dir = 1
            continue
        if data == 0x03 and Bytenum_dir == 1:
            Bytenum_dir = 2
            continue
        if data == 0x02 and Bytenum_dir == 2:
            Bytenum_dir = 3
            continue
        if Bytenum_dir == 3:
            data_high = data
            Bytenum_dir = 4
            continue
        if Bytenum_dir == 4:
            data_low = data
            Bytenum_dir = 0
            position = data_high * 256 + data_low
            pose = position - last_pose
            last_pose = position

            if (pose >= 0 and pose < 512) or (pose > -1024 and pose < -512):
                return 1
            if (pose < 0 and pose > -512) or (pose < 1024 and pose > 512):
                return -1

    return None


class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_vel')
        self.pub = self.create_publisher(Odometry, 'encoder', 10)
        self.port = self.declare_parameter('serial_port', '/dev/encoder').value
        self.baud = self.declare_parameter('baud_rate', 57600).value
        self.k = self.declare_parameter('k', 1).value
        self.ser = None
        self._last_open_attempt = 0.0
        self._last_warn = {}
        self._connect_serial()
        self.timer = self.create_timer(0.02, self.timer_callback)

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
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05, write_timeout=0.05)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f'Serial port {self.port} opened: {self.ser.is_open}')
            return True
        except SerialException as exc:
            self._warn_throttled('open', f'encoder serial reopen failed: {exc}')
            self.ser = None
            return False

    def _query_register(self, request_hex):
        if not self._connect_serial():
            return None

        try:
            self.ser.write(bytes.fromhex(request_hex))
            datahex = self.ser.read(7)
        except (SerialException, OSError) as exc:
            self._warn_throttled('io', f'encoder serial I/O error: {exc}')
            self._close_serial()
            return None

        if len(datahex) != 7:
            self._warn_throttled('short', f'encoder short frame: expected 7 bytes, got {len(datahex)}')
            try:
                self.ser.reset_input_buffer()
            except SerialException:
                self._close_serial()
            return None

        return datahex

    def timer_callback(self):
        angle_frame = self._query_register('01 03 00 03 00 01 74 0A')
        if angle_frame is None:
            return
        angle_v = DueVelData(angle_frame)

        direction_frame = self._query_register('01 03 00 00 00 01 84 0A')
        if direction_frame is None:
            return
        direction = DueDirData(direction_frame)

        if angle_v is None or direction is None:
            self._warn_throttled('decode', 'encoder frame decode failed, skip this cycle')
            return

        vel = 3.57 * angle_v * C / 1024.0 / 0.02 * self.k * 0.25 * direction

        pub_vel = Odometry()
        pub_vel.header.frame_id = 'odom'
        pub_vel.child_frame_id = 'base_footprint'
        pub_vel.header.stamp = self.get_clock().now().to_msg()
        pub_vel.twist.twist.linear.x = vel
        self.pub.publish(pub_vel)

    def destroy_node(self):
        self._close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EncoderNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
