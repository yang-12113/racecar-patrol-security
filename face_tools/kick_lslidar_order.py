#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8

class KickNode(Node):
    def __init__(self):
        super().__init__('kick_lslidar_order_once')
        self.pub = self.create_publisher(Int8, '/lslidar_order', 10)


def main():
    rclpy.init()
    node = KickNode()
    msg = Int8()
    msg.data = 1
    end = time.time() + 1.5
    while time.time() < end:
        node.pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

