#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import math

class ObstacleAlarmNode(Node):
    def __init__(self):
        super().__init__('obstacle_alarm_node')

        # 订阅激光雷达数据
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # 发布报警
        self.alarm_pub = self.create_publisher(
            Bool,
            '/alarm_trigger',
            10
        )

        # 报警消息发布
        self.alarm_msg_pub = self.create_publisher(
            String,
            '/alarm_message',
            10
        )

        self.get_logger().info('障碍物检测报警节点已启动')

        # 参数
        self.alarm_distance = 0.5  # 50cm内触发报警

    def scan_callback(self, msg):
        # 检查前方是否有障碍物
        front_angles = range(-30, 31)  # 前方±30度
        min_distance = float('inf')

        for i, angle in enumerate(front_angles):
            if i < len(msg.ranges):
                distance = msg.ranges[i]
                if distance < min_distance and distance > 0.1:  # 过滤无效值
                    min_distance = distance

        # 如果距离太近，触发报警
        if min_distance < self.alarm_distance:
            self.get_logger().warn(f'检测到障碍物！距离: {min_distance:.2f}m')

            # 发布报警
            alarm_msg = Bool()
            alarm_msg.data = True
            self.alarm_pub.publish(alarm_msg)

            # 发布报警消息
            msg_str = String()
            msg_str.data = f'前方障碍物！距离{min_distance:.2f}米'
            self.alarm_msg_pub.publish(msg_str)

def main():
    rclpy.init()
    node = ObstacleAlarmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
