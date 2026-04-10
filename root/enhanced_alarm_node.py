#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
import subprocess
import threading
import time

class EnhancedAlarmNode(Node):
    def __init__(self):
        super().__init__('enhanced_alarm_node')

        # 订阅报警触发
        self.alarm_sub = self.create_subscription(
            Bool,
            '/alarm_trigger',
            self.alarm_callback,
            10
        )

        # 订阅报警消息
        self.alarm_msg_sub = self.create_subscription(
            String,
            '/alarm_message',
            self.alarm_message_callback,
            10
        )

        # 订阅速度命令（用于急停检测）
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 发布报警状态
        self.alarm_status_pub = self.create_publisher(
            Bool,
            '/alarm_status',
            10
        )

        self.get_logger().info('增强报警节点已启动')
        self.alarm_active = False
        self.last_cmd_time = time.time()

        # 创建定时器检查急停
        self.timer = self.create_timer(1.0, self.check_emergency_stop)

    def alarm_callback(self, msg):
        if msg.data and not self.alarm_active:
            self.trigger_alarm("通用报警")

    def alarm_message_callback(self, msg):
        if not self.alarm_active:
            self.trigger_alarm(msg.data)

    def cmd_vel_callback(self, msg):
        self.last_cmd_time = time.time()

    def check_emergency_stop(self):
        # 如果5秒没有速度命令，可能需要急停报警
        if time.time() - self.last_cmd_time > 5.0:
            # 可以在这里添加急停逻辑
            pass

    def trigger_alarm(self, message):
        self.alarm_active = True
        self.get_logger().warn(f'🚨 报警: {message}')

        # 发布报警状态
        status = Bool()
        status.data = True
        self.alarm_status_pub.publish(status)

        # 在新线程中执行报警
        alarm_thread = threading.Thread(target=self.execute_alarm, args=(message,))
        alarm_thread.start()

    def execute_alarm(self, message):
        try:
            # 尝试使用Python报警脚本
            result = subprocess.run(['python3', '/root/alarm_atlas.py', '3'],
                                  capture_output=True, text=True, timeout=5)
            if result.returncode != 0:
                self.get_logger().error(f"报警脚本执行失败: {result.stderr}")
        except Exception as e:
            self.get_logger().error(f"报警执行异常: {str(e)}")

        # 重置报警状态
        time.sleep(3)
        self.alarm_active = False
        status = Bool()
        status.data = False
        self.alarm_status_pub.publish(status)

def main():
    rclpy.init()
    node = EnhancedAlarmNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
