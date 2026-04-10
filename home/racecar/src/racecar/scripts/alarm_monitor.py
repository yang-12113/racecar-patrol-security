#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import time

class AlarmMonitor(Node):
    def __init__(self):
        super().__init__('alarm_monitor')
        
        # 订阅报警状态
        self.alarm_status_sub = self.create_subscription(
            Bool, '/alarm_status', self.status_callback, 10)
        self.alarm_reason_sub = self.create_subscription(
            String, '/alarm_reason', self.reason_callback, 10)
        
        # 当前状态
        self.current_status = False
        self.current_reason = ""
        
        self.get_logger().info('报警监控器已启动')

    def status_callback(self, msg):
        报警状态回调
        if msg.data != self.current_status:
            self.current_status = msg.data
            if msg.data:
                self.get_logger().error('🚨 报警状态激活!')
            else:
                self.get_logger().info('✅ 报警状态解除')

    def reason_callback(self, msg):
        报警原因回调
        self.current_reason = msg.data
        self.get_logger().error(f'报警原因: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    monitor = AlarmMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
