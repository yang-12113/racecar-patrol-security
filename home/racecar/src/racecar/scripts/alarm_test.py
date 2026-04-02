#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class AlarmTest(Node):
    def __init__(self):
        super().__init__('alarm_test')
        
        # 发布器
        self.alarm_trigger_pub = self.create_publisher(
            Bool, '/alarm_trigger', 10)
        
        self.get_logger().info('Alarm test node started')

    def test_alarm(self):
        Test alarm system
        self.get_logger().info('Starting alarm system test...')
        
        # Wait for system startup
        time.sleep(2)
        
        # Test manual alarm trigger
        self.get_logger().info('Testing manual alarm trigger...')
        trigger_msg = Bool()
        trigger_msg.data = True
        self.alarm_trigger_pub.publish(trigger_msg)
        
        # Wait 5 seconds
        time.sleep(5)
        
        self.get_logger().info('Alarm test completed')

def main(args=None):
    rclpy.init(args=args)
    test = AlarmTest()
    
    try:
        test.test_alarm()
    finally:
        test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
