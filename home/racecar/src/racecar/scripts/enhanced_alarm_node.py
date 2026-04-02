#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class EnhancedAlarmNode(Node):
    def __init__(self):
        super().__init__('enhanced_alarm_node')
        
        self.alarm_active = False
        self.alarm_reason = 
        
        self.alarm_trigger_sub = self.create_subscription(
            Bool, '/alarm_trigger', self.alarm_trigger_callback, 10)
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10)
        
        self.alarm_status_pub = self.create_publisher(
            Bool, '/alarm_status', 10)
        self.alarm_reason_pub = self.create_publisher(
            String, '/alarm_reason', 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.safety_distance = 0.5
        
        self.get_logger().info('Enhanced alarm node started')

    def alarm_trigger_callback(self, msg):
        if msg.data and not self.alarm_active:
            self.trigger_alarm(Manual alarm triggered)

    def laser_callback(self, msg):
        if not msg.ranges:
            return
            
        front_ranges = msg.ranges[len(msg.ranges)//3 : 2*len(msg.ranges)//3]
        min_distance = min([r for r in front_ranges if r > 0]) if front_ranges else float('inf')
        
        if min_distance < self.safety_distance and not self.alarm_active:
            alarm_text = Obstacle alarm: %.2fm % min_distance
            self.trigger_alarm(alarm_text)

    def trigger_alarm(self, reason):
        self.alarm_active = True
        self.alarm_reason = reason
        
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_cmd)
        
        self.get_logger().warn('Alarm triggered: %s' % reason)

    def timer_callback(self):
        status_msg = Bool()
        status_msg.data = self.alarm_active
        self.alarm_status_pub.publish(status_msg)
        
        if self.alarm_active:
            reason_msg = String()
            reason_msg.data = self.alarm_reason
            self.alarm_reason_pub.publish(reason_msg)

def main(args=None):
    rclpy.init(args=args)
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
