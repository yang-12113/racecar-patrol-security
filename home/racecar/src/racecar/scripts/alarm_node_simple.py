#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist

class SimpleAlarmNode(Node):
    def __init__(self):
        super().__init__('simple_alarm_node')
        
        self.alarm_active = False
        
        self.alarm_trigger_sub = self.create_subscription(
            Bool, '/alarm_trigger', self.alarm_callback, 10)
        
        self.alarm_status_pub = self.create_publisher(
            Bool, '/alarm_status', 10)
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Simple alarm node started')

    def alarm_callback(self, msg):
        if msg.data and not self.alarm_active:
            self.alarm_active = True
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_cmd)
            self.get_logger().warn('Alarm triggered - stopping vehicle')

    def timer_callback(self):
        status_msg = Bool()
        status_msg.data = self.alarm_active
        self.alarm_status_pub.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAlarmNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
