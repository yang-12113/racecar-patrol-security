import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class AlarmNode(Node):
    def __init__(self):
        super().__init__('alarm_node')
        self.create_subscription(
            Bool,
            '/alarm_trigger',
            self.alarm_callback,
            10
        )
        self.get_logger().info('报警节点已启动')

    def alarm_callback(self, msg):
        if msg.data:
            self.get_logger().warn('⚠️ 小车报警：检测到危险！')


def main():
    rclpy.init()
    node = AlarmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
