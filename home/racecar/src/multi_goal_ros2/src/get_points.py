import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import sys, select, termios, tty

count = 0
point = []

class PoseStampedSubscriber(Node):

    def __init__(self):
        super().__init__('get_point')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.settings = termios.tcgetattr(sys.stdin)

    def listener_callback(self, data):
        global count
        gx = data.pose.position.x
        gy = data.pose.position.y
        gz = data.pose.orientation.z
        gw = data.pose.orientation.w
        point.append([gx, gy, gz, gw])
        if count == 3:
            self.data_write_csv('test.csv', point)
        count += 1

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def data_write_csv(self, file_name, datas):
        with open(file_name, 'w+', newline='', encoding='utf-8') as file_csv:
            writer = csv.writer(file_csv, delimiter=',', quoting=csv.QUOTE_MINIMAL)
            for data in datas:
                writer.writerow(data)
        self.get_logger().info("write succ!!")


def main(args=None):
    rclpy.init(args=args)
    node = PoseStampedSubscriber()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            key = node.get_key()
            if key.lower() == 'f':
                node.data_write_csv('test.csv', point)
            if key == '\x03':
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()