import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import os

# 设置无显示环境
os.environ['DISPLAY'] = ''

class RacecarLineFollow(Node):
    def __init__(self):
        super().__init__('racecar_line_follow')
        self.publisher_cmd = self.create_publisher(Twist, '/car_cmd_vel', 10)

        # 控制参数
        self.msg = Twist()
        self.msg.linear.x = float(1525)  # 油门中值
        self.msg.angular.z = float(90)   # 转向中值

        # PID控制器参数
        self.kp = 0.0011
        self.ki = 0.0000
        self.kd = 0.0001
        self.error_sum = 0
        self.last_error = 0

        # HSV颜色参数
        self.lower_yellow = np.array([21, 66, 115])
        self.upper_yellow = np.array([179, 255, 255])

        # 图像参数
        self.image_width = 640
        self.image_height = 480

        self.get_logger().info('巡线节点初始化完成（无显示模式）')

    def create_mask(self, picture, mask_point):
        mask = np.zeros_like(picture)
        cv2.fillPoly(mask, mask_point, 255)
        mask_img = cv2.bitwise_and(picture, mask)
        return mask_img

    def calculate_line_angle(self, contour):
        [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
        if abs(vx) < 0.001:
            return 90.0, 0
        k = vy / vx
        y0 = self.image_height / 2
        x0 = k * y0 + x - k * y
        delta_x = x0 - self.image_width / 2
        delta_y = self.image_height - y0
        angle = -np.arctan2(delta_x, delta_y) * 180 / np.pi
        angle = max(40, min(140, angle))
        return angle, k

    def pid_control(self, target_angle, current_angle, dt):
        error = target_angle - current_angle
        p_term = self.kp * error
        self.error_sum += error * dt
        i_term = self.ki * self.error_sum
        d_term = self.kd * (error - self.last_error) / dt if dt > 0 else 0
        output = p_term + i_term + d_term
        self.last_error = error
        return output

    def process_frame(self, frame):
        try:
            mask_point = np.array([[
                (0, self.image_height * 2//3),
                (self.image_width, self.image_height * 2//3),
                (self.image_width, self.image_height),
                (0, self.image_height)
            ]])

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)
            masked_yellow = self.create_mask(yellow_mask, mask_point)

            kernel = np.ones((3, 3), np.uint8)
            masked_yellow = cv2.morphologyEx(masked_yellow, cv2.MORPH_CLOSE, kernel)
            masked_yellow = cv2.morphologyEx(masked_yellow, cv2.MORPH_OPEN, kernel)

            edges = cv2.Canny(masked_yellow, 50, 150)
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 100:
                    angle, slope = self.calculate_line_angle(largest_contour)
                    dt = 0.033
                    steering_adjustment = self.pid_control(90, angle, dt)
                    final_angle = 90 + steering_adjustment
                    final_angle = max(40, min(140, final_angle))
                    
                    self.msg.angular.z = float(final_angle)
                    self.publisher_cmd.publish(self.msg)
                    
                    self.get_logger().info(f'检测角度: {angle:.1f}°, 最终角度: {final_angle:.1f}°, 斜率: {slope:.3f}')
                else:
                    self.get_logger().warn('未找到足够大的线条')
            else:
                self.get_logger().warn('未检测到线条')

        except Exception as e:
            self.get_logger().error(f'处理帧时出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    try:
        node = RacecarLineFollow()
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            node.get_logger().error('无法打开摄像头')
            return

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        node.get_logger().info('开始巡线（无显示模式），运行10秒后自动退出')
        
        import time
        start_time = time.time()
        
        while rclpy.ok() and (time.time() - start_time) < 10:
            ret, frame = cap.read()
            if not ret:
                node.get_logger().error('无法读取摄像头')
                break
            
            node.process_frame(frame)
            rclpy.spin_once(node, timeout_sec=0.001)

    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    except Exception as e:
        node.get_logger().error(f'运行时错误: {str(e)}')
    finally:
        if 'cap' in locals():
            cap.release()
        
        stop_msg = Twist()
        stop_msg.linear.x = 1500.0
        stop_msg.angular.z = 90.0
        node.publisher_cmd.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
