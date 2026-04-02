import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class RacecarLineFollow(Node):
    def __init__(self):
        super().__init__('racecar_line_follow')
        self.publisher_cmd = self.create_publisher(Twist, '/car_cmd_vel', 10)

        # 控制参数
        self.msg = Twist()
        self.msg.linear.x = float(1525)  # 油门中值
        self.msg.angular.z = float(90)   # 转向中值

        # PID控制器参数
        self.kp = 0.0011  # 比例系数
        self.ki = 0.0000  # 积分系数
        self.kd = 0.0001  # 微分系数
        self.error_sum = 0
        self.last_error = 0

        # HSV颜色参数（可配置）
        self.lower_yellow = np.array([21, 66, 115])
        self.upper_yellow = np.array([179, 255, 255])

        # 图像参数
        self.image_width = 640
        self.image_height = 480

        self.get_logger().info('巡线节点初始化完成')

    def create_mask(self, picture, mask_point):
        """创建掩模函数"""
        mask = np.zeros_like(picture)
        cv2.fillPoly(mask, mask_point, 255)
        mask_img = cv2.bitwise_and(picture, mask)
        return mask_img

    def calculate_line_angle(self, contour):
        """根据轮廓计算线条角度"""
        [vx, vy, x, y] = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)

        # 避免除零错误
        if abs(vx) < 0.001:
            return 90.0  # 垂直线

        # 计算斜率
        k = vy / vx

        # 计算角度（相对于画面中心）
        y0 = self.image_height / 2
        x0 = k * y0 + x - k * y

        # 计算偏离中心的角度
        delta_x = x0 - self.image_width / 2
        delta_y = self.image_height - y0

        angle = -np.arctan2(delta_x, delta_y) * 180 / np.pi

        # 限制角度范围
        angle = max(40, min(140, angle))

        return angle, k

    def pid_control(self, target_angle, current_angle, dt):
        """PID控制器计算"""
        error = target_angle - current_angle

        # PID计算
        p_term = self.kp * error
        self.error_sum += error * dt
        i_term = self.ki * self.error_sum
        d_term = self.kd * (error - self.last_error) / dt if dt > 0 else 0

        # 计算输出
        output = p_term + i_term + d_term

        self.last_error = error

        return output

    def process_frame(self, frame):
        """处理每一帧图像"""
        try:
            # 创建ROI掩模（感兴趣区域）
            mask_point = np.array([[
                (0, self.image_height * 2//3),
                (self.image_width, self.image_height * 2//3),
                (self.image_width, self.image_height),
                (0, self.image_height)
            ]])

            # 转换为HSV颜色空间
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 创建黄色掩模
            yellow_mask = cv2.inRange(hsv, self.lower_yellow, self.upper_yellow)

            # 应用ROI掩模
            masked_yellow = self.create_mask(yellow_mask, mask_point)

            # 形态学操作去噪
            kernel = np.ones((3, 3), np.uint8)
            masked_yellow = cv2.morphologyEx(masked_yellow, cv2.MORPH_CLOSE, kernel)
            masked_yellow = cv2.morphologyEx(masked_yellow, cv2.MORPH_OPEN, kernel)

            # 边缘检测
            edges = cv2.Canny(masked_yellow, 50, 150)

            # 查找轮廓
            contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 找到最大的轮廓
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)

                if cv2.contourArea(largest_contour) > 100:  # 面积阈值
                    angle, slope = self.calculate_line_angle(largest_contour)

                    # 使用PID控制计算转向
                    dt = 0.033  # 假设30fps
                    steering_adjustment = self.pid_control(90, angle, dt)

                    # 计算最终转向角度
                    final_angle = 90 + steering_adjustment
                    final_angle = max(40, min(140, final_angle))

                    # 更新控制消息
                    self.msg.angular.z = float(final_angle)
                    self.publisher_cmd.publish(self.msg)

                    self.get_logger().info(f'检测角度: {angle:.1f}°, 最终角度: {final_angle:.1f}°, 斜率: {slope:.3f}')

                    # 在图像上绘制信息（调试用）
                    cv2.putText(frame, f'Angle: {angle:.1f}°', (10, 30),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    cv2.putText(frame, f'Steering: {final_angle:.1f}°', (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    self.get_logger().warn('未找到足够大的线条')
            else:
                self.get_logger().warn('未检测到线条')

        except Exception as e:
            self.get_logger().error(f'处理帧时出错: {str(e)}')

def main(args=None):
    rclpy.init(args=args)

    try:
        # 创建节点
        node = RacecarLineFollow()

        # 打开摄像头
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            node.get_logger().error('无法打开摄像头')
            return

        # 设置摄像头参数
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)

        node.get_logger().info('开始巡线，按q退出')

        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                node.get_logger().error('无法读取摄像头')
                break

            # 处理图像
            node.process_frame(frame)

            # 显示图像（调试用，可选）
            cv2.imshow('Line Follow', frame)

            # 处理ROS回调
            rclpy.spin_once(node, timeout_sec=0.001)

            # 检查退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        node.get_logger().info('用户中断')
    except Exception as e:
        node.get_logger().error(f'运行时错误: {str(e)}')
    finally:
        # 清理资源
        if 'cap' in locals():
            cap.release()
        cv2.destroyAllWindows()

        # 停止小车
        stop_msg = Twist()
        stop_msg.linear.x = 1500.0
        stop_msg.angular.z = 90.0
        node.publisher_cmd.publish(stop_msg)

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
