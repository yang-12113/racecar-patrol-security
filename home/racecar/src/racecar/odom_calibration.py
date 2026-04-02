# 创建校准脚本
  cat > odom_calibration.py << 'EOF'
  #!/usr/bin/env python3
  import rclpy
  from rclpy.node import Node
  from nav_msgs.msg import Odometry
  import numpy as np
  import json
  import time

  class OdomCalibration(Node):
      def __init__(self):
          super().__init__('odom_calibration')

          # 订阅者
          self.odom_sub = self.create_subscription(
              Odometry, '/odom', self.odom_callback, 10
          )

          # 数据存储
          self.positions = []
          self.start_pos = None
          self.calibration_done = False
          self.calibration_matrix = None

          self.get_logger().info('?? 里程计校准工具启动')

      def odom_callback(self, msg):
          position = msg.pose.pose.position
          current_pos = (position.x, position.y)

          if self.start_pos is None:
              # 计算行驶距离
              distance = np.sqrt(
                  (current_pos[0] - self.start_pos[0])**2 +
                  (current_pos[1] - self.start_pos[1])**2
              )
              self.positions.append((current_pos[0], current_pos[1], time.time()))

          self.last_pos = current_pos

      def calculate_calibration(self):
          if len(self.positions) < 3:
              self.get_logger().error('? 需要至少3个位置点进行校准')
              return None

          # 构建A矩阵 [Ax = b]
          positions = np.array(self.positions)
          times = np.array([p[2] for p in positions])

          # 构建b向量 [bx = by]
          b = np.array([
              positions[1][0] - positions[0][0],
              positions[1][1] - positions[0][1]
          ])

          # 计算校准矩阵
          # X = (A^T * A^-1) * b
          calibration_matrix = np.linalg.inv(A.T @ A.T) @ b

          self.calibration_matrix = calibration_matrix
          self.calibration_done = True

          self.get_logger().info('? 里程计校准完成！')
          self.get_logger().info(f'校准矩阵:\n{calibration_matrix}')

          return calibration_matrix

      def save_calibration(self):
          calibration_data = {
              'calibration_matrix': self.calibration_matrix.tolist(),
              'positions': self.positions,
              'timestamp': time.time(),
              'start_pos': self.start_pos,
              'calibration_matrix': self.calibration_matrix.tolist()
          }

          # 保存校准数据
          timestamp = time.strftime("%Y%m%d_%H%M%S")
          filename = f'/home/racecar/calibration/odom_calibration_{timestamp}.json'

          with open(filename, 'w') as f:
              json.dump(calibration_data, f, indent=2)

          self.get_logger().info(f'?? 校准数据已保存到: {filename}')
          self.get_logger().info('?? 校准数据路径:
  /home/racecar/calibration/odom_calibration_{timestamp}.json')

  def main(args=None):
      rclpy.init()
      calibrator = OdomCalibration()

      print("=== ?? 里程计校准工具启动 ===")
      print("?? 操作说明：")
      print("1. 使用键盘遥控将小车移动到起始位置")
      print("2. 逐步移动到第二位置")
      print("3. 继续移动到第三位置")
      print("4. 返回起始位置")
      print("5. 自动计算校准矩阵")
      print("6. 校准数据自动保存")
      print("7. 按 Ctrl+C 停止校准")

      try:
          rclpy.spin(calibrator, timeout_sec=60)
      except KeyboardInterrupt:
          if calibrator.calibration_done:
              calibrator.save_calibration()
              print("?? ? 校准数据已保存")
          else:
              print("?? 校准未完成，数据未保存")
      finally:
          calibrator.destroy_node()
          rclpy.shutdown()

  if __name__ == '__main__':
      main()
  EOF