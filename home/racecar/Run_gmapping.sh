#!/bin/bash

# Function to handle termination
terminate() {
  echo "终止所有后台进程..."
  kill $ros2_launch_pid $ekf_node_pid $imu_encoder_mix_pid $slam_gmapping_pid
  wait $ros2_launch_pid $ekf_node_pid $imu_encoder_mix_pid $slam_gmapping_pid
  echo "所有进程已终止。"
}

# Trap Ctrl+C (SIGINT)
trap terminate SIGINT

# 检查ROS 2环境
if [ -z "$ROS_DISTRO" ]; then
  echo "请先设置好ROS 2环境变量，例如'source /opt/ros/humble/setup.bash'"
  exit 1
fi

# 设置参数文件路径
PARAMS_FILE="/home/racecar/src/racecar/config/ekf_params1.yaml"

# 检查参数文件是否存在
if [ ! -f "$PARAMS_FILE" ]; then
  echo "参数文件 $PARAMS_FILE 不存在，请检查路径是否正确。"
  exit 1
fi

# 打印调试信息
echo "参数文件路径: $PARAMS_FILE"

# 设置launch文件路径
LAUNCH_FILE="/home/racecar/src/racecar/launch/sensor.launch.py"

# 检查launch文件是否存在
if [ ! -f "$LAUNCH_FILE" ]; then
  echo "launch文件 $LAUNCH_FILE 不存在，请检查路径是否正确。"
  exit 1
fi

# 打印调试信息
echo "launch文件路径: $LAUNCH_FILE"

# 启动 ROS 2 launch 文件
echo "启动 ROS 2 launch 文件: racecar sensor.launch.py"
ros2 launch racecar sensor.launch.py > sensor_launch.log 2>&1 &
ros2_launch_pid=$!

# 等待5秒
sleep 5

# 启动 EKF 节点
#echo "启动 EKF 节点"
#ros2 run robot_localization ekf_node --ros-args --params-file "$PARAMS_FILE" -r odom:=/odometry/filtered > ekf_node.log 2>&1 &
#ekf_node_pid=$!

# 启动 encoder_imu imu_encoder_mix 节点
echo "启动 encoder_imu imu_encoder_mix 节点"
ros2 run encoder_imu imu_encoder_mix > imu_encoder_mix.log 2>&1 &
imu_encoder_mix_pid=$!

# 等待5秒
sleep 5


ros2 run odom_tf odom_tf > odom_tf.log 2>&1 &
odom_tf_pid=$!
# 等待5秒
sleep 5

# 启动 slam_gmapping 节点
 echo "启动 slam_gmapping 节点"
 ros2 run slam_gmapping slam_gmapping > slam_gmapping.log 2>&1 &
 slam_gmapping_pid=$!

# echo "所有节点已启动。日志文件分别为 sensor_launch.log、ekf_node.log、imu_encoder_mix.log 和 slam_gmapping.log。"

#Wait for all background processes to complete
wait $ros2_launch_pid $ekf_node_pid $imu_encoder_mix_pid $slam_gmapping_pid