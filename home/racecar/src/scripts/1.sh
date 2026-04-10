#!/bin/bash


# 设置参数文件路径
PARAMS_FILE="/home/racecar/src/racecar/config/ekf_params1.yaml"

# 检查参数文件是否存在
if [ ! -f "$PARAMS_FILE" ]; then
  echo "参数文件 $PARAMS_FILE 不存在，请检查路径是否正确。"
  exit 1
fi

# 启动 EKF 节点
ros2 run robot_localization ekf_node --ros-args --params-file "$PARAMS_FILE"