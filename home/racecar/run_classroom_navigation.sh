#!/bin/bash

# 教室导航启动脚本
echo 启动教室导航功能...

# 设置ROS2环境
source /opt/ros/humble/setup.bash
cd /home/racecar
source install/setup.bash

# 清理之前的进程
echo 清理之前的进程...
pkill -f nav2
pkill -f racecar_driver
pkill -f amcl
sleep 2

# 启动基础系统（传感器驱动）
echo 启动基础传感器...
ros2 launch racecar Run_car.launch.py &
sleep 3

# 启动导航系统
echo 启动导航系统...
ros2 launch racecar classroom_nav.launch.py     map:=/home/racecar/src/racecar/map/test_map.yaml     use_rviz:=false &
sleep 5

# 单独运行导航测试
echo 5秒后开始导航测试...
sleep 5

python3 /home/racecar/src/racecar/scripts/navigation_test.py &
echo 导航测试已启动...

# 保持运行
echo 按Ctrl+C退出...
trap 'echo "正在停止..."; pkill -f nav2; pkill -f racecar; exit' INT
wait
