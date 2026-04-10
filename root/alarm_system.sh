#!/bin/bash

# 小车报警系统启动脚本

ALARM_DURATION=3

echo "=== 小车报警系统 ==="
echo "1. 测试报警"
echo "2. 启动ROS2报警节点"
echo "3. 安装GPIO依赖"
echo "4. 退出"

read -p "请选择操作 [1-4]: " choice

case $choice in
    1)
        echo "测试报警系统..."
        python3 /root/alarm_atlas.py $ALARM_DURATION
        ;;
    2)
        echo "启动ROS2报警节点..."
        source /opt/ros/humble/setup.bash
        cd /root
        python3 /root/alarm_node.py &
        echo "报警节点已启动，PID: $!"
        echo "发送报警: ros2 topic pub /alarm_trigger std_msgs/Bool 'data: true'"
        ;;
    3)
        echo "安装GPIO依赖..."
        apt update
        apt install -y python3-gpiod gpiod
        echo "依赖安装完成"
        ;;
    4)
        exit 0
        ;;
    *)
        echo "无效选择"
        ;;
esac
