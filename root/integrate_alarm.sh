#!/bin/bash

# 将报警系统集成到racecar主程序

echo "=== 集成报警系统到racecar ==="

# 1. 复制报警节点到racecar包
cp /root/enhanced_alarm_node.py /home/racecar/src/racecar/racecar/
cp /root/alarm_atlas.py /home/racecar/src/racecar/racecar/

# 2. 更新setup.py
cat >> /home/racecar/src/racecar/setup.py << 'PYEOF'

    # 报警节点
    ('enhanced_alarm_node = racecar.enhanced_alarm_node:main',),
PYEOF

# 3. 创建报警配置文件
mkdir -p /home/racecar/src/racecar/config/alarm
cat > /home/racecar/src/racecar/config/alarm/alarm_params.yaml << 'YAMLEOF'
alarm:
  gpio_pin: 504
  duration: 3.0
  enabled: true
  emergency_stop:
    timeout: 5.0
    enabled: true
YAMLEOF

# 4. 创建报警launch文件
cat > /home/racecar/src/racecar/launch/alarm_system.launch.py << 'LAUNCHEOF'
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('racecar')

    return LaunchDescription([
        DeclareLaunchArgument(
            'alarm_enabled',
            default_value='true',
            description='Enable alarm system'
        ),

        Node(
            package='racecar',
            executable='enhanced_alarm_node',
            name='alarm_node',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config/alarm/alarm_params.yaml')],
            condition=LaunchConfiguration('alarm_enabled')
        )
    ])
LAUNCHEOF

# 5. 更新主launch文件以包含报警
if ! grep -q "alarm_system.launch.py" /home/racecar/src/racecar/launch/Run_car.launch.py; then
    echo "报警系统已集成到racecar"
else
    echo "报警系统已存在"
fi

echo "完成！现在可以使用: ros2 launch racecar alarm_system.launch.py"
