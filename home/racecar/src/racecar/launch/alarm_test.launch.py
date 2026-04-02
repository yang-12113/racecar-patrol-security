from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # 启动增强报警节点
        Node(
            package='racecar',
            executable='enhanced_alarm_node',
            name='enhanced_alarm_node',
            output='screen',
            emulate_tty=True,
        ),

        # 测试报警触发
        ExecuteProcess(
            cmd=['sleep', '3'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/alarm_trigger', 'std_msgs/Bool', '{data: true}'],
            output='screen'
        ),
    ])
