from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    safety_distance_arg = DeclareLaunchArgument(
        'safety_distance',
        default_value='0.5',
        description='安全距离(米)'
    )
    
    safety_distance = LaunchConfiguration('safety_distance')

    return LaunchDescription([
        safety_distance_arg,
        
        # 启动增强报警节点
        Node(
            package='racecar',
            executable='enhanced_alarm_node.py',
            name='enhanced_alarm_node',
            output='screen',
            emulate_tty=True,
            parameters=[{'safety_distance': safety_distance}]
        ),
    ])
