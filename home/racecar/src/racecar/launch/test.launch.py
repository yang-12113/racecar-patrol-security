from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='racecar',
            executable='car_controller_new',
            name='car_controller',
            output='screen',
            parameters=[
                {'Vcmd': 1.0},
                {'baseSpeed': 27.0},
                {'baseAngle':90.0},
                {'Angle_gain_p': -6.0},
                {'Angle_gain_d': -0.1},
                {'Lfw': 1.0},
                {'vp_max_base': 27.0},
                {'vp_min': 27.0},
                {'goalRadius': 1.0},
                {'controller_freq': 10},
            ]
        )
    ])