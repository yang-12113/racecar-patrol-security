from launch import LaunchDescription
from launch_ros.actions import Node
import launch.substitutions

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    
    parameters = {
        'use_sim_time': use_sim_time,
        'odom_frame': 'odom',
        'base_frame': 'base_footprint'
    }
    
    return LaunchDescription([
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            output='screen',
            parameters=[parameters],
        ),
    ])