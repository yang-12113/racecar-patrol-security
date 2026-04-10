#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node


def generate_launch_description():
    driver_dir = os.path.join(
        get_package_share_directory('lslidar_driver'),
        'params',
        'lsx10.yaml',
    )
    enable_rviz = LaunchConfiguration('enable_rviz')

    driver_node = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[driver_dir],
    )

    rviz_dir = os.path.join(
        get_package_share_directory('lslidar_driver'),
        'rviz',
        'nav2_default_view.rviz',
    )
    rviz_node = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_dir],
        output='screen',
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='false',
            description='Launch RViz together with the lidar driver.',
        ),
        driver_node,
        rviz_node,
    ])
