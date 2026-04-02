from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 声明参数
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='是否启动RViz'
    )
    
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value='/home/racecar/src/racecar/map/test_map.yaml',
        description='地图文件路径'
    )
    
    declare_sim_cmd = DeclareLaunchArgument(
        'sim',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    declare_init_x_cmd = DeclareLaunchArgument(
        'init_x',
        default_value='0.0',
        description='AMCL初始X位置'
    )
    
    declare_init_y_cmd = DeclareLaunchArgument(
        'init_y',
        default_value='0.0',
        description='AMCL初始Y位置'
    )
    
    declare_init_a_cmd = DeclareLaunchArgument(
        'init_a',
        default_value='0.0',
        description='AMCL初始角度'
    )

    # 获取包路径
    racecar_dir = get_package_share_directory('racecar')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # 配置文件路径
    map_path = LaunchConfiguration('map')
    nav2_params_path = os.path.join(racecar_dir, 'config', 'racecar_nav2_params.yaml')
    rviz_config_path = os.path.join(racecar_dir, 'rviz', 'navigation.rviz')
    
    # 静态TF发布器
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )
    
    static_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_publisher',
        arguments=['0.07', '0.0', '0.0', '0', '0', '0', 'base_link', 'laser_link'],
        output='screen'
    )
    
    static_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_publisher',
        arguments=['0.1653', '0.0', '0.0', '0', '0', '0', 'base_link', 'IMU_link'],
        output='screen'
    )
    
    # Nav2启动
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_dir, 'launch', 'bringup_launch.py'
        ]),
        launch_arguments={
            'map': map_path,
            'use_sim_time': LaunchConfiguration('sim'),
            'params_file': nav2_params_path,
            'autostart': 'true',
            'use_composition': 'true'
        }.items()
    )
    
    # RViz（可选）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('sim')}],
        output='screen'
    )
    
    # AMCL初始位姿发布器（延迟启动）
    initial_pose_publisher = TimerAction(
        period=3.0,  # 3秒后发布初始位姿
        actions=[
            Node(
                package='ros2topic',
                executable='ros2topic',
                name='initial_pose_publisher',
                arguments=[
                    'pub', '--once', '/initialpose',
                    'geometry_msgs/msg/PoseWithCovarianceStamped',
                    '{header: {stamp: {sec: 0}, frame_id: map}, pose: {pose: {position: {x: ' + LaunchConfiguration('init_x') + ', y: ' + LaunchConfiguration('init_y') + ', z: 0.0}, orientation: {x: 0.0, y: 0.0, z: ' + LaunchConfiguration('init_a') + ', w: 1.0}}, covariance: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}'
                ],
                output='screen'
            )
        ]
    )
    
    # 导航测试脚本节点
    navigation_test_node = TimerAction(
        period=5.0,  # 5秒后开始导航测试
        actions=[
            Node(
                package='racecar',
                executable='navigation_test',
                name='navigation_test',
                output='screen',
                parameters=[{
                    'test_points': [
                        [2.0, 0.0, 0.0],    # 第一个目标点
                        [2.0, 2.0, 1.57],   # 第二个目标点
                        [0.0, 2.0, 3.14],   # 第三个目标点
                        [0.0, 0.0, 0.0]     # 回到起点
                    ],
                    'wait_time': 3.0  # 到达每个点后等待时间
                }]
            )
        ]
    )
    
    return LaunchDescription([
        declare_use_rviz_cmd,
        declare_map_cmd,
        declare_sim_cmd,
        declare_init_x_cmd,
        declare_init_y_cmd,
        declare_init_a_cmd,
        static_map_to_odom,
        static_base_to_laser,
        static_base_to_imu,
        nav2_bringup_launch,
        rviz_node,
        initial_pose_publisher,
        navigation_test_node
    ])
