import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包的共享目录
    racecar_dir = get_package_share_directory('racecar')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    rf2o_share_dir = FindPackageShare('rf2o_laser_odometry').find('rf2o_laser_odometry')

    # 声明launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    map_yaml_path = LaunchConfiguration('map', default=os.path.join(racecar_dir, 'map', 'my_map.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(racecar_dir, 'config', 'racecar_nav2_params.yaml'))
    rviz_config_dir = os.path.join(nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # 声明初始位置参数
    init_x = LaunchConfiguration('init_x', default='0.0')
    init_y = LaunchConfiguration('init_y', default='0.0')
    init_a = LaunchConfiguration('init_a', default='0.0')

    # 启动文件和节点定义
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([nav2_bringup_dir, 'launch', 'bringup_launch.py'])
        ]),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path,
            'base_frame_id': base_link,
        }.items(),
    )
    
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([rf2o_share_dir, 'launch', 'rf2o_laser_odometry.launch.py'])
        ])
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )

    static_transform_publisher_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'map', 'odom']
    )

    static_transform_publisher_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', 'base_link', 'laser']
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': '/home/racecar/src/racecar/map/my_map.yaml'}]  # Ensure path is a string
    )

    
    encoder_imu_node = Node(
        package='encoder_imu',
        executable='imu_encoder_mix',
        name='imu_encoder_mix',
    )

    racecar_controller_node = Node(
        package='racecar',
        executable='car_controller_new',
        name='car_controller',
        output='screen',
        parameters=[{
            'Vcmd': 1.0,
            'baseSpeed': 1500.0,
            'baseAngle': 90.0,
            'Angle_gain_p': -3.0,
            'Angle_gain_d': -3.0,
            'Lfw': 1.5,
            'vp_max_base': 300,
            'vp_min': 380,
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to start RViz'
        ),
        DeclareLaunchArgument(
            'init_x',
            default_value='0.0',
            description='Initial X position for AMCL'
        ),
        DeclareLaunchArgument(
            'init_y',
            default_value='0.0',
            description='Initial Y position for AMCL'
        ),
        DeclareLaunchArgument(
            'init_a',
            default_value='0.0',
            description='Initial orientation for AMCL'
        ),
        nav2_bringup_launch,
        rf2o_launch,
        rviz_node,
        static_transform_publisher_map_to_odom,
        static_transform_publisher_base_to_laser,
        map_server_node,
        encoder_imu_node,
        racecar_controller_node
    ])