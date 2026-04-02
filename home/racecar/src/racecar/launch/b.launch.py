import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

MAP_NAME = 'my_map'

def generate_launch_description():

    init_x = LaunchConfiguration('init_x', default='0.0')
    init_y = LaunchConfiguration('init_y', default='0.0')
    init_a = LaunchConfiguration('init_a', default='0.0')


    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to start RViz'
    )
    declare_init_x_cmd = DeclareLaunchArgument(
        'init_x',
        default_value='0.0',
        description='Initial X position for AMCL'
    )
    declare_init_y_cmd = DeclareLaunchArgument(
        'init_y',
        default_value='0.0',
        description='Initial Y position for AMCL'
    )
    declare_init_a_cmd = DeclareLaunchArgument(
        'init_a',
        default_value='0.0',
        description='Initial orientation for AMCL'
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


    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_bringup_launch_file_path = PathJoinSubstitution([nav2_bringup_dir, 'launch', 'bringup_launch.py'])

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('racecar'), 'rviz', 'navigation.rviz']
    )
    default_map_path = PathJoinSubstitution(
        [FindPackageShare('racecar'), 'map', f'{MAP_NAME}.yaml']
    )
    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('racecar'), 'config/nav', 'navigation.yaml']
    )
    nav2_sim_config_path = PathJoinSubstitution(
        [FindPackageShare('racecar'), 'config/nav', 'navigation_sim.yaml']
    )


    declare_sim_cmd = DeclareLaunchArgument(
        name='sim', 
        default_value='false',
        description='Enable use_sim_time to true'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        name='rviz', 
        default_value='false',
        description='Run rviz'
    )
    declare_map_cmd = DeclareLaunchArgument(
        name='map', 
        default_value=default_map_path,
        description='Navigation map path'
    )


    include_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_file_path),
        condition=UnlessCondition(LaunchConfiguration("sim")),
        launch_arguments={
            'map': LaunchConfiguration("map"),
            'use_sim_time': LaunchConfiguration("sim"),
            'params_file': nav2_config_path
        }.items()
    )

    include_nav2_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_bringup_launch_file_path),
        condition=IfCondition(LaunchConfiguration("sim")),
        launch_arguments={
            'map': LaunchConfiguration("map"),
            'use_sim_time': LaunchConfiguration("sim"),
            'params_file': nav2_sim_config_path
        }.items()
    )


    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration("rviz")),
        parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
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
        declare_use_rviz_cmd,
        declare_init_x_cmd,
        declare_init_y_cmd,
        declare_init_a_cmd,
        static_transform_publisher_map_to_odom,
        static_transform_publisher_base_to_laser,
        declare_sim_cmd,
        declare_rviz_cmd,
        declare_map_cmd,
        include_nav2_launch,
        include_nav2_sim_launch,
        rviz_node,
        racecar_controller_node
    ])