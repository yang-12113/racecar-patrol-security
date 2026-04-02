import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Arguments
    use_rviz_arg = DeclareLaunchArgument('use_rviz', default_value='false')
    init_x_arg = DeclareLaunchArgument('init_x', default_value='0.0')
    init_y_arg = DeclareLaunchArgument('init_y', default_value='0.0')
    init_a_arg = DeclareLaunchArgument('init_a', default_value='0.0')

    # Nodes and processes
    map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0.0', 'map', 'odom'],
        parameters=[{'frequency': 1000}]
    )
    base_to_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0.0', 'base_link', 'laser'],
        parameters=[{'frequency': 1000}]
    )
    map_server_node = Node(
        package='map_server',
        executable='map_server',
        name='map_server',
        arguments=[os.path.join(get_package_share_directory('racecar'), 'map', 'my_map.yaml')]
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_se',
        parameters=[os.path.join(get_package_share_directory('racecar'), 'param', 'ekf_params1.yaml')],
        remappings=[('/odom', '/odometry/filtered')]
    )
    move_base_node = Node(
        package='nav2_bringup',
        executable='bringup_launch.py',
        name='move_base',
        parameters=[
            {'base_global_planner': 'navfn/NavfnROS'},
            {'base_local_planner': 'dwa_local_planner/DWAPlannerROS'},
            os.path.join(get_package_share_directory('racecar'), 'param', 'dwa_local_planner_params.yaml'),
            os.path.join(get_package_share_directory('racecar'), 'param', 'local_costmap_params.yaml'),
            os.path.join(get_package_share_directory('racecar'), 'param', 'global_costmap_params.yaml'),
            os.path.join(get_package_share_directory('racecar'), 'param', 'base_global_planner_params.yaml'),
            os.path.join(get_package_share_directory('racecar'), 'param', 'move_base_params.yaml')
        ],
        remappings=[('/odom', '/odometry/filtered')]
    )
    car_controller_node = Node(
        package='racecar',
        executable='car_controller_new',
        name='car_controller',
        parameters=[
            {'Vcmd': 1.0},
            {'baseSpeed': 30},
            {'baseAngle': 0.0},
            {'Angle_gain_p': -3.0},
            {'Angle_gain_d': -3.0},
            {'Lfw': 1.5},
            {'vp_max_base': 50},
            {'vp_min': 30}
        ]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('racecar'), 'rviz', 'amcl.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    # Launch Description
    return LaunchDescription([
        use_rviz_arg,
        init_x_arg,
        init_y_arg,
        init_a_arg,
        map_to_odom_node,
        base_to_laser_node,
        map_server_node,
        ekf_node,
        move_base_node,
        car_controller_node,
        rviz_node
    ])