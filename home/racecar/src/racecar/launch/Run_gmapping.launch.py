import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 找到包的共享目录
    rf2o_share_dir = FindPackageShare('rf2o_laser_odometry').find('rf2o_laser_odometry')
    racecar_share_dir = FindPackageShare('racecar').find('racecar')

    # 包含 rf2o_laser_odometry 启动文件
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution([rf2o_share_dir, 'launch', 'rf2o_laser_odometry.launch.py'])])
    )

    # 传感器数据节点
    encoder_imu_node = Node(
        package='encoder_imu',
        executable='imu_encoder_mix',
        name='imu_encoder_mix',
        output='screen',
    )

    return LaunchDescription([
        rf2o_launch,
        encoder_imu_node,
    ])

if __name__ == '__main__':
    import sys
    print("Starting launch script", file=sys.stderr)
    ld = generate_launch_description()
    print("Launch description generated:", ld, file=sys.stderr)
    
    # 使用 LaunchService 启动描述
    ls = launch.LaunchService()
    ls.include_launch_description(ld)
    print("Running LaunchService", file=sys.stderr)
    ls.run()
    print("LaunchService finished", file=sys.stderr)