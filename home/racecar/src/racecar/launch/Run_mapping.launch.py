from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    racecar_share_dir = FindPackageShare("racecar").find("racecar")
    slam_gmapping_share_dir = FindPackageShare("slam_gmapping").find("slam_gmapping")

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([racecar_share_dir, "launch", "Run_car.launch.py"])]
        )
    )

    encoder_imu_node = Node(
        package="encoder_imu",
        executable="imu_encoder_mix",
        name="imu_encoder_mix",
        output="screen",
    )

    odom_tf_node = Node(
        package="odom_tf",
        executable="odom_tf",
        name="odom_tf",
        output="screen",
    )

    gmapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [PathJoinSubstitution([slam_gmapping_share_dir, "launch", "slam_gmapping.launch.py"])]
        )
    )

    return LaunchDescription(
        [
            sensor_launch,
            encoder_imu_node,
            odom_tf_node,
            gmapping_launch,
        ]
    )
