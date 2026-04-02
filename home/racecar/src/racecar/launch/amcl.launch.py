from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('init_x', default_value='0', description='Initial x position'),
        DeclareLaunchArgument('init_y', default_value='0', description='Initial y position'),
        DeclareLaunchArgument('init_a', default_value='0', description='Initial yaw orientation'),
        
        # AMCL node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'transform_tolerance': 0.2,
                'gui_publish_rate': 10.0,
                'laser_max_beams': 30,
                'min_particles': 100,
                'max_particles': 5000,
                'kld_err': 0.01,
                'kld_z': 0.99,
                'odom_alpha1': 0.2,
                'odom_alpha2': 0.2,
                'odom_alpha3': 0.8,
                'odom_alpha4': 0.2,
                'laser_z_hit': 0.5,
                'laser_z_short': 0.05,
                'laser_z_max': 0.05,
                'laser_z_rand': 0.5,
                'laser_sigma_hit': 0.2,
                'laser_lambda_short': 0.1,
                'laser_model_type': 'likelihood_field',
                'laser_likelihood_max_dist': 2.0,
                'update_min_d': 0.15,
                'update_min_a': 0.15,
                'resample_interval': 2,
                'transform_tolerance': 0.1,
                'recovery_alpha_slow': 0.0,
                'recovery_alpha_fast': 0.1,
                'use_map_topic': True,
                'first_map_only': True,
                'tf_broadcast': True,
                'odom_frame_id': 'odom',
                'global_frame_id': 'map',
                'base_frame_id': 'base_footprint',
                'odom_model_type': 'diff',
                'initial_pose_x': LaunchConfiguration('init_x'),
                'initial_pose_y': LaunchConfiguration('init_y'),
                'initial_pose_a': LaunchConfiguration('init_a'),
                'initial_cov_xx': 0.25,
                'initial_cov_yy': 0.25,
                'initial_cov_aa': 0.2,
            }],
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()