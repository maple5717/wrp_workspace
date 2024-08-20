from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    wheelchair_slam_share_dir = FindPackageShare('wheelchair_slam').find('wheelchair_slam')
    config_file = str(wheelchair_slam_share_dir) + '/config/imu_odom.yaml'
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[config_file],
        
        ),
    ])
