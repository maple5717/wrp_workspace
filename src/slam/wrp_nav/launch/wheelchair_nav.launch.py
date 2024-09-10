from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare the arguments to the launch file

    robot_nav_dir = FindPackageShare('wrp_nav').find('wrp_nav')

    # Path to the YAML config file in the robot_nav package
    params_file_path = os.path.join(robot_nav_dir, 'config', 'nav2_params.yaml')

    base_launch_file_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch_file_path),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': params_file_path
                }.items()
            ),
        ])
