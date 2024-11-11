from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetRemap
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Declare the arguments to the launch file
    use_pid = DeclareLaunchArgument('use_pid', default_value='True', description='whether to inclued another PID loop outside the wheelchair controller')

    robot_nav_dir = FindPackageShare('wrp_nav').find('wrp_nav')

    # Path to the YAML config file in the robot_nav package
    nav2_param_file_name = 'nav2_params.yaml' if use_pid else 'nav2_params_no_pid.yaml'
    params_file_path = os.path.join(robot_nav_dir, 'config', nav2_param_file_name)

    base_launch_file_path = os.path.join(
        robot_nav_dir, # get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    return LaunchDescription([
            # SetRemap(src='/cmd_vel',dst='/expected_vel'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch_file_path),
                launch_arguments={
                    'use_sim_time': 'false',
                    'params_file': params_file_path, 
                    # 'remappings': '["/cmd_vel:=/expected_vel"]',
                }.items()
            ),
        ])
