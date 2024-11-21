from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pointcloud_to_laserscan = Node(
            package='pointcloud_to_laserscan', 
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', 'camera/camera/depth/color/points'),

                        ],
            parameters=[{
                'target_frame': 'camera_link',
                'transform_tolerance': 0.01,
                'min_height': -1.1,
                'max_height': 0.0,
                'angle_min': -3.1416,  # -M_PI/2
                'angle_max': 3.1416,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 5.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ) 

    config_dir = os.path.join(
        get_package_share_directory('wheelchair_slam'),
        'config',
    )

    slam_toolbox_launch_dir = get_package_share_directory('wheelchair_slam') + '/launch/'
    slam_toolbox_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                slam_toolbox_launch_dir + 'slam_toolbox_online_async.py'
            ]),
        )

    
    rviz_file = os.path.join(
        FindPackageShare('wheelchair_slam').find('wheelchair_slam'),
        'config',
        'wheelchair.rviz'
    )
    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': False}],
            output='screen')
    

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': open(os.path.join(config_dir, "wheelchair_robot.urdf"), 'r').read()
            }
            ]
    )

    wheelchair_slam_share_dir = FindPackageShare('wheelchair_slam').find('wheelchair_slam')
    ekf_config_file = str(wheelchair_slam_share_dir) + '/config/orb3_odom.yaml'
    ekf = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[ekf_config_file],
        
        )

    ld = LaunchDescription()

    ld.add_action(pointcloud_to_laserscan)
    ld.add_action(slam_toolbox_node)

    ld.add_action(rviz_node)
    ld.add_action(robot_state_publisher)
  
    return ld