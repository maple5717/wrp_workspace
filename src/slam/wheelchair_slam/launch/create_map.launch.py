from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # use_sim_time = LaunchConfiguration('use_sim_time')

    # declare_use_sim_time_argument = DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='false',
    #     description='Use simulation (Gazebo) clock if true'),
    # use_record_data = DeclareLaunchArgument(
    #     'use_record_data',
    #     default_value='false',
    #     description='Whether to use recorded data (true/false)'
    # )

    # temporary frame definition
    # base_tf = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'base_link'],
    #         output='screen',
    #     )
    
    # odom_tf = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
    #         output='screen',
    #     )

    
    camera_base_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_base_tf',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'camera_color_optical_frame'],
            # output='screen',
        )
    lidar_base_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_base_tf', 
            arguments=['0', '0', '1.657', '0', '0', '0', '1', 'base_link', 'rslidar'],
            # output='screen',
        )
    # arguments=['0', '0', '1.2', '0.5', '0.5', '0.5', '0.5', 'base_link', 'camera_link'],
    # arguments=['0', '0', '1.2', '0', '0', '0', '1', 'base_link', 'camera_link'],
    
    # fake_tf_publisher = Node(
    #         # package=FindPackageShare('wheelchair_slam').find('wheelchair_slam'),  # Replace with your actual package name
    #         executable='fake_lidar_tf.py',  # Replace with your script name
    #         name='fake_lidar_tf',
    #         output='screen',
    #     )



    pointcloud_to_laserscan = Node(
            package='pointcloud_to_laserscan', 
            executable='pointcloud_to_laserscan_node',
            remappings=[
                ('cloud_in', 'rslidar_points'),
                # ('scan', ['scanner', '/scan'])
                        ],
            parameters=[{
                'target_frame': 'rslidar',
                'transform_tolerance': 0.01,
                'min_height': 0.0,
                'max_height': 2.0,
                'angle_min': -3.1416,  # -M_PI/2
                'angle_max': 3.1416,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.1,
                'range_min': 0.2,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ) 


    pkg_cartographer_ros = FindPackageShare('cartographer_ros').find('cartographer_ros')
    config_dir = os.path.join(
        get_package_share_directory('wheelchair_slam'),
        'config',
    )

    # create maps
    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        arguments = [
            '-configuration_directory', config_dir,
            '-configuration_basename', 'wheelchair_2d.lua'
            ],
        remappings = [
            # ('/odom', '/mobile_base_node/odom')
            # ('/odom', '/kimera_vio_ros/odometry')
            # ('/odom', '/vins_estimator/odometry')
            ('/odom', '/transformed_odom')
            ],
        output = 'screen'
        )
    
    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': False},
            {'resolution': 0.05}],
        )

    # amcl = Node(   
    #     package='nav2_map_server',
    #     executable='map_server',
    #     name='map_server',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}, 
    #                 {'yaml_filename':map_file} 
    #                 ]),
    

    
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

    transformer_config_path = os.path.join(wheelchair_slam_share_dir, "config", "odom_transformer_params.yaml")

    odom_transformer = Node(
        package="odom_transformer",
        executable="transformer_node.py",
        name="odom_transformer",
        output={"both": {"screen", "log", "own_log"}},
        parameters=[transformer_config_path],
    )

    odom2tf = Node(
        package="odom_to_tf_ros2",
        executable="odom_to_tf",
        name="odom_to_tf",
        output={"both": {"screen", "log", "own_log"}},
        parameters=[{
                'frame_id': 'odom',        
                'child_frame_id': 'base_link',   
                'odom_topic': '/transformed_odom'             
            }]
    )

    ld = LaunchDescription()

    # ld.add_action(declare_use_sim_time_argument)
    # ld.add_action(start_sync_slam_toolbox_node)
    # ld.add_action(base_tf)
    # ld.add_action(odom_tf)
    ld.add_action(camera_base_tf)
    # ld.add_action(lidar_base_tf)
    # ld.add_action(fake_tf_publisher)

    # ld.add_action(pointcloud_to_laserscan)
    ld.add_action(cartographer_node)
    ld.add_action(cartographer_occupancy_grid_node)

    ld.add_action(rviz_node)
    ld.add_action(robot_state_publisher)
    ld.add_action(odom_transformer)
    ld.add_action(odom2tf)
    # ld.add_action(ekf)
    return ld