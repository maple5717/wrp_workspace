import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os 
def generate_launch_description():
    orbslam3_share_dir = FindPackageShare('orbslam3').find('orbslam3')
    wheelchair_slam_share_dir = FindPackageShare('wheelchair_slam').find('wheelchair_slam')
    voc_file =  str(orbslam3_share_dir) + '/vocabulary/ORBvoc.txt'
    camera_file = str(wheelchair_slam_share_dir) + '/config/RealSense_D435i.yaml'
    transformer_config_path = os.path.join(wheelchair_slam_share_dir, "config", "odom_transformer_params.yaml") 
    return LaunchDescription([

        # Launch the ORB-SLAM3 node
        Node(
            package='orbslam3',
            executable='rgbd-inertial',  # Update this with the actual executable name
            name='orbslam3',
            output='screen',
            parameters=[

            ],
            remappings=[
                # Add any required topic remappings here if necessary
            ],
            arguments=[ voc_file, camera_file]
        ),
        
        Node(
            package="odom_transformer",
            executable="transformer_node.py",
            name="odom_transformer",
            output={"both": {"screen", "log", "own_log"}},
            parameters=[transformer_config_path],
        ),

        Node(
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
        ])
