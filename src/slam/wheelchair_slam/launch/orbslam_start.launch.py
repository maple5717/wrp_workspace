import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    orbslam3_share_dir = FindPackageShare('orbslam3_ros2').find('orbslam3_ros2')
    wheelchair_slam_share_dir = FindPackageShare('wheelchair_slam').find('wheelchair_slam')
    return LaunchDescription([
        # Launch the RealSense2 camera node
    
        # Launch the ORB-SLAM3 node
        Node(
            package='orbslam3',
            executable='orbslam3_node',  # Update this with the actual executable name
            name='orbslam3',
            output='screen',
            parameters=[
                {'vocabulary_file': str(orbslam3_share_dir) + '/vocabulary/ORBvoc.txt'},
                {'config_file': str(wheelchair_slam_share_dir) + 'config/RealSense_D435i.yaml'},
                {'depth_module.depth_profile': '640x480x30'},
                {'depth_module.infra_profile': '640x480x30'}, 
                {'rgb_camera.color_profile': '1280x720x30'}
            ],
            remappings=[
                # Add any required topic remappings here if necessary
            ],
        ),
    ])
