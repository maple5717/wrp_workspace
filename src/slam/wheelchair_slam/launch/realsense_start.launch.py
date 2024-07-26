import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # realsense2_share_dir = FindPackageShare('realsense2_camera').find('realsense2_camera')
    
    return LaunchDescription([
        # Launch the RealSense2 camera node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            output='screen',
            parameters=[
                {'unite_imu_method': 2},
                {'align_depth.enable': True},
                {'enable_accel': True},
                {'enable_gyro': True},
            ],
            remappings=[
                # Add any required topic remappings here if necessary
            ],
        ),
        
    
    ])
