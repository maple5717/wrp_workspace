from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from os.path import join

def generate_launch_description():
    realsense2_share_dir = FindPackageShare('realsense2_camera').find('realsense2_camera')
    
    return LaunchDescription([
        # Launch the RealSense2 camera node
        # Node(
        #     package='realsense2_camera',
        #     executable='realsense2_camera_node',
        #     name='realsense2_camera',
        #     output='screen',
        #     parameters=[
        #         {'unite_imu_method': 2},
        #         {'align_depth.enable': True},
        #         {'enable_accel': True},
        #         {'enable_gyro': True},
        #         {'depth_module.depth_profile': '640x480x30'},
        #         {'depth_module.infra_profile': '640x480x30'}, 
        #         {'rgb_camera.color_profile': '1280x720x30'}
        #     ],
        #     remappings=[
        #         # Add any required topic remappings here if necessary
        #     ],
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(join(realsense2_share_dir, 'launch/rs_launch.py')),
            launch_arguments={
                'unite_imu_method': '2',
                'align_depth.enable': 'true',
                'enable_accel': 'true',
                'enable_gyro': 'true',
                'depth_module.depth_profile': '640x480x30',
                # 'depth_module.infra_profile': '640x480x30', 
                'rgb_camera.color_profile': '640x480x30'
            }.items()
        ),
        
    
    ])
