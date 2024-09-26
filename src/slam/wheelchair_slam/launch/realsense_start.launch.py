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
                'unite_imu_method': '1',
                'align_depth.enable': 'true',
                'enable_accel': 'true',
                'enable_gyro': 'true',
                'enable_sync': 'true',
                # 'enable_infra1': 'true',
                # 'enable_infra2': 'true', 
                'enable_depth': 'true',  #
                'depth_module.global_time_enabled': 'true',
                'rgb_camera.global_time_enabled': 'true',
                'tf_publish_rate': '1.0',
                'pointcloud.enable': 'true',
                'pointcloud.allow_no_texture_points': 'false',
                # 'pointcloud.ordered_pc': 'true',
                # 'depth_module.emitter_enabled': 'false', #
                # 'depth_module.emitter_always_on': 'false', #
                'initial_reset': 'true',
                'depth_module.depth_profile': '640x480x15',
                # 'depth_module.infra_profile': '640x480x30', 
                'rgb_camera.color_profile': '640x480x15', 
                'rgb_camera.power_line_frequency': '1'
            }.items()
        ),
        
    
    ])
