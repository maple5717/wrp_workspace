from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
   

    
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


    ld = LaunchDescription()

    ld.add_action(rviz_node)

    return ld