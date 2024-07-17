'''
Author: Ibrahim Hroob <ihroob@lincoln.ac.uk> 2024
'''
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('ros2_topic_monitor')
    cfg_pth_default = os.path.join(package_share_directory,'config', 'cfg_topics_monitor.yaml')
    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument('cfg_pth', default_value=cfg_pth_default), #yaml file config path for the topics to be monitored

        # Launch the manual_topomapping node
        Node(
            package='ros2_topic_monitor',
            executable='monitor.py',
            name='ros2_topic_monitor',
            output='screen',
            parameters=[{
                'cfg_pth' : LaunchConfiguration('cfg_pth'),
            }]
        ),
    ])
