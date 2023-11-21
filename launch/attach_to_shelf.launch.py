import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    pkg_name = 'attach_shelf'
    rviz_config_dir = os.path.join(get_package_share_directory(pkg_name), 'rviz_config')
    rviz_config_file = os.path.join(rviz_config_dir, 'rvizconfigv1.rviz')

    return LaunchDescription([
        Node(
            package='attach_shelf',
            executable='approach_service_server',
            name='approach_service_server',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
        Node(
            package='attach_shelf',
            executable='pre_approach_v2_node',
            name='pre_approach_v2_node',
            parameters=[
                {'obstacle': LaunchConfiguration('obstacle', default='0.0')},
                {'degrees': LaunchConfiguration('degrees', default='0.0')},
                {'final_approach': LaunchConfiguration('final_approach', default='false')}
            ],
        ),
    ])
