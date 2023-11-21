from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='attach_shelf',
            executable='approach_service_server',
            name='approach_service_server',
            output='screen',
        ), Node(
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
