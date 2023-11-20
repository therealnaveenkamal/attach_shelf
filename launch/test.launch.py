from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('obstacle', default_value='1.0', description='Distance to the obstacle at which the robot will stop.'),
        DeclareLaunchArgument('degrees', default_value='0.0', description='Number of degrees for the rotation of the robot after stopping.'),
        Node(
            package='attach_shelf',
            executable='pre_approach_node',
            name='pre_approach_node',
            output='screen',
            parameters=[
                {'obstacle': LaunchConfiguration('obstacle')},
                {'degrees': LaunchConfiguration('degrees')},
            ],
        ),
    ])

