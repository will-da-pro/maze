import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    sllidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_c1_launch.py'
    )

    return LaunchDescription([
        Node(
            package='movement',
            executable='odom_publisher',
            name='odom_publisher',
            output='screen',
        ),

        Node(
            package='movement',
            executable='twist_subscriber',
            name='twist_subscriber',
            output='screen',
        ),

        Node(
            package='maze_navigator',
            executable='navigator_node',
            name='navigator_node',
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_path),
        ),
    ])
