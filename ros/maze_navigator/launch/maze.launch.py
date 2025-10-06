import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode, Node


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

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch_path),
        ),

        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
        ),

        LifecycleNode(
            package='maze_navigator',
            executable='wall_sensor_node',
            name='wall_sensor_node',
            namespace='',
            output='screen',
        ),

        LifecycleNode(
            package='maze_navigator',
            executable='camera_subscriber_node',
            name='camera_subscriber_node',
            namespace='',
            output='screen',
        ),

        Node(
            package='maze_navigator',
            executable='navigator_node',
            name='navigator_node',
            output='screen',
        ),
    ])
