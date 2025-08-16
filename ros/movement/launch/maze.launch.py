import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('movement'), 'configuration_files'
    )

    cartographer_config_basename = 'cartographer_config.lua'
    nav2_config_basename = 'nav2_config.yaml'

    sllidar_launch_path = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_c1_launch.py'
    )

    nav2_launch_path = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch',
        'navigation_launch.py'
    )

    nav2_config_path = os.path.join(
        config_dir,
        nav2_config_basename
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_0',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_1',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_2',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'laser'],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', cartographer_config_basename,
            ],
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=[
                '-resolution', '0.05',
            ],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={'params_file': nav2_config_path}.items()
        ),
    ])
