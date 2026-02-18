import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('ict_bot_nav')

    slam_launch = os.path.join(
        get_package_share_directory('slam_toolbox'),
        'launch', 'online_async_launch.py'
    )

    slam_params = os.path.join(pkg, 'config', 'nav2_params.yaml')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': slam_params
            }.items()
        ),

        # RViz to visualize the map being built
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),
    ])
