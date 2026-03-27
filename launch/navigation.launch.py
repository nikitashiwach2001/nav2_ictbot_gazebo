import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg = get_package_share_directory('ict_bot_nav')

    nav2_params  = os.path.join(pkg, 'config', 'nav2_params.yaml')
    default_map  = os.path.join(pkg, 'maps', 'office_map_partcial.yaml')

    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map yaml file'
    )
    map_file = LaunchConfiguration('map')


    rviz_config  = os.path.join(pkg, 'config', 'nav2_rviz.rviz')

    nav2_launch = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch', 'bringup_launch.py'
    )

    return LaunchDescription([
        declare_map,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': nav2_params
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config]
        ),
    ])
