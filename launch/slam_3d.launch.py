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

    slam_params = os.path.join(pkg, 'config', 'slam_3d_params.yaml')

    return LaunchDescription([

        # Publishes /robot_description (for RViz robot model) — no TF, no conflict with Isaac Sim
        Node(
            package='ict_bot_nav',
            executable='urdf_publisher.py',
            name='urdf_publisher',
            output='screen',
        ),

        # base_link is a URDF alias of base_footprint (identity). Isaac Sim calls it link_base
        # and does not publish base_link TF, so we add it as a static transform here.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen',
        ),

        # Convert /point_cloud (PointCloud2) → /scan_3d (LaserScan)
        # Slices a horizontal band ±0.1 m around the lidar_3d frame
        # so slam_toolbox gets a clean 2D ring of points.
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            parameters=[{
                'use_sim_time': True,
                'target_frame': '',
                'transform_tolerance': 0.1,
                'min_height': -0.05,
                'max_height':  1.0,
                'angle_min': -3.14159,
                'angle_max':  3.14159,
                'angle_increment': 0.01745,   # ~1 degree
                'scan_time': 0.1,
                'range_min': 0.1,
                'range_max': 12.0,
                'use_inf': True,
                'qos_overrides./scan_3d.publisher.reliability': 'reliable',
            }],
            remappings=[
                ('cloud_in', '/point_cloud'),
                ('scan',     '/scan_3d'),
            ],
            output='screen',
        ),

        # slam_toolbox using /scan_3d
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': slam_params,
            }.items()
        ),

        # RViz — open manually and add: Map, LaserScan (/scan_3d), PointCloud2 (/point_cloud)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
