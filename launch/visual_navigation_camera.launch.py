"""
visual_navigation_camera.launch.py — RTAB-Map localization + Nav2 using RGB-D camera ONLY (no LiDAR).

Difference from visual_navigation.launch.py:
  - subscribe_scan: false        → RTAB-Map does NOT subscribe to /scan
  - scan remapping removed       → LiDAR completely ignored for localization
  - nav2_params_camera.yaml used → Nav2 costmaps use depth camera PointCloud2
                                    instead of LaserScan for obstacle detection

Run AFTER gazebo.launch.py AND after building a map with vslam_camera.launch.py.

Architecture:
  RGB-D camera  → rgbd_sync  → RTAB-Map (localization)
                                      ├  publishes /map       → Nav2 global costmap static layer
                                      └  publishes TF map→odom → Nav2 (replaces AMCL)

  Depth camera (/camera/depth_camera/points)  → Nav2 costmap obstacle_layer
                                                      (marks obstacles from depth cloud)

Usage:
  ros2 launch ict_bot_nav visual_navigation_camera.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg         = get_package_share_directory('ict_bot_nav')
    nav2_pkg    = get_package_share_directory('nav2_bringup')

    rtabmap_params = os.path.join(pkg, 'config', 'rtabmap_params.yaml')
    # Camera-only Nav2 params: obstacle_layer uses PointCloud2 instead of LaserScan
    nav2_params    = os.path.join(pkg, 'config', 'nav2_params_camera.yaml')
    rviz_config    = os.path.join(pkg, 'config', 'nav2_rviz.rviz')

    nav2_navigation_launch = os.path.join(
        nav2_pkg, 'launch', 'navigation_launch.py'
    )

    return LaunchDescription([

        #   0. Static TF: camera_link → camera_optical_link          
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '-1.5708', '--pitch', '0', '--yaw', '-1.5708',
                '--frame-id', 'camera_link',
                '--child-frame-id', 'camera_optical_link',
            ],
        ),

        #  1. RGB-D Sync                          
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'approx_sync': True,
                'approx_sync_max_interval': 0.8,     # Isaac Sim (higher jitter ~123ms)
                'queue_size': 50,                    # Isaac Sim
            }],
            remappings=[
                ('rgb/image',       '/camera/depth_camera/image_raw'),
                ('rgb/camera_info', '/camera/depth_camera/camera_info'),
                ('depth/image',     '/camera/depth_camera/depth/image_raw'),
            ]
        ),

        #   2. RTAB-Map (camera-only localization mode)            
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                rtabmap_params,
                {
                    'use_sim_time': True,
                    'Mem/IncrementalMemory': 'false',
                    'Mem/InitWMWithAllNodes': 'true',
                    'map_always_update': True,
                    #   Camera-only overrides                 
                    'subscribe_scan': False,
                    'RGBD/ProximityBySpace': 'false',
                    # estimate set in RViz (2D Pose Estimate button).
                    'Kp/MaxFeatures': '-1',

                    'Grid/3D': 'true',               # enable 3D occupancy grid
                    'Grid/Sensor': '1',              # 1 = depth camera (not LiDAR)
                    'Grid/CellSize': '0.05',         # 5cm voxel size
                    'Grid/RangeMax': '4.0',          # max depth range in meters
                    'Grid/RangeMin': '0.1',          # ignore points too close
                    'Grid/NormalsSegmentation': 'false',  # no floor filtering for now
                    'RGBD/CreateOccupancyGrid': 'true',
                    'cloud_voxel_size': 0.02,    # point cloud density
                    'cloud_max_depth': 4.0,

                    #   Localization registration: visual (Vis) not ICP      
                    'Reg/Strategy': '0',
                    # 'Mem/LocalizationDataSaved': 'true',
                    # 'publish_maps_in_background': 'false',
                    'Rtabmap/DetectionRate': '1',
                } 
            ],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                # ('scan', '/scan'),  # commented out — no LiDAR in this mode
                ('odom',            '/odom'),
            ],
        ),

        #   3. Nav2 navigation stack (camera-only obstacle detection)     
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_navigation_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file':  nav2_params,
            }.items()
        ),

        #   4. RViz                              
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config],
        ),
    ])