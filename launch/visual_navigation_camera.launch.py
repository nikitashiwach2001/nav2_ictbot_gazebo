"""
visual_navigation_camera.launch.py — RTAB-Map localization + Nav2 using RGB-D camera ONLY (no LiDAR).

Difference from visual_navigation.launch.py:
  - subscribe_scan: false        → RTAB-Map does NOT subscribe to /scan
  - scan remapping removed       → LiDAR completely ignored for localization
  - nav2_params_camera.yaml used → Nav2 costmaps use depth camera PointCloud2
                                    instead of LaserScan for obstacle detection

Run AFTER gazebo.launch.py AND after building a map with vslam_camera.launch.py.

Architecture:
  RGB-D camera ──→ rgbd_sync ──→ RTAB-Map (localization)
                                      ├── publishes /map       → Nav2 global costmap static layer
                                      └── publishes TF map→odom → Nav2 (replaces AMCL)

  Depth camera (/camera/depth_camera/points) ──→ Nav2 costmap obstacle_layer
                                                      (marks obstacles from depth cloud)

Usage:
  ros2 launch ict_bot_nav visual_navigation_camera.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
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

        # ── 1. RGB-D Sync ──────────────────────────────────────────────────
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'approx_sync': True,
                # 'approx_sync_max_interval': 0.02,  # Gazebo (low jitter)
                # 'queue_size': 10,                  # Gazebo
                'approx_sync_max_interval': 0.1,     # Isaac Sim (higher jitter ~123ms)
                'queue_size': 30,                    # Isaac Sim
            }],
            remappings=[
                ('rgb/image',       '/camera/depth_camera/image_raw'),
                ('rgb/camera_info', '/camera/depth_camera/camera_info'),
                ('depth/image',     '/camera/depth_camera/depth/image_raw'),
            ]
        ),

        # ── 2. RTAB-Map (camera-only localization mode) ───────────────────
        # subscribe_scan: false  → ignore /scan, localize using RGB-D only.
        # Mem/IncrementalMemory: false → localization only, no new map nodes.
        # Mem/InitWMWithAllNodes: true → load full saved map from rtabmap.db.
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
                    # ── Camera-only overrides ─────────────────────────────
                    'subscribe_scan': False,
                    'RGBD/ProximityBySpace': 'false',

                    # Disable visual loop closure (same reason as vslam_camera.launch.py:
                    # textureless Gazebo walls cause perceptual aliasing → false closures
                    # → position jumps). In localization mode with Kp/MaxFeatures=0,
                    # RTAB-Map tracks position using wheel odometry from the initial pose
                    # estimate set in RViz (2D Pose Estimate button).
                    'Kp/MaxFeatures': '0',
                }
            ],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                # ('scan', '/scan'),  # commented out — no LiDAR in this mode
                ('odom',       '/odom'),
            ],
        ),

        # ── 3. Nav2 navigation stack (camera-only obstacle detection) ─────
        # Uses nav2_params_camera.yaml where obstacle_layer reads
        # /camera/depth_camera/points (PointCloud2) instead of /scan (LaserScan).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_navigation_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file':  nav2_params,
            }.items()
        ),

        # ── 4. RViz ───────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config],
        ),
    ])
