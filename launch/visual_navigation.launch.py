"""
visual_navigation.launch.py — Visual SLAM localization + Nav2 autonomous navigation.

Run AFTER gazebo.launch.py AND after you have built a map with vslam.launch.py
(the map is persisted in ~/.ros/rtabmap.db).

RTAB-Map runs in localization mode:
  - Loads the saved map from rtabmap.db
  - Provides TF map → odom  (replaces AMCL)
  - Publishes /map           (replaces map_server)

Nav2 runs in navigation-only mode (navigation_launch.py):
  - controller_server, planner_server, behavior_server, bt_navigator, waypoint_follower
  - NO AMCL, NO map_server  (RTAB-Map covers both)

LiDAR (/scan) is still used by Nav2 costmap layers for real-time obstacle avoidance.

Usage:
  ros2 launch ict_bot_nav visual_navigation.launch.py
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
    nav2_params    = os.path.join(pkg, 'config', 'nav2_params.yaml')
    rviz_config    = os.path.join(pkg, 'config', 'nav2_rviz.rviz')

    # navigation_launch.py launches only the navigation nodes (no AMCL / map_server).
    # RTAB-Map provides the /map topic and the map→odom TF instead.
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
                'approx_sync_max_interval': 0.02,
                'queue_size': 10,
            }],
            remappings=[
                # libgazebo_ros_camera.so ignores <remapping> when camera_name is set.
                # Actual published topics are under /camera/depth_camera/ namespace.
                ('rgb/image',       '/camera/depth_camera/image_raw'),
                ('rgb/camera_info', '/camera/depth_camera/camera_info'),
                ('depth/image',     '/camera/depth_camera/depth/image_raw'),
            ]
        ),

        # ── 2. RTAB-Map (localization mode) ───────────────────────────────
        # Mem/IncrementalMemory=false  → do NOT add new nodes; only localise.
        # Mem/InitWMWithAllNodes=true  → load full saved map into working memory.
        # No --delete_db_on_start      → loads ~/.ros/rtabmap.db.
        # Publishes:
        #   /map          (nav_msgs/OccupancyGrid — loaded map for Nav2)
        #   /cloud_map    (PointCloud2 — 3D colored map for RViz)
        #   TF: map → odom
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
                }
            ],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                ('scan',       '/scan'),
                ('odom',       '/odom'),
            ],
        ),

        # ── 3. Nav2 navigation stack (without AMCL / map_server) ──────────
        # navigation_launch.py starts: controller_server, planner_server,
        # behavior_server, bt_navigator, waypoint_follower, lifecycle_manager.
        # The /map topic and map→odom TF come from RTAB-Map above.
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
