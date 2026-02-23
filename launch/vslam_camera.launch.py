"""
vslam_camera.launch.py — Visual SLAM mapping using RTAB-Map + RGB-D camera ONLY (no LiDAR).

Difference from vslam.launch.py:
  - subscribe_scan: false  → RTAB-Map does NOT subscribe to /scan
  - scan remapping removed → LiDAR data is completely ignored
  - SLAM relies purely on RGB-D visual odometry + wheel odometry

Run AFTER gazebo.launch.py.  Drive the robot around to build the 3D map.
The map is saved automatically to ~/.ros/rtabmap.db.

Limitation: Without LiDAR, loop closure detection depends entirely on visual
features. Textureless / uniform-color environments (plain white walls) may
produce fewer loop closures than with LiDAR.

Usage:
  ros2 launch ict_bot_nav vslam_camera.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('ict_bot_nav')

    rtabmap_params = os.path.join(pkg, 'config', 'rtabmap_params.yaml')
    rviz_config    = os.path.join(pkg, 'config', 'nav2_rviz.rviz')

    return LaunchDescription([

        # ── 1. RGB-D Sync ──────────────────────────────────────────────────
        # Gazebo mode: subscribe_rgbd=True → rgbd_sync is needed
        # Isaac Sim mode: subscribe_depth=True → RTAB-Map subscribes directly; rgbd_sync not needed
        # Node(
        #     package='rtabmap_sync',
        #     executable='rgbd_sync',
        #     name='rgbd_sync',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': True,
        #         'approx_sync': True,
        #         # 'approx_sync_max_interval': 0.02,  # Gazebo (low jitter)
        #         # 'queue_size': 10,                  # Gazebo
        #         'approx_sync_max_interval': 0.1,     # Isaac Sim (higher jitter ~123ms)
        #         'queue_size': 30,                    # Isaac Sim
        #     }],
        #     remappings=[
        #         ('rgb/image',       '/camera/depth_camera/image_raw'),
        #         ('rgb/camera_info', '/camera/depth_camera/camera_info'),
        #         ('depth/image',     '/camera/depth_camera/depth/image_raw'),
        #     ]
        # ),

        # ── 2. RTAB-Map SLAM (camera-only mapping mode) ───────────────────
        # subscribe_scan: false  → ignore /scan completely, RGB-D only.
        # All other settings same as vslam.launch.py.
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                rtabmap_params,
                {
                    'use_sim_time': True,
                    'Mem/IncrementalMemory': 'true',
                    'Mem/InitWMWithAllNodes': 'false',
                    'map_always_update': True,
                    # ── Camera-only overrides ─────────────────────────────
                    'subscribe_scan': False,   # do NOT use LiDAR scan
                    'RGBD/ProximityBySpace': 'false',

                    # Disable visual loop closure detection completely.
                    #
                    # WHY: Gazebo uses uniform/textureless walls (same color, no
                    # patterns). RTAB-Map's feature detector (ORB) finds visually
                    # similar features in completely different locations
                    # ("perceptual aliasing"). Some false pairs accidentally pass
                    # the geometric inlier check → loop closure accepted → pose
                    # graph reoptimized → robot position jumps in RViz → map
                    # builds at wrong points.
                    #
                    # EFFECT: Map is built purely from depth images + wheel
                    # odometry. No visual drift correction, but no jumps either.
                    # Gazebo's diff_drive odometry is accurate enough for
                    # reliable map building without loop closure.
                    #
                    # To re-enable loop closure (real robot / textured env):
                    # comment out the line below.
                    'Kp/MaxFeatures': '0',
                }
            ],
            remappings=[
                # Gazebo mode (subscribe_rgbd=True): uncomment below
                # ('rgbd_image', '/rgbd_image'),
                # Isaac Sim mode (subscribe_depth=True): subscribe directly to raw topics
                ('rgb/image',       '/camera/depth_camera/image_raw'),
                ('rgb/camera_info', '/camera/depth_camera/camera_info'),
                ('depth/image',     '/camera/depth_camera/depth/image_raw'),
                # ('scan', '/scan'),  # commented out — no LiDAR in this mode
                ('odom',            '/odom'),
            ],
            arguments=['--delete_db_on_start'],
        ),

        # ── 3. RViz ───────────────────────────────────────────────────────
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config],
        ),
    ])
