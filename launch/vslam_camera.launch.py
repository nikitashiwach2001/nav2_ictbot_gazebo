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

        Node(
            package='ict_bot_nav',
            executable='camera_info_pub',
            name='camera_info_pub',
            output='screen',
        ),

        # ── 0. Static TF: camera_link → camera_optical_link ───────────────
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

        # ── 1. RGB-D Sync ──────────────────────────────────────────────────
        # Synchronises raw rgb + depth topics into /rgbd_image for RTAB-Map.
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'approx_sync': True,
                'approx_sync_max_interval': 0.4, 
                'queue_size': 10,
            }],
            remappings=[
                ('rgb/image',       '/camera/depth_camera/image_raw'),
                ('rgb/camera_info', '/camera/depth_camera/camera_info'),
                ('depth/image',     '/camera/depth_camera/depth/image_raw'),
            ]
        ),

        # ── 2. RTAB-Map SLAM (camera-only mapping mode) ───────────────────
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
                    # Camera + LiDAR SLAM ───────────────────────────────
                    'subscribe_scan': True,    # fuse LiDAR scan for clean wall mapping
                    'RGBD/ProximityBySpace': 'true',  # LiDAR makes this reliable
                    'Kp/MaxFeatures': '-1',

                    'database_path': '/home/user/Documents/ros2_ws_ict_nav2/src/ict_bot_nav/maps/office_3d_map.db',

                    # 3D point cloud (dense, detailed) ─────────────────────
                    'cloud_voxel_size': 0.02,        # 2cm voxels → captures chair legs
                    'cloud_decimation': 1,           # all depth pixels
                    'cloud_max_depth': 8.0,          # full office coverage
                    'cloud_min_depth': 0.3,

                    # 2D occupancy grid — LiDAR only, flat 2D (no projection artifacts)
                    'Grid/3D': 'false',              # flat 2D grid: no wall-to-floor projection
                    'Grid/Sensor': '0',              # 0 = LiDAR only for grid (clean 360° walls)
                    'Grid/CellSize': '0.05',
                    'Grid/RangeMin': '0.1',
                    'Grid/RangeMax': '8.0',
                    'Grid/GroundIsObstacle': 'false',
                    'Grid/MaxObstacleHeight': '0.8', # chairs + desk legs, not ceiling
                    'Grid/RayTracing': 'false',       # LiDAR ray tracing is clean — safe to enable

                    # Noise filtering ───────────────────────────────────────
                    'Grid/NormalsSegmentation': 'false',
                    'Grid/NoiseFilteringRadius': '0.1',
                    'Grid/NoiseFilteringMinNeighbors': '2',

                    # Registration & localization ───────────────────────────
                    'Reg/Strategy': '2',             # 0 = Visual only (no LiDAR/ICP)

                }
            ],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                ('scan',       '/scan'),
                ('odom',       '/odom'),
            ],
            # arguments=['--delete_db_on_start'],
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