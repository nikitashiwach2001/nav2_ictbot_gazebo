"""
vslam.launch.py — Visual SLAM mapping mode using RTAB-Map + RGB-D camera.

Run AFTER gazebo.launch.py.  Drive the robot around to build the 3D map.
The map is saved automatically to ~/.ros/rtabmap.db.

To visualise the 3D map in RViz, add a PointCloud2 display on /cloud_map.

Usage:
  ros2 launch ict_bot_nav vslam.launch.py
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
        # Synchronises /camera/color/image_raw + /camera/depth/image_raw into
        # a single /rgbd_image topic that RTAB-Map consumes.
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

        # ── 2. RTAB-Map SLAM (mapping mode) ───────────────────────────────
        # --delete_db_on_start = fresh map every launch.
        # Mem/IncrementalMemory=true  → continuously add new nodes to the map.
        # map_always_update=true      → publish /map immediately on first frame.
        # Publishes:
        #   /map          (nav_msgs/OccupancyGrid — 2D, for Nav2)
        #   /cloud_map    (sensor_msgs/PointCloud2 — 3D colored map, for RViz)
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
                    'Mem/IncrementalMemory': 'true',
                    'Mem/InitWMWithAllNodes': 'false',
                    'map_always_update': True,
                }
            ],
            remappings=[
                ('rgbd_image', '/rgbd_image'),
                ('scan',       '/scan'),
                ('odom',       '/odom'),
            ],
            arguments=['--delete_db_on_start'],
        ),

        # ── 3. RViz ───────────────────────────────────────────────────────
        # Open with the existing nav2 config.
        # Manually add PointCloud2 → /rtabmap/cloud_map to see the 3D map.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config],
        ),
    ])
