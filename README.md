# ict_bot_nav — ROS2 Nav2 + RTAB-Map Navigation

Differential drive robot with RGB-D camera + 2D LiDAR for 3D mapping and autonomous navigation.
Simulator: **Isaac Sim** | ROS2: **Humble**

---

## Requirements

```bash
sudo apt install ros-humble-rtabmap-ros ros-humble-nav2-bringup ros-humble-slam-toolbox
```

---

## Build

```bash
cd ~/Documents/ros2_ws_ict_nav2
colcon build --symlink-install
source install/setup.bash
```

> Clean build:
> ```bash
> rm -rf build install log
> colcon build --symlink-install
> ```

---

## 1. Build a Map

Start Isaac Sim with the robot scene, then:

```bash
source install/setup.bash
ros2 launch ict_bot_nav vslam_camera.launch.py
```

Drive the robot around to build the map.
Saves automatically to `src/ict_bot_nav/maps/office_3d_map.db`

> Fresh remap:
> ```bash
> rm src/ict_bot_nav/maps/office_3d_map.db
> ```

---

## 2. Run Navigation

Start Isaac Sim first, wait for topics, then:

```bash
source install/setup.bash
ros2 launch ict_bot_nav visual_navigation_camera.launch.py
```

**In RViz:**
1. **2D Pose Estimate** — click robot position, drag facing direction
2. Wait 2–3s for localization
3. **Nav2 Goal** — click destination

---

## Project Structure

```
src/ict_bot_nav/
├── launch/
│   ├── vslam_camera.launch.py               # mapping
│   └── visual_navigation_camera.launch.py   # navigation
├── config/
│   ├── rtabmap_params.yaml                  # RTAB-Map / ICP tuning
│   ├── nav2_params_camera.yaml              # Nav2 costmap, planner, recovery
│   └── navigate_to_pose_w_replanning_and_recovery.xml  # custom BT
├── maps/
│   └── office_3d_map.db                     # RTAB-Map database
└── urdf/
    └── ict_bot.urdf
```

---

## Sensors

| Sensor | Topic | Role |
|--------|-------|------|
| RGB-D Camera | `/camera/depth_camera/*` | 3D point cloud, local obstacle detection |
| 2D LiDAR | `/scan` | ICP localization, 2D occupancy grid |

---

## Known Issues

- Chair wheels not always in static map — local costmap handles them at runtime
- Plain walls cause visual feature warnings — ICP-only localization (`Reg/Strategy: 1`) handles this
- If map drifts after multiple nav runs, delete `.db` and remap
