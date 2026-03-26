# ict_bot_nav

ROS2 navigation package for ICT Bot — a differential-drive robot with 2D LiDAR and 3D point cloud sensor. Supports autonomous navigation using Nav2 and map building using SLAM Toolbox, with Isaac Sim as the simulator.

---

## Prerequisites

- ROS2 Humble
- Isaac Sim (with ROS2 bridge enabled)
- Nav2: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`
- SLAM Toolbox: `sudo apt install ros-humble-slam-toolbox`

---

## Build

```bash
cd ~/Documents/ros2_ws_ict_nav2
colcon build --packages-select ict_bot_nav
source install/setup.bash
```

---

## Usage

### 1. Start Isaac Sim

Open Isaac Sim and load your robot scene. Ensure the ROS2 bridge is active and the following topics are publishing:

| Topic | Type | Used For |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | AMCL, costmaps |
| `/point_cloud` | `sensor_msgs/PointCloud2` | 3D obstacle detection |
| `/odom` | `nav_msgs/Odometry` | Localization |
| `/tf` | TF tree including `lidar_3d` | Sensor transforms |

> In the Isaac Sim Action Graph, ensure the **Transform Tree** node publishes **before** the **Point Cloud** node to avoid TF timestamp sync issues.

---

### 2. Option A — Navigation with Existing Map

Use this when you already have a map (e.g. `isaac_room_map.yaml`).

```bash
ros2 launch ict_bot_nav navigation.launch.py
```

This starts:
- Nav2 stack (AMCL, planner, controller, costmaps)
- RViz2 with Nav2 panel

**Send a goal** using RViz2 (2D Nav Goal button) or the script:

```bash
ros2 run ict_bot_nav send_goal.py
```

**Send multiple waypoints:**

```bash
ros2 run ict_bot_nav send_multipoint.py
```

---

### 2. Option B — SLAM (Build a New Map)

Use this to drive the robot and build a map simultaneously.

```bash
ros2 launch ict_bot_nav slam.launch.py
```

Drive the robot manually to cover the environment. When done, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map_name
```

This saves `map_name.yaml` and `map_name.pgm`. Copy them to `maps/` and update `yaml_filename` in `config/nav2_params.yaml`.

## isaac sim scene (3d lidar)

https://drive.google.com/drive/folders/1atrdg5wlMmPaBRIeUtHRM9ydsIf9IJZ4?usp=drive_link

---

## Configuration

All Nav2 parameters are in [`config/nav2_params.yaml`](config/nav2_params.yaml).

| Parameter | Location | Description |
|---|---|---|
| `yaml_filename` | `map_server` | Absolute path to map yaml |
| `inflation_radius` | costmaps | Safety margin around obstacles |
| `desired_linear_vel` | `FollowPath` | Robot forward speed (m/s) |
| `allow_reversing` | `FollowPath` | Enable backward motion |
| `update_frequency` | global costmap | How often global map refreshes |

---

## Package Structure

```
ict_bot_nav/
├── config/
│   ├── nav2_params.yaml     # All Nav2 parameters
│   └── nav2_rviz.rviz       # RViz layout
├── launch/
│   ├── navigation.launch.py # Nav2 + RViz
│   ├── slam.launch.py       # SLAM + RViz
│   └── gazebo.launch.py     # Gazebo simulation (alternative to Isaac Sim)
├── maps/
│   ├── isaac_room_map.yaml  # Default navigation map
│   └── room_map.yaml        # Alternate map
├── scripts/
│   ├── send_goal.py         # Send single nav goal
│   └── send_multipoint.py   # Send waypoint sequence
└── urdf/
    └── ict_bot.urdf         # Robot description
```
