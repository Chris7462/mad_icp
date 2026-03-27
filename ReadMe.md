# mad_icp

![License](https://img.shields.io/badge/license-Apache--2.0-green)
![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![C++](https://img.shields.io/badge/C++-17-blue)

A ROS2 wrapper package for [mad_icp_core](https://github.com/Chris7462/mad_icp_core),
providing a `lidar_odometry_node` that subscribes to raw LiDAR point clouds and
publishes odometry, path, and map topics.

This package is the ROS2 companion to `mad_icp_core`, which implements the core
algorithm of [MAD-ICP: It Is All About Matching Data — Robust and Informed LiDAR Odometry](https://github.com/rvp-group/mad-icp)
(RA-L 2024).

---

## Nodes

### `lidar_odometry_node`

The single node that drives the full MAD-ICP odometry pipeline.

#### Subscribed Topics

| Topic | Type | Description |
|---|---|---|
| `<input_topic>` | `sensor_msgs/msg/PointCloud2` | Raw LiDAR point cloud (configurable, default `/kitti/velo`) |

#### Published Topics

| Topic | Type | Description |
|---|---|---|
| `odom` | `nav_msgs/msg/Odometry` | Estimated LiDAR odometry |
| `odom_path` | `nav_msgs/msg/Path` | Full odometry trajectory path |
| `map` | `sensor_msgs/msg/PointCloud2` | Accumulated keyframe map (published when map updates) |

---

## Parameters

All parameters are set in `params/mad_icp_params.yaml`.

| Parameter | Type | Default | Description |
|---|---|---|---|
| `sensor_hz` | double | `10.0` | LiDAR scan frequency (Hz) |
| `min_range` | double | `0.7` | Minimum point range filter (m) |
| `max_range` | double | `120.0` | Maximum point range filter (m) |
| `deskew` | bool | `false` | Enable motion deskewing of point clouds |
| `b_max` | double | `0.2` | Max size of MAD-tree leaf nodes (m) |
| `b_min` | double | `0.1` | Min node size; propagate normal when flatter than this (m) |
| `b_ratio` | double | `0.02` | Search radius increase factor for data association |
| `p_th` | double | `0.8` | Keyframe promotion threshold (fraction of registered points) |
| `rho_ker` | double | `0.1` | Huber robust kernel threshold in MAD-ICP |
| `num_keyframes` | int | `4` | Max number of keyframes kept in the local map |
| `num_threads` | int | `4` | Number of threads for parallel ICP |
| `processing_frequency` | double | `50.0` | Timer frequency for point cloud processing (Hz) |
| `max_processing_queue_size` | int | `3` | Max queue size before dropping old frames |
| `queue_size` | int | `10` | ROS2 subscriber QoS history depth |
| `input_topic` | string | `/kitti/velo` | Input point cloud topic |
| `output_odom_topic` | string | `odom` | Output odometry topic |
| `output_path_topic` | string | `odom_path` | Output path topic |
| `output_map_topic` | string | `map` | Output map topic |

---

## Dependencies

| Dependency | Notes |
|---|---|
| ROS2 (Jazzy) | [Installation](https://docs.ros.org/en/jazzy/Installation.html) |
| mad_icp_core | [mad_icp_core](https://github.com/Chris7462/mad_icp_core) |
| Eigen3 | `sudo apt-get install libeigen3-dev` |
| PCL | `sudo apt-get install libpcl-dev` |
| pcl_conversions | ROS2 package |
| tf2, tf2_ros, tf2_eigen | ROS2 packages |

---

## Building

Clone both `mad_icp_core` and `mad_icp` into your ROS2 workspace and build together:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Chris7462/mad_icp_core.git
git clone https://github.com/Chris7462/mad_icp.git
cd ~/ros2_ws
colcon build --packages-select mad_icp_core mad_icp
source install/setup.bash
```

---

## Usage

All parameters are configured in `params/mad_icp_params.yaml`. Edit this file
to match your LiDAR sensor and topic names before launching.

### Launch odometry only

```bash
ros2 launch mad_icp mad_icp_launch.py
```

### Launch with RViz and rosbag playback

```bash
ros2 launch mad_icp mad_icp_rviz_launch.py
```

---

## Related Packages

**[mad_icp_core](https://github.com/Chris7462/mad_icp_core)** — The ROS-agnostic pure
C++ algorithm library that this package wraps.

---
