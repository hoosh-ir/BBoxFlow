# BBoxFlow

## Overview

BBoxFlow is a ROS2-based project for processing LiDAR point cloud data and publishing detected bounding boxes for objects in a mapped environment. It includes modules for subscribing to multiple LiDAR sensors, transforming their point clouds into their global frame using the actual sensors global position, and publishing a static map point cloud for visualization.

---

## Modules

### bbox_flow_node.py

- Subscribes to configurable LiDAR topics (defined in `bboxflow_configs.yaml`).
- Transforms incoming point clouds from local sensor frames to a global coordinate frame using RSU coordinates (`rsu_coordinates.yaml`).
- Publishes detected objects as dummy bounding boxes (placeholder detection logic, to be extended).
- Publishes transformed global point clouds for visualization or further processing.

### pcd_map_publisher.py

- Publishes a static PCD map (`map.pcd`) once on the `map_points` topic for visualization in RViz.
- Uses `open3d` to load and process the PCD file.

---

## Configuration Files

- `bboxflow_configs.yaml` — Defines which sensors are active and their ROS topics.
- `rsu_coordinates.yaml` — Contains the position (x, y, z) and orientation (yaw, pitch, roll) of RSU LiDARs and cameras used for transforming point clouds.



---

## Usage

1. Build the ROS2 workspace:

```bash
mkdir -p ~/bbox_ws/src
cd ~/bbox_ws/src
git clone https://github.com/hoosh-ir/BBoxFlow.git
cd ~/bbox_ws
colcon build
```

2. run object detection
```bash
cd ~/bbox_ws
source install/setup.bash
ros2 launch bboxflow bboxflow.launch.py
```

3. Publish map.pcd for visualization in RViZ.
```bash
cd ~/bbox_ws
source install/setup.bash
ros2 run bboxflow pcd_map_publisher
```

4. in RViZ, for displaying lidar topics and PCD map
- Lidar topics: Click add by display type --> add pointcloud2 --> select the topic you want to display
- PCD map: Click add by display type --> add pointcloud2 --> enter `map_points` as topic name and `map` as fixed frame




### Dependencies and Notes on `pcd_map_publisher`

- The pcd_map_publisher uses Open3D for reading PCD files.
- Due to compatibility requirements, NumPy version must be constrained to <1.25. To ensure compatibility, run:

If you encounter import errors or warnings related to NumPy or SciPy, please ensure that your environment uses NumPy version less than 1.25 by running:

```bash
pip install "numpy<1.25" --force-reinstall
```

This constraint avoids import errors caused by incompatibilities between Open3D, SciPy, and newer NumPy versions.