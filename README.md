# BBoxFlow

## Overview

BBoxFlow is a ROS2-based perception pipeline for processing LiDAR point cloud data, running object detection, and publishing detected 3D bounding boxes for visualization.

The system consists of two main components:

### 1. BBoxFlow ROS2 Workspace (this repository)
- Subscribes to LiDAR topics (defined in `bboxflow_configs.yaml`).
- Transforms point clouds into a global frame using configured sensor positions (defined in `rsu_coordinates.yaml`).
- Sends LiDAR data to the <b>Object Detection Core service</b>.
- Receives detection results (object centers, dimensions, orientations, labels, and scores).
- Publishes bounding boxes and processed point clouds for visualization in RViz.

### 2. Object Detection Core (Docker container)
- Runs inside a dedicated Ubuntu 20.04–based Docker container with CUDA, PyTorch, and required dependencies preinstalled.
- Provides an HTTP API (`/inference/lidar`) that receives LiDAR point clouds, performs inference with 3D object detection models, and returns detection results.
- Must be set up and running before using BBoxFlow.

---


## Object Detection Core Setup

BBoxFlow communicates with an external <b>Object Detection Core</b>, which is provided in a separate repository: 

- [hoosh-ir/object_detection](https://github.com/hoosh-ir/object_detection)

### Steps to set up:
1. Clone the core repository:
    ```bash
    git clone https://github.com/hoosh-ir/object_detection.git
    cd object_detection
    ```
2. Build and run the Docker container:
    ```bash
    sudo ./run_docker.sh run
    ```
    - This script will automatically download and install all required dependencies (≈5 GB).
    - The container exposes an inference API on localhost:8000, which BBoxFlow will use to send LiDAR data.
    - Leave the container running whenever you want to perform inference.


## BBoxFlow Modules

### `bbox_flow_node.py`
- Subscribes to configurable LiDAR topics (defined in `bboxflow_configs.yaml`).
- Transforms incoming point clouds from local sensor frames to a global coordinate frame using RSU coordinates (`rsu_coordinates.yaml`).
- Converts LiDAR data to Base64 and sends it to the Object Detection Core.
- Receives detection results containing:
  - Center (`x, y, z`)
  - Dimensions (`length, width, height`)
  - Rotation (`yaw angle in radians`)
  - Label (`object class`)
  - Score (`confidence`)
- Creates bounding boxes using Autoware perception message format.
- Publishes processed bounding boxes and LiDAR data in the global frame.

### `pcd_map_publisher.py`
- Publishes a static PCD map (`map.pcd`) once on the `map_points` topic for visualization in RViz.
- Uses Open3D to load and process the PCD file.
- Can be replaced with your environment’s own PCD map.

## Configuration Files
- `bboxflow_configs.yaml` — Defines active sensors and their ROS topics.
- `rsu_coordinates.yaml` — Contains global positions and orientations (`x, y, z, yaw, pitch, roll`) of LiDARs and cameras for frame transformation.


## Usage

### 1. Build the ROS2 workspace:

```bash
mkdir -p ~/bbox_ws/src
cd ~/bbox_ws/src
git clone https://github.com/hoosh-ir/BBoxFlow.git
cd ~/bbox_ws
colcon build
```

### 2. Run the Object Detection Core
In a separate terminal, start the Docker container from the object_detection repo:
```bash
./run_docker.sh run
```

### 3. Launch BBoxFlow
```bash
cd ~/bbox_ws
source install/setup.bash
ros2 launch bboxflow bboxflow.launch.py
```

### 4. Publish the PCD layout Map (Optional).
```bash
cd ~/bbox_ws
source install/setup.bash
ros2 run bboxflow pcd_map_publisher
```

### 5. Visualize in RViz
- <b>LiDAR topics:</b> Add PointCloud2 → select the sensor topic.
- <b>PCD map:</b> Add PointCloud2 → set topic to map_points and fixed frame to map.
- <b>Bounding boxes:</b> Add MarkerArray (or appropriate Autoware visualization).


### Dependencies and Notes on `pcd_map_publisher`

- ROS2 (tested with Humble)
- Open3D (used for PCD handling)
- NumPy must be pinned to <1.25 due to Open3D/SciPy compatibility:

    ```bash
    pip install "numpy<1.25" --force-reinstall
    ```
If you encounter import errors or warnings related to NumPy or SciPy, ensure you are using this version constraint.