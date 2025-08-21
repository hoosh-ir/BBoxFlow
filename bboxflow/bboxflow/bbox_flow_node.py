#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import MarkerArray, Marker
from ament_index_python.packages import get_package_share_directory
import random
import os
import yaml
import collections.abc 
import numpy as np
from autoware_perception_msgs.msg import DetectedObjects, DetectedObject, Shape, ObjectClassification, DetectedObjectKinematics
from geometry_msgs.msg import Pose, Vector3, PoseWithCovariance, Quaternion
import math
from std_msgs.msg import Header
import requests
import base64
import time



class BBoxFlow(Node):
    def __init__(self):
        super().__init__('bbox_flow')

        # Uncomment if you want to get the topics name form launch file
        self.declare_parameter('lidar_topic', '/rsu/intersection2/livox_avia_2/pointcloud')
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.get_logger().info(f"Using LiDAR topic: {self.lidar_topic}")

        ## Uncomment if you want to get the topics from config files
        # self.configs = self.config_file_loader('bboxflow_configs.yaml')
        # self.lidar_topic = self.configs['object_detection_input']['lidars']['topics']

        self.rsu_coordinates = self.config_file_loader('rsu_coordinates.yaml')
        self.lidars_coordinates = self.rsu_coordinates['rsu_coordinates']['lidars']

        self.subscription = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            10)
        self.get_logger().info(f'BBoxFlow Node Started... Subscribed to: {self.lidar_topic}')
        
        self.bbox_topic = self.lidar_topic + '/detected_objects'
        self.detected_objects_pub = self.create_publisher(DetectedObjects, self.bbox_topic, 10)

        self.transformed_lidar_topic = self.lidar_topic + '/global'
        self.transformed_lidar_pub = self.create_publisher(PointCloud2, self.transformed_lidar_topic, 10)

        self.lidar_name = 'lidar' + self.lidar_topic.split('/')[3].split('_')[2]
        # self.lidar_name = self.lidar_topic.split('/sim/')[1]

        self.lidar_data = None

        
    def config_file_loader(self, file_name):
        # Get the share directory of bboxflow package
        bboxflow_pkg_share = get_package_share_directory('bboxflow')

        # Correct path to configs directory inside bboxflow
        config_path = os.path.join(bboxflow_pkg_share, 'configs', file_name)
        # Load the configuration file
        with open(config_path, "r") as file:
            config = yaml.safe_load(file)
        return config

    def lidar_callback(self, msg):
        self.get_logger().info(f"Received LiDAR frame with {msg.width} points")

        # This is a list of numpy structured elements
        points_list = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))

        # Extract columns explicitly:
        x_vals = np.array([p['x'] for p in points_list], dtype=np.float32)
        y_vals = np.array([p['y'] for p in points_list], dtype=np.float32)
        z_vals = np.array([p['z'] for p in points_list], dtype=np.float32)

        # Stack into (N, 4) array:
        self.lidar_data = np.column_stack((x_vals, y_vals, z_vals))
        self.get_logger().info(f"lidar data before concat structure {self.lidar_data.shape}, dtype: {self.lidar_data.dtype}")

        # === Transform to global frame ===
        self.transform_points(self.lidar_data, self.lidars_coordinates, msg)

        # === concat a column of 0 for intensity ===
        self.lidar_data = np.concatenate([self.lidar_data, np.ones((self.lidar_data.shape[0], 1))], axis=1)

        self.get_logger().info(f"self.lidar_data shape: {self.lidar_data.shape}, dtype: {self.lidar_data.dtype}")
        self.object_detection()


    @staticmethod
    def transform_point(point, coordinates):
        """
        Transform a single point [x, y, z] from local LiDAR frame to global frame.
        """
        tx = coordinates['x']
        ty = coordinates['y']
        tz = coordinates['z']
        yaw_unity_frame = coordinates['yaw']
        yaw_deg = (0 - yaw_unity_frame) % 360
        pitch_deg = coordinates['pitch']
        roll_deg = coordinates['roll']

        # Convert to radians
        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        roll = math.radians(roll_deg)

        # Rotation matrices
        Rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw),  math.cos(yaw), 0],
            [0, 0, 1]
        ])

        Ry = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll),  math.cos(roll)]
        ])

        # Combined rotation
        R = Rz @ Ry @ Rx

        # Apply transform
        local_point = np.array([point[0], point[1], point[2]], dtype=np.float32)
        rotated = R @ local_point
        translated = rotated + np.array([tx, ty, tz], dtype=np.float32)

        return translated

    @staticmethod
    def rotate_shift_lidar(lidar, pitch_angle=5.0, shift_z_up=2.5):
        """Rotate lidar around Y-axis (pitch) and shift Z up."""
        pitch_angle = np.deg2rad(pitch_angle)
        cos_pitch, sin_pitch = np.cos(pitch_angle), np.sin(pitch_angle)

        R = np.array([
            [cos_pitch, 0, sin_pitch],
            [0,         1, 0        ],
            [-sin_pitch,0, cos_pitch]
        ])

        xyz_rotated = lidar[:, :3] @ R.T
        xyz_rotated[:, 2] += shift_z_up
        lidar[:, :3] = xyz_rotated
        return lidar
    
    @staticmethod
    def undo_rotate_shift_lidar(points, pitch_angle=5.0, shift_z_up=2.5):
        """Inverse of rotate_shift_lidar: undo Z-shift and Y-rotation."""
        pitch_angle = np.deg2rad(pitch_angle)
        cos_pitch, sin_pitch = np.cos(pitch_angle), np.sin(pitch_angle)

        R_inv = np.array([
            [cos_pitch, 0, -sin_pitch],
            [0,         1,  0        ],
            [sin_pitch, 0,  cos_pitch]
        ])

        # Undo shift
        points[:, 2] -= shift_z_up
        # Undo rotation
        points[:, :3] = points[:, :3] @ R_inv.T
        return points

    # Modify this method to publish detected objects localhost:8000 nkgflyfgsb.loclx.io
    def object_detection(self, base_url="http://localhost:8000"):
        if self.lidar_data is None:
            self.get_logger().warn("No LiDAR data yet. Skipping dummy bounding box publish.")
            return
        
        coords = self.lidars_coordinates[self.lidar_name]
        
        #------  TODO: Fardin Object detection -----#
        
        # Preprocess LiDAR for model: rotate + shift
        proc_lidar = self.rotate_shift_lidar(self.lidar_data.copy())
        lidar_bytes = proc_lidar.tobytes()
        lidar_base64 = base64.b64encode(lidar_bytes).decode('utf-8')

        # Prepare request
        payload = {
            "model_name": "pointpillars",
            "score_threshold": 0.65,
            "lidar_data_base64": lidar_base64
        }

        # Send request
        try:
            start_time = time.time()
            response = requests.post(
                f"{base_url}/inference/lidar",
                json=payload,
                headers={'Content-Type': 'application/json'},
                timeout=10
            )
            response.raise_for_status()
            
            result = response.json()
            elapsed = time.time() - start_time

            # --- Create Detected Objects --- 
            detected_objects_msg = DetectedObjects()
            detected_objects_msg.header.frame_id = "map"
            detected_objects_msg.header.stamp = self.get_clock().now().to_msg()

            for i, detection in enumerate(result['detections']):
                self.get_logger().info(f"  Detection {i+1}:")
                self.get_logger().info(f"  Score: {detection['score']:.3f}")
                self.get_logger().info(f"  Label: {detection['label']}")
                self.get_logger().info(f"  Center: [{detection['center'][0]:.2f}, {detection['center'][1]:.2f}, {detection['center'][2]:.2f}]")
                self.get_logger().info(f"  Dimensions: [{detection['dimensions'][0]:.2f}, {detection['dimensions'][1]:.2f}, {detection['dimensions'][2]:.2f}]")
                self.get_logger().info(f"  Rotation: {detection['rotation']:.3f}")

                # --- Create a Detected Object --- 
                object_center = self.undo_rotate_shift_lidar(np.array(detection['center']).reshape(1, 3)).flatten()
                object_orientation = detection['rotation'] # object orientation
                object_dimensions = detection['dimensions'] # object dimensions
                object_label = detection['label']
                existence_probability = 0.7 # existence probability
                classification_probability = detection['score'] # classification probability
            
                detected_obj = self.make_bounding_box(object_center, object_orientation, object_dimensions, object_label, existence_probability, classification_probability, coords)

                # Add to DetectedObjects array
                detected_objects_msg.objects.append(detected_obj)

            # --- Publish Detected Objects ---
            self.detected_objects_pub.publish(detected_objects_msg)
            
            self.get_logger().info(f"✓ LiDAR inference successful in {elapsed:.2f}s")
            self.get_logger().info(f"  Model: {result['model_name']}")
            self.get_logger().info(f"  Detections: {result['num_detections']}")
            self.get_logger().info(f"  Processing time: {result['processing_time']:.2f}s")
        
        except Exception as e:
            self.get_logger().warn(f"✗ LiDAR inference failed: {e}")
            return False
        
        #------  END -----#

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


    def make_bounding_box(self, object_pose, object_yaw, object_dimensions, object_label, existence_probability, classification_probability, coords):
        
        # --- Create a Detected Object --- 
        detected_obj = DetectedObject()
        detected_obj.existence_probability = existence_probability

        # --- Classification ---
        classification = ObjectClassification()

        label_map = { 
            1: ObjectClassification.CAR, 
            2: ObjectClassification.TRUCK, 
            3: ObjectClassification.BUS, 
            4: ObjectClassification.TRAILER, 
            5: ObjectClassification.MOTORCYCLE, 
            6: ObjectClassification.BICYCLE, 
            7: ObjectClassification.PEDESTRIAN, 
        } 
        classification.label = label_map.get(object_label, ObjectClassification.UNKNOWN)
        # classification.label = object_label # ObjectClassification.CAR # TODO: change it with the label comming from the object detection

        classification.probability = classification_probability
        detected_obj.classification.append(classification)

        # ---  Transform to global frame  --- 
        object_global_pose = self.transform_point(object_pose, coords)

        # --- Kinematics (Pose with Covariance) ---
        kinematics = DetectedObjectKinematics()
        pose = Pose()
        pose.position.x = float(object_global_pose[0])
        pose.position.y = float(object_global_pose[1])
        pose.position.z = float(object_global_pose[2]) + 0.0  # Offset up for visualization
        # pose.orientation.w = object_orientation  # if 1 No rotation
        pose.orientation = self.yaw_to_quaternion(object_yaw) # TODO: check if the incomming rotation value is indeed a yaw in radians.

        pose_with_covariance = PoseWithCovariance()
        pose_with_covariance.pose = pose
        kinematics.pose_with_covariance = pose_with_covariance
        kinematics.orientation_availability = DetectedObjectKinematics.AVAILABLE
        kinematics.has_position_covariance = False
        detected_obj.kinematics = kinematics

        # --- Shape ---
        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        # shape.dimensions = Vector3(x=4.0, y=2.0, z=1.5)  # dimensions
        shape.dimensions = Vector3(x=object_dimensions[0], y=object_dimensions[1], z=object_dimensions[2])  # dimensions
        detected_obj.shape = shape

        return detected_obj



    def transform_points(self, points, coordinates, msg):
        
        # LiDAR global position
        tx = coordinates[self.lidar_name]['x']
        ty = coordinates[self.lidar_name]['y']
        tz = coordinates[self.lidar_name]['z']
        yaw_unity_frame = coordinates[self.lidar_name]['yaw']
        yaw_deg = (0 - yaw_unity_frame) % 360
        pitch_deg = coordinates[self.lidar_name]['pitch']
        roll_deg = coordinates[self.lidar_name]['roll']
        self.get_logger().info(f"coordinates: {tx}, {ty}, {tz}, {yaw_deg}, {pitch_deg}, {roll_deg}")


        # Convert to radians
        yaw = math.radians(yaw_deg)
        pitch = math.radians(pitch_deg)
        roll = math.radians(roll_deg)

        # Rotation matrices
        Rz = np.array([
            [math.cos(yaw), -math.sin(yaw), 0],
            [math.sin(yaw),  math.cos(yaw), 0],
            [0, 0, 1]
        ])

        Ry = np.array([
            [math.cos(pitch), 0, math.sin(pitch)],
            [0, 1, 0],
            [-math.sin(pitch), 0, math.cos(pitch)]
        ])

        Rx = np.array([
            [1, 0, 0],
            [0, math.cos(roll), -math.sin(roll)],
            [0, math.sin(roll),  math.cos(roll)]
        ])

        # Combined rotation: Rz * Ry * Rx
        R = Rz @ Ry @ Rx

        # Apply transformation
        rotated_points = points @ R.T
        translated_points = rotated_points + np.array([tx, ty, tz], dtype=np.float32)

        # Publish transformed points
        global_pcd = self.create_pointcloud2(msg.header.frame_id, msg.header.stamp, translated_points)
        self.transformed_lidar_pub.publish(global_pcd)

    @staticmethod
    def create_pointcloud2(frame_id, stamp, points):
        # Create PointCloud2 message from numpy array
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return point_cloud2.create_cloud_xyz32(header, points)


def main(args=None):
    rclpy.init(args=args)
    node = BBoxFlow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
