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
from geometry_msgs.msg import Pose, Vector3, PoseWithCovariance
import math
from std_msgs.msg import Header


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

        self.lidar_data = np.array

        
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
        
        # points_np = np.array(points_list, dtype=np.float32)
        # === Transform to global frame ===
        # self.transform_points(points_np, self.lidars_coordinates, msg)

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
        self.lidar_data = np.concatenate([self.lidar_data, np.zeros((self.lidar_data.shape[0], 1))+0], axis=1)

        self.get_logger().info(f"self.lidar_data shape: {self.lidar_data.shape}, dtype: {self.lidar_data.dtype}")
        self.detect_objects()


    def transform_point(self, point, coordinates):
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


    # Modify this method to publish detected objects
    def detect_objects(self):
        if self.lidar_data is None:
            self.get_logger().warn("No LiDAR data yet. Skipping dummy bounding box publish.")
            return

        # Randomly pick one point from LiDAR data
        random_idx = random.randint(0, len(self.lidar_data) - 1)
        random_point = self.lidar_data[random_idx]

        detected_objects_msg = DetectedObjects()
        detected_objects_msg.header.frame_id = "map"
        detected_objects_msg.header.stamp = self.get_clock().now().to_msg()

        # --- Create a Dummy Detected Object --- 
        detected_obj = DetectedObject()
        detected_obj.existence_probability = 0.95

        # --- Classification ---
        classification = ObjectClassification()
        classification.label = ObjectClassification.CAR
        classification.probability = 0.9
        detected_obj.classification.append(classification)

        # ---  Transform to global frame  --- 
        lidar_name = 'lidar' + self.lidar_topic.split('/')[3].split('_')[2]
        # lidar_name = self.lidar_topic.split('/sim/')[1]
        coords = self.lidars_coordinates[lidar_name]
        global_point = self.transform_point(random_point, coords)

        # --- Kinematics (Pose with Covariance) ---
        kinematics = DetectedObjectKinematics()
        pose = Pose()
        pose.position.x = float(global_point[0])
        pose.position.y = float(global_point[1])
        pose.position.z = float(global_point[2]) + 1.0  # Offset up for visualization
        pose.orientation.w = 1.0  # No rotation
        pose_with_covariance = PoseWithCovariance()
        pose_with_covariance.pose = pose
        kinematics.pose_with_covariance = pose_with_covariance
        kinematics.orientation_availability = DetectedObjectKinematics.AVAILABLE
        kinematics.has_position_covariance = False
        detected_obj.kinematics = kinematics

        # --- Shape ---
        shape = Shape()
        shape.type = Shape.BOUNDING_BOX
        shape.dimensions = Vector3(x=4.0, y=2.0, z=1.5)  # Dummy dimensions
        detected_obj.shape = shape

        # Add to DetectedObjects array
        detected_objects_msg.objects.append(detected_obj)

        # === Publish ===
        self.detected_objects_pub.publish(detected_objects_msg)
        self.get_logger().info(f"Published dummy DetectedObject at ({pose.position.x:.2f}, {pose.position.y:.2f}, {pose.position.z:.2f}). on {self.bbox_topic}")



    def transform_points(self, points, coordinates, msg):
        
        # LiDAR global position
        lidar_name = 'lidar' + self.lidar_topic.split('/')[3].split('_')[2]
        # lidar_name = self.lidar_topic.split('/sim/')[1]

        tx = coordinates[lidar_name]['x']
        ty = coordinates[lidar_name]['y']
        tz = coordinates[lidar_name]['z']
        yaw_unity_frame = coordinates[lidar_name]['yaw']
        yaw_deg = (0 - yaw_unity_frame) % 360
        pitch_deg = coordinates[lidar_name]['pitch']
        roll_deg = coordinates[lidar_name]['roll']
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

    def create_pointcloud2(self, frame_id, stamp, points):
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
