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


class BBoxFlow(Node):
    def __init__(self):
        super().__init__('bbox_flow')

        # Uncomment if you want to get the topics name form launch file
        self.declare_parameter('lidar_topic', '/sim/lidar2')
        self.lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value
        self.get_logger().info(f"Using LiDAR topic: {self.lidar_topic}")

        ## Uncomment if you want to get the topics from config files
        # self.configs = self.config_file_loader('bboxflow_configs.yaml')
        # self.lidar_topic = self.configs['object_detection_input']['lidars']['topics']

        self.subscription = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            10)
        self.get_logger().info(f'BBoxFlow Node Started... Subscribed to: {self.lidar_topic}')
        
        self.bbox_topic = self.lidar_topic + '/detected_objects'
        self.detected_objects_pub = self.create_publisher(DetectedObjects, self.bbox_topic, 10)
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
        point_gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)

        # Convert to list first:
        points_list = list(point_gen)  # This is a list of numpy structured elements

        # Extract columns explicitly:
        x_vals = np.array([p['x'] for p in points_list], dtype=np.float32)
        y_vals = np.array([p['y'] for p in points_list], dtype=np.float32)
        z_vals = np.array([p['z'] for p in points_list], dtype=np.float32)

        # Stack into (N, 4) array:
        self.lidar_data = np.column_stack((x_vals, y_vals, z_vals))
        self.lidar_data = np.concatenate([self.lidar_data, np.zeros((self.lidar_data.shape[0], 1))+0], axis=1)

        self.get_logger().info(f"self.lidar_data shape: {self.lidar_data.shape}, dtype: {self.lidar_data.dtype}")
        self.detect_objects()

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

        # === Create a Dummy Detected Object ===
        detected_obj = DetectedObject()
        detected_obj.existence_probability = 0.95

        # --- Classification ---
        classification = ObjectClassification()
        classification.label = ObjectClassification.CAR
        classification.probability = 0.9
        detected_obj.classification.append(classification)

        # --- Kinematics (Pose with Covariance) ---
        kinematics = DetectedObjectKinematics()
        pose = Pose()
        pose.position.x = float(random_point[0])
        pose.position.y = float(random_point[1])
        pose.position.z = float(random_point[2]) + 1.0  # Offset up for visualization
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


def main(args=None):
    rclpy.init(args=args)
    node = BBoxFlow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
