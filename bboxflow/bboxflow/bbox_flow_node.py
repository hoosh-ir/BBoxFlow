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


class BBoxFlow(Node):
    def __init__(self):
        super().__init__('bbox_flow')

        # Declare and get 'lidar_topic' parameter

        ## Uncomment if you want to declare topic from launch 
        # self.declare_parameter('lidar_topic', '/sim/lidar2')
        # lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value

        ## Insert the topic you want to subscribe in config file
        self.configs = self.config_file_loader('bboxflow_configs.yaml')
        self.lidar_topic = self.configs['object_detection_input']['lidar_topic']['name']

        self.subscription = self.create_subscription(
            PointCloud2,
            self.lidar_topic,
            self.lidar_callback,
            10)

        self.bbox_topic = self.lidar_topic + '/detected_objects'
        self.marker_pub = self.create_publisher(MarkerArray, self.bbox_topic, 10)
        self.lidar_data = np.array

        self.get_logger().info(f'BBoxFlow Node Started... Subscribed to: {self.lidar_topic}')

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



def main(args=None):
    rclpy.init(args=args)
    node = BBoxFlow()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
