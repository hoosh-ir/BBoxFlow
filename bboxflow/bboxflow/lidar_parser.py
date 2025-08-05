#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import MarkerArray, Marker
import random


class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_parser')

        # Declare and get 'lidar_topic' parameter
        self.declare_parameter('lidar_topic', '/sim/lidar2')
        lidar_topic = self.get_parameter('lidar_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            PointCloud2,
            lidar_topic,
            self.lidar_callback,
            10)

        self.marker_pub = self.create_publisher(MarkerArray, 'bbox_markers', 10)

        self.get_logger().info(f'BBoxFlow Node Started... Subscribed to: {lidar_topic}')


    def lidar_callback(self, msg):
        self.get_logger().info(f"Received LiDAR frame with {msg.width} points")
        # === SNAPSHOT POINTCLOUD ===
        point_gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = list(point_gen)

        # === RUN OBJECT DETECTION ===
        # For now, create dummy bounding boxes (you can replace this with your detection code)
        detected_bboxes = self.fake_object_detection(points)

        # === PUBLISH BOUNDING BOXES ===
        marker_array = MarkerArray()
        for i, bbox in enumerate(detected_bboxes):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = msg.header.stamp
            marker.ns = "bbox"
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = bbox['x']
            marker.pose.position.y = bbox['y']
            marker.pose.position.z = bbox['z']
            marker.scale.x = bbox['dx']
            marker.scale.y = bbox['dy']
            marker.scale.z = bbox['dz']
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            marker.lifetime.sec = 0  # Persist until overwritten
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)

    def fake_object_detection(self, points):
        # === Replace this with your real detection code ===
        # For demo, randomly create 2 bounding boxes near first few points
        bboxes = []
        if len(points) >= 10:
            for i in range(2):
                px, py, pz = points[random.randint(0, 9)]
                bbox = {
                    'x': float(px),  # Convert to native Python float
                    'y': float(py),
                    'z': float(pz),
                    'dx': 2.0,
                    'dy': 2.0,
                    'dz': 2.0
                }
                bboxes.append(bbox)
        return bboxes


def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
