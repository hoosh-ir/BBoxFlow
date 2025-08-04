#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_parser')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/sim/lidar2',  # Change topic if needed
            self.lidar_callback,
            10)
        self.get_logger().info('Lidar Subscriber Node Started...')

    def lidar_callback(self, msg):
        self.get_logger().info(f"Received LiDAR frame with {msg.width} points")
        # Process point cloud snapshot here (later for detection)
        point_gen = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        print("type: ", type(point_gen))
        print("size: ", len(point_gen))
        count = 0
        for point in point_gen:
            x, y, z = point
            print(f"Point {count}: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            count += 1
            if count >= 5:  # Just print first 5 points per message
                break

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
