import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from ament_index_python.packages import get_package_share_directory
import open3d as o3d
import numpy as np

class PCDPublisher(Node):
    def __init__(self):
        super().__init__('pcd_map_publisher')
        qos_profile = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE
        )
        self.pub = self.create_publisher(PointCloud2, 'map_points', qos_profile)

        # Publish the map once
        self.publish_map('map.pcd')

    def publish_map(self, file_name):
        bboxflow_pkg_share = get_package_share_directory('bboxflow')
        pcd_map_path = os.path.join(bboxflow_pkg_share, 'data', file_name)

        # Load PCD 
        # pc = pypcd.PointCloud.from_path(pcd_map_path)
        # points = zip(pc.pc_data['x'], pc.pc_data['y'], pc.pc_data['z'])
        pcd = o3d.io.read_point_cloud(pcd_map_path)
        points = np.asarray(pcd.points)

        # Create header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'map'


        # Create PointCloud2 message
        msg = point_cloud2.create_cloud_xyz32(header, list(points))

        # Publish once
        self.pub.publish(msg)
        self.get_logger().info(f"Published {pcd_map_path} once.")


def main(args=None):
    rclpy.init(args=args)
    node = PCDPublisher()
    rclpy.shutdown()
