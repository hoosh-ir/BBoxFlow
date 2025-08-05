from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bboxflow_pkg = get_package_share_directory('bboxflow')
    rviz_config_path = os.path.join(bboxflow_pkg, 'rviz', 'bboxflow_config.rviz')

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'lidar_topic',
        #     default_value='/sim/lidar2',
        #     description='LiDAR topic to subscribe to'
        # ),
        Node(
            package='bboxflow',
            executable='bbox_flow',
            name='bbox_flow_node',
            output='screen'
            # parameters=[{'lidar_topic': LaunchConfiguration('lidar_topic')}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
