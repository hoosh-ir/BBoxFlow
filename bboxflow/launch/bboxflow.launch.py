from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_topic',
            default_value='/sim/lidar2',
            description='LiDAR topic to subscribe to'
        ),
        Node(
            package='bboxflow',
            executable='lidar_parser',
            name='lidar_parser_node',
            output='screen',
            parameters=[{'lidar_topic': LaunchConfiguration('lidar_topic')}]
        )
    ])
