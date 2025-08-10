from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os, yaml

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bboxflow_pkg = get_package_share_directory('bboxflow')
    
    # RViZ config
    rviz_config_path = os.path.join(bboxflow_pkg, 'rviz', 'object_detection_vis.rviz')
    
    # object_detection_config_path
    object_detection_config_path = os.path.join(bboxflow_pkg, 'configs', 'bboxflow_configs.yaml')

    with open(object_detection_config_path, "r") as file:
        object_detection_config = yaml.safe_load(file)

    lidars_cfg = object_detection_config['object_detection_input']['lidars']
    cameras_cfg = object_detection_config['object_detection_input']['cameras']

    lidar_nodes = []

    if lidars_cfg['status'] == 'ON':
        lidar_topics = lidars_cfg['topics']  # dict: {name: topic}
        print(f"Found {len(lidar_topics)} LiDAR topics: {list(lidar_topics.values())}")
        
        for lidar_name, lidar_topic in lidar_topics.items():
            lidar_nodes.append(
                Node(
                    package='bboxflow',
                    executable='bbox_flow',
                    name=f'bbox_flow_node_{lidar_name}',
                    output='screen',
                    parameters=[{'lidar_topic': lidar_topic}]
                )
            )

    camera_nodes = []

    if cameras_cfg['status'] == 'ON':
        camera_topics = cameras_cfg['topics']
        print(f"Found {len(camera_topics)} camera topics: {list(camera_topics.values())}")
        
        for camera_name, camera_topic in camera_topics.items():
            camera_nodes.append(
                Node(
                    package='bboxflow',
                    executable='bbox_flow',
                    name=f'bbox_flow_node_{camera_name}',
                    output='screen',
                    parameters=[{'camera_topic': camera_topic}]
                )
            )


    return LaunchDescription(lidar_nodes + [
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen'
        )
    ])
