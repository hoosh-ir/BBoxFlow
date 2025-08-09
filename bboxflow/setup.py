from setuptools import setup
import os
from glob import glob

package_name = 'bboxflow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/configs', ['configs/bboxflow_configs.yaml']),
        ('share/' + package_name + '/configs', ['configs/rsu_coordinates.yaml']),
        ('share/' + package_name + '/launch', ['launch/bboxflow.launch.py']),
        ('share/' + package_name + '/rviz', ['rviz/bboxflow_config.rviz']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hamidreza',
    maintainer_email='your_email@example.com',
    description='BBoxFlow - LiDAR Object Detection Pipeline',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bbox_flow = bboxflow.bbox_flow_node:main',
        ],
    },
)
