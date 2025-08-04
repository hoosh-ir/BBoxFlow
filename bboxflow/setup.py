from setuptools import setup

package_name = 'bboxflow'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to install launch files
        ('share/' + package_name + '/launch', ['launch/bboxflow.launch.py']),
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
            'lidar_parser = bboxflow.lidar_parser:main',
        ],
    },
)
