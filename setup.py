import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, "yolov5"]),  
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/ament_index/resource_index/packages',
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
                'visualizer = px4_offboard.visualizer:main',
                'uav_camera_det = px4_offboard.uav_camera_det:main',
                'cv_offboard = px4_offboard.cv_offboard:main',
                'cv_image_publisher = px4_offboard.uav_track_and_detect:main',
                'system_stats_publisher = px4_offboard.system_stats_publisher:main',
                'ros2mavlink = px4_offboard.ros2mavlink:main',
        ],
    },
)
