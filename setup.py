import os
from glob import glob
from setuptools import setup

package_name = 'px4_offboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/ament_index/resource_index/packages',
        ['resource/visualize.rviz']),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    (os.path.join('share', package_name), glob('resource/*rviz'))
],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
                'uav_camera_det = px4_offboard.uav_camera_det:main',
                'offboard_base_comm = px4_offboard.offboard_base_comm:main',
                'cv_offboard = px4_offboard.cv_offboard:main',
                'simple_controller = velocity_controller.simple_controller:main',
                'uav_detection = visual_guidance.uav_detection:main',
                'system_stats_publisher = px4_offboard.system_stats_publisher:main',
        ],
    },
)
