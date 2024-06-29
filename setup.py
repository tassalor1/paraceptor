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
            ['resource/' + 'visualize.rviz']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('resource/*rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'recon_drone_path = px4_offboard.recon_drone_path:main',
                'inteceptor_path = px4_offboard.inteceptor_path:main',
                'visualizer = px4_offboard.visualizer:main',
                'uav_camera_det = px4_offboard.uav_camera_det:main',
                'base_station = px4_offboard.base_station:main',
                'offboard_base_comm = px4_offboard.offboard_base_comm:main',
                'cv_offboard = px4_offboard.cv_offboard:main',
                'linear_flight = px4_offboard.linear_flight:main',
        ],
    },
)
