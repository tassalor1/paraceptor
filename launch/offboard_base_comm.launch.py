#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    return LaunchDescription([
        Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='px4_offboard',
            namespace='px4_2',
            executable='offboard_base_comm',
            name='inteceptor'
        ),
        Node(
            package='px4_offboard',
            namespace='px4_1',
            executable='recon_drone_path',
            name='recon'
        ),
        Node(
            package = 'px4_offboard',
            namespace='base_station',
            executable= 'base_station',
            name = 'base_station'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(package_dir, 'visualize.rviz')]]
        )
    ])