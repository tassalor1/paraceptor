#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_offboard',
            node_namespace='cv_offboard',
            node_executable='visualizer',
            name='visualizer'
        ),
        Node(
            package='px4_offboard',
            node_namespace='px4_2',
            node_executable='cv_offboard',
            name='cv_offboard'
        ),
    ])