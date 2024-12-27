#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='visual_guidance',
            executable='uav_detection',
            name='uav_detection',
            output='screen'
        ),
        Node(
            package='velocity_controller',
            namespace = 'px4_2',
            executable='simple_controller',
            name='simple_controller',
            output='screen'
        )
        # Node(
        #     package = 'velocity_controller',
        #     executable = 'debugging',
        #     name = 'debugging',
        #     output = 'screen'
        # )
    ])