#!/usr/bin/env python

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # MAVROS Node for USB connection
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            parameters=[
                {'fcu_url': '/dev/ttyACM0:57600'},  # USB connection to Pixracer
                {'gcs_url': ''},                   # Ground control station, leave empty for none
                {'target_system_id': 1},           # System ID for Pixracer
                {'target_component_id': 1},        # Component ID for Pixracer
            ]
        ),
        # Visualizer Node
        #Node(
         #   package='px4_offboard',
         #   namespace='cv_offboard',
         #   executable='visualizer',
         #   name='visualizer'
        #),
        # Offboard Control Node
        Node(
            package='px4_offboard',
            namespace='px4_2',
            executable='cv_offboard',
            name='cv_offboard'
        ),
    ])

