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
                {'fcu_url': '/dev/ttyACM0:921600'}, 
                {'target_system_id': 1},
                {'target_component_id': 1},
            ],
        ),

        # Transform: map -> odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # Transform: odom -> base_link_frd
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # Offboard Control Node
        Node(
            package='px4_offboard',
            namespace='px4_1',
            executable='cv_offboard',
            name='cv_offboard',
            output='screen'
        ),

        # System Stats Node
        Node(
	    package='px4_offboard',  
	    executable='system_stats_publisher',
	    name='system_stats_publisher',
	),
	# ros 2 mavlink Node
        Node(
	    package='px4_offboard',  
	    executable='ros2mavlink',
	    name='ros2mavlink',
	),

    ])

