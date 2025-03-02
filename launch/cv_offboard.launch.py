#!/usr/bin/env python3

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

        # Transform: odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # Offboard Control Node (PX4-native)
        Node(
            package='px4_offboard',
            namespace='px4_2',
            executable='cv_offboard',
            name='cv_offboard',
            output='screen'
        ),
        # cv publisher
        #Node(
        #    package='px4_offboard',
         #   namespace='cv_image_publisher',
         #   executable='cv_image_publisher',
         #   name='cv_image_publisher',
        #),
        # System Stats Node
        Node(
            package='px4_offboard',  
            executable='system_stats_publisher',
            name='system_stats_publisher',
        ),

        # ROS 2 MAVLink Node
        #Node(
         #   package='px4_offboard',  
         #   executable='ros2mavlink',
          #  name='ros2mavlink',
        #),
    ])
