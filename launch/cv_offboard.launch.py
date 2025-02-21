#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Add argument for sim vs hardware
    use_sim = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='Use simulation if true, hardware if false'
    )

    # Get sim parameter
    sim = LaunchConfiguration('sim')

    # MAVROS configs for sim and hardware
    sim_config = {
        'fcu_url': 'udp://:14540@localhost:14557',
        'gcs_url': '',
    }
    
    hw_config = {
        'fcu_url': '/dev/ttyACM0:921600',
        'gcs_url': '',
    }

    return LaunchDescription([
        use_sim,
        
        # MAVROS node with conditional settings
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros_main',
            output='screen',
            parameters=[
                {'fcu_url': sim_config['fcu_url'] if LaunchConfiguration('sim') else hw_config['fcu_url']},
                {'gcs_url': ''},
                {'system_id': 1},
                {'component_id': 1},
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

        # Offboard Control Node
        Node(
            package='px4_offboard',
            namespace='px4_1',
            executable='cv_offboard',
            name='cv_offboard',
            parameters=[
                {'use_sim_time': sim}
            ],
            output='screen'
        ),

        # # System Stats Node
        # Node(
        #     package='px4_offboard',  
        #     executable='system_stats_publisher',
        #     name='system_stats_publisher',
        # ),

        # # ROS 2 MAVLink Node
        # Node(
        #     package='px4_offboard',  
        #     executable='ros2mavlink',
        #     name='ros2mavlink',
        # ),
    ])