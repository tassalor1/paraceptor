from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'typhoon_h480', '-file', '/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/typhoon_h480'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='gzserver',
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='gzclient',
            output='screen'
        )
    ])
