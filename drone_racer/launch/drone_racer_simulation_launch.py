from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(package='drone_racer', executable='drone_control', output='screen'),
        Node(package='drone_racer', executable='gate_finder', output='screen'),
        Node(package='drone_racer', executable='monitor', output='screen'),
    ])
