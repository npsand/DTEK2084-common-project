from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        
        DeclareLaunchArgument('test_arg', default_value="a", description='Test argument'),

        Node(package='drone_racer', executable='drone_control', output='screen'),
        Node(package='drone_racer', executable='gate_finder', output='screen', parameters=[{'test_arg': LaunchConfiguration('test_arg')}]),
        Node(package='drone_racer', executable='monitor', output='screen')
    ])

