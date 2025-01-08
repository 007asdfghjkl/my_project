from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='butler',
            executable='order_manager',
            name='order_manager',
            output='screen'
        ),
        Node(
            package='butler',
            executable='navigation_controller',
            name='navigation_controller',
            output='screen'
        )
    ])
