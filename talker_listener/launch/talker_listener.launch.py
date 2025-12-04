from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talker_listener',
            executable='talker',
            name='minimal_publisher',
            namespace='demo',
            parameters=[{'message_prefix': 'launched_prefix'}],
            output='screen',
        ),
        Node(
            package='talker_listener',
            executable='listener',
            name='minimal_subscriber',
            namespace='demo',
            output='screen',
        )
    ])
