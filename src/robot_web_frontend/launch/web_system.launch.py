"""
Web System Launch File
Launches rosbridge_websocket for web frontend communication
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # rosbridge WebSocket server
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
        }]
    )

    return LaunchDescription([
        rosbridge_node,
    ])
