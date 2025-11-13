from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_api_server',
            executable='api_server',
            name='robot_api_server_node',
            output='screen',
            emulate_tty=True,
        ),
    ])
