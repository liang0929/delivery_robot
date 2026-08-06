"""啟動 mock Pico 集線板節點（不需硬體）"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pico_sensor_hub',
            executable='mock_pico_sensor',
            name='mock_pico_sensor_hub',
            output='screen',
        ),
    ])
