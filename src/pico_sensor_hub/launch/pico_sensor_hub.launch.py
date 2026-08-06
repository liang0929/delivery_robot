"""啟動 pico_sensor_hub 節點（真實硬體）"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pico_sensor_hub'),
        'config', 'pico_sensor_hub.yaml')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/pico_sensor_hub',
        description='Pico 序列埠（udev 固定節點；未裝 rule 時用 /dev/ttyACM0）')

    return LaunchDescription([
        port_arg,
        Node(
            package='pico_sensor_hub',
            executable='pico_sensor_node',
            name='pico_sensor_hub',
            output='screen',
            parameters=[config, {'port': LaunchConfiguration('port')}],
        ),
    ])
