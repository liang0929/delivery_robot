"""啟動 pico_sensor_hub 節點（真實硬體）與 battery_state adapter。

adapter 預設一起啟動：它只是把同一份 `/pico/*` 資料換成標準
`sensor_msgs/BatteryState`，成本是 2 Hz 一則訊息，而 opennav_docking 少了它
就完全判不出充電中。要單獨關掉的話直接 `ros2 run` 起真節點即可。
"""

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
        Node(
            package='pico_sensor_hub',
            executable='battery_state_node',
            name='pico_battery_state',
            output='screen',
            parameters=[config],
        ),
    ])
