"""啟動 mock Pico 集線板節點（不需硬體）與 battery_state adapter。

adapter 與真實 launch 完全相同的參數檔與節點名——mock 的意義就是讓下游
看到一模一樣的介面，adapter 只跟著 `/pico/*` 走，不在乎資料是誰發的。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pico_sensor_hub'),
        'config', 'pico_sensor_hub.yaml')

    return LaunchDescription([
        Node(
            package='pico_sensor_hub',
            executable='mock_pico_sensor',
            name='mock_pico_sensor_hub',
            output='screen',
        ),
        Node(
            package='pico_sensor_hub',
            executable='battery_state_node',
            name='pico_battery_state',
            output='screen',
            parameters=[config],
        ),
    ])
