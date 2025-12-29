from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    鍵盤控制 Launch 檔案

    使用方式：
    1. 啟動此 launch: ros2 launch motor_control keyboard_control.launch.py
    2. 另開終端執行: ros2 run teleop_twist_keyboard teleop_twist_keyboard
    """
    # 取得參數檔路徑
    config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'hs_motor_config.yaml'
    )

    # 啟動 HS 協議馬達控制器（直連 RS-232）
    hs_motor_node = Node(
        package='motor_control',
        executable='hs_motor_controller',
        name='hs_motor_controller',
        output='screen',
        parameters=[config],
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    return LaunchDescription([
        hs_motor_node,
    ])