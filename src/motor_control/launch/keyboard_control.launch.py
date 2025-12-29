from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 取得參數檔路徑
    config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'hs_motor_config.yaml'
    )

    # 啟動鍵盤控制
    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',  # 讓鍵盤控制有自己的終端
        remappings=[('/cmd_vel', '/cmd_vel')]
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
        teleop_twist_keyboard_node,
        hs_motor_node,
    ])