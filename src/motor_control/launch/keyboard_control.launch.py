from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 取得參數檔路徑
    config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'motor_config.yaml'
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

    # 啟動基礎控制器（與ESP32通訊），改用 YAML 參數
    base_control_node = Node(
        package='motor_control',
        executable='uart_send_to_esp32',
        name='esp32_motor_controller',
        output='screen',
        parameters=[config],  # 直接讀取 YAML
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    robot_localization_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config]
    )

    return LaunchDescription([
        teleop_twist_keyboard_node,
        base_control_node,
        robot_localization_node
    ])