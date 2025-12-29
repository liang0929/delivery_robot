"""
Modbus 馬達控制器啟動文件
直接通過 RS-232 連接馬達驅動器（無需 ESP32）
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'modbus_motor_config.yaml'
    )

    # Modbus 馬達控制節點
    modbus_motor_node = Node(
        package='motor_control',
        executable='modbus_motor_controller',
        name='modbus_motor_controller',
        output='screen',
        parameters=[config]
    )

    # EKF 定位融合節點
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([
        modbus_motor_node,
        robot_localization_node
    ])
