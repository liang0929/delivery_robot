"""
HS 協議馬達控制器啟動文件
使用 AGV-BLD-2S 自定義 HS 協議
"""
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'hs_motor_config.yaml'
    )

    # HS 馬達控制節點
    hs_motor_node = Node(
        package='motor_control',
        executable='hs_motor_controller',
        name='hs_motor_controller',
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
        hs_motor_node,
        robot_localization_node
    ])
