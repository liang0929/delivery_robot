from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('motor_control'),
        'config',
        'motor_config.yaml'
    )

    esp32_motor_node = Node(
        package='motor_control',
        executable='uart_send_to_esp32',
        name='esp32_motor_controller',
        output='screen',
        parameters=[config],  
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
        esp32_motor_node, 
        robot_localization_node
    ])