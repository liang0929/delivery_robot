"""
RViz2 機器人視覺化 Launch 檔案

使用方式：
  ros2 launch robot_description view_robot.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('robot_description')
    rviz_config = os.path.join(pkg_dir, 'config', 'robot.rviz')

    return LaunchDescription([
        # 啟動 robot_description（帶 joint_state_publisher）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_dir, 'launch', 'robot_description.launch.py')
            ),
            launch_arguments={'use_joint_state_publisher': 'true'}.items()
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
        ),
    ])
