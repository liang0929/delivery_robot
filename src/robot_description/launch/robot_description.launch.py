"""
機器人描述 Launch 檔案

啟動 robot_state_publisher 來發布 URDF 和 TF

使用方式：
  ros2 launch robot_description robot_description.launch.py

  # 發布 joint_states（用於 RViz2 顯示輪子轉動）
  ros2 launch robot_description robot_description.launch.py use_joint_state_publisher:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 套件路徑
    pkg_dir = get_package_share_directory('robot_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'robot.urdf.xacro')

    # Launch 參數
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher')

    # 使用 xacro 處理 URDF
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        # 參數宣告
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='使用模擬時間'
        ),
        DeclareLaunchArgument(
            'use_joint_state_publisher',
            default_value='false',
            description='啟動 joint_state_publisher（用於 RViz2 顯示）'
        ),

        # Robot State Publisher
        # 發布 robot_description 和靜態 TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
                'publish_frequency': 50.0,
            }]
        ),

        # Joint State Publisher (可選)
        # 發布 joint_states 讓輪子在 RViz2 中顯示
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_joint_state_publisher)
        ),
    ])
