"""
SLAM 建圖 Launch 檔案

使用方式：
1. 確保 bringup.launch.py 已啟動（robot-core.service）
2. 啟動此 launch: ros2 launch nav2 mapping.launch.py
3. 另開終端執行鍵盤控制: ros2 run teleop_twist_keyboard teleop_twist_keyboard
4. 另開終端執行 RViz: rviz2

注意：以下節點已被 bringup.launch.py 啟動，這裡不再重複啟動：
- hs_motor_controller
- sllidar_node
- bno055 (IMU)
- ekf_filter_node
- 靜態 TF (base_footprint_to_base_link, base_link_to_laser, base_link_to_imu)
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # SLAM Toolbox 配置
    nav2_dir = get_package_share_directory('nav2')
    slam_config = os.path.join(nav2_dir, 'config', 'slam_toolbox_params.yaml')

    # SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_config],
        output='screen'
    )

    # Map Relay (解決 slam_toolbox 與 rosbridge QoS 不相容問題)
    map_relay_node = Node(
        package='motor_control',
        executable='map_relay',
        name='map_relay',
        output='screen'
    )

    # 注意：以下節點已被 bringup.launch.py 啟動，不再重複
    # - hs_motor_controller
    # - sllidar_node (lidar)
    # - bno055 (imu)
    # - ekf_filter_node
    # - base_footprint_to_base_link
    # - base_link_to_laser
    # - base_link_to_imu

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        slam_toolbox_node,
        map_relay_node,
    ])
