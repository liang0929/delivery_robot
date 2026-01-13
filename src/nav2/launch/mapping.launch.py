"""
SLAM 建圖 Launch 檔案

使用方式：
1. 啟動此 launch: ros2 launch nav2 mapping.launch.py
2. 另開終端執行鍵盤控制: ros2 run teleop_twist_keyboard teleop_twist_keyboard
3. 另開終端執行 RViz: rviz2
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

    # 馬達控制器配置
    motor_control_dir = get_package_share_directory('motor_control')
    motor_config = os.path.join(motor_control_dir, 'config', 'hs_motor_config.yaml')

    # SLAM Toolbox 配置
    nav2_dir = get_package_share_directory('nav2')
    slam_config = os.path.join(nav2_dir, 'config', 'slam_toolbox_params.yaml')

    # HS 協議馬達控制器
    hs_motor_node = Node(
        package='motor_control',
        executable='hs_motor_controller',
        name='hs_motor_controller',
        output='screen',
        parameters=[motor_config]
    )

    # LiDAR 節點
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/lidar',
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }]
    )

    # IMU 節點
    imu_node = Node(
        package='imu_bno055',
        executable='bno055_i2c_node',
        name='bno055',
        output='screen',
        parameters=[{
            'device': '/dev/i2c-7',
            'address': 40,
            'frame_id': 'imu_link',
        }]
    )

    # EKF 定位融合
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[motor_config]
    )

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

    # 靜態 TF
    base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_footprint', 'base_link']
    )

    base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
    )

    base_link_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        hs_motor_node,
        lidar_node,
        imu_node,
        ekf_node,
        slam_toolbox_node,
        map_relay_node,
        base_footprint_to_base_link,
        base_link_to_laser,
        base_link_to_imu,
    ])
