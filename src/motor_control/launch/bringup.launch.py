"""
機器人統一啟動 Launch 檔案

使用方式：
  # 基本模式（核心節點）
  ros2 launch motor_control bringup.launch.py

  # 包含 Web 服務（推薦）
  ros2 launch motor_control bringup.launch.py enable_web:=true

啟動後，可透過 Web 前端選擇：
  - 遙控模式（預設）
  - 建圖模式（Start Mapping）
  - 導航模式（Start Navigation）
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # ========== Launch 參數 ==========
    enable_web = LaunchConfiguration('enable_web', default='true')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ========== 套件路徑 ==========
    motor_control_dir = get_package_share_directory('motor_control')
    motor_config = os.path.join(motor_control_dir, 'config', 'hs_motor_config.yaml')

    # ========== 核心節點 ==========

    # HS 協議馬達控制器
    motor_node = Node(
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

    # ========== 靜態 TF ==========
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

    # ========== Web 服務 ==========
    # rosbridge WebSocket
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'call_services_in_new_thread': True,
            'send_action_goals_in_new_thread': True,
            'default_call_service_timeout': 10.0,
            'max_message_size': 10000000,
            'unregister_timeout': 10.0,
        }],
        condition=IfCondition(enable_web)
    )

    # rosapi
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
        parameters=[{
            'call_services_in_new_thread': True,
        }],
        condition=IfCondition(enable_web)
    )

    # API Server (控制建圖/導航模式)
    api_server = ExecuteProcess(
        cmd=['ros2', 'run', 'robot_api_server', 'api_server'],
        output='screen',
        condition=IfCondition(enable_web)
    )

    # ========== 組合 ==========
    return LaunchDescription([
        # 參數宣告
        DeclareLaunchArgument('enable_web', default_value='true',
                             description='啟用 Web 服務 (rosbridge + API)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='使用模擬時間'),

        # 核心節點
        motor_node,
        lidar_node,
        imu_node,
        ekf_node,

        # 靜態 TF
        base_footprint_to_base_link,
        base_link_to_laser,
        base_link_to_imu,

        # Web 服務
        rosbridge_node,
        rosapi_node,
        api_server,
    ])
