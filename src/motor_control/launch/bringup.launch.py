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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_tf_config():
    """從配置文件載入 TF 轉換參數"""
    motor_control_dir = get_package_share_directory('motor_control')
    tf_config_path = os.path.join(motor_control_dir, 'config', 'tf_config.yaml')

    with open(tf_config_path, 'r') as f:
        config = yaml.safe_load(f)

    return config.get('tf_transforms', {})


def create_static_tf_node(name: str, tf_config: dict) -> Node:
    """根據配置創建靜態 TF 節點"""
    t = tf_config.get('translation', {})
    r = tf_config.get('rotation', {})

    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            str(t.get('x', 0.0)),
            str(t.get('y', 0.0)),
            str(t.get('z', 0.0)),
            str(r.get('roll', 0.0)),
            str(r.get('pitch', 0.0)),
            str(r.get('yaw', 0.0)),
            tf_config.get('parent_frame', 'base_link'),
            tf_config.get('child_frame', 'child')
        ]
    )


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

    # ========== 靜態 TF (從配置文件載入) ==========
    tf_config = load_tf_config()

    base_footprint_to_base_link = create_static_tf_node(
        'base_footprint_to_base_link',
        tf_config.get('base_footprint_to_base_link', {})
    )

    base_link_to_laser = create_static_tf_node(
        'base_link_to_laser',
        tf_config.get('base_link_to_laser', {})
    )

    base_link_to_imu = create_static_tf_node(
        'base_link_to_imu',
        tf_config.get('base_link_to_imu', {})
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

    # rosapi (暫時停用 - 在 Jetson 上不穩定)
    # rosapi_node = Node(
    #     package='rosapi',
    #     executable='rosapi_node',
    #     name='rosapi',
    #     output='screen',
    #     parameters=[{
    #         'call_services_in_new_thread': True,
    #     }],
    #     condition=IfCondition(enable_web)
    # )

    # API Server (控制建圖/導航模式)
    api_server = ExecuteProcess(
        cmd=['ros2', 'run', 'robot_api_server', 'api_server'],
        output='screen',
        condition=IfCondition(enable_web)
    )

    # ========== 啟動順序說明 ==========
    # 1. 靜態 TF (立即啟動) - 不依賴其他節點
    # 2. 感測器節點 (立即啟動) - motor, lidar, imu
    # 3. EKF (延遲 2 秒) - 需要等待 /odom_raw 和 /imu/data 準備好
    # 4. Web 服務 (延遲 3 秒) - 需要等待核心節點準備好

    # EKF 延遲啟動（等待 odom_raw 和 imu/data 準備好）
    delayed_ekf = TimerAction(
        period=2.0,
        actions=[ekf_node]
    )

    # Web 服務延遲啟動（等待核心節點準備好）
    # 注意：rosbridge_node 和 api_server 已經有 condition=IfCondition(enable_web)
    delayed_web_services = TimerAction(
        period=3.0,
        actions=[
            rosbridge_node,
            # rosapi_node,  # 暫時停用
            api_server,
        ]
    )

    # ========== 組合 ==========
    return LaunchDescription([
        # 參數宣告
        DeclareLaunchArgument('enable_web', default_value='true',
                             description='啟用 Web 服務 (rosbridge + API)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='使用模擬時間'),

        # 靜態 TF (立即啟動)
        base_footprint_to_base_link,
        base_link_to_laser,
        base_link_to_imu,

        # 感測器節點 (立即啟動)
        motor_node,
        lidar_node,
        imu_node,

        # EKF (延遲 2 秒)
        delayed_ekf,

        # Web 服務 (延遲 3 秒)
        delayed_web_services,
    ])
