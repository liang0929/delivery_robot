"""
機器人統一啟動 Launch 檔案

使用方式：
  # 基本模式（核心節點）
  ros2 launch motor_control bringup.launch.py

  # 包含 Web 服務（推薦）
  ros2 launch motor_control bringup.launch.py enable_web:=true

  # 模擬模式（不需要實際硬體）
  ros2 launch motor_control bringup.launch.py simulation:=true

  # 模擬模式 + Web 服務
  ros2 launch motor_control bringup.launch.py simulation:=true enable_web:=true

啟動後，可透過 Web 前端選擇：
  - 遙控模式（預設）
  - 建圖模式（Start Mapping）
  - 導航模式（Start Navigation）
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_tf_config():
    """從配置文件載入 TF 轉換參數"""
    motor_control_dir = get_package_share_directory('motor_control')
    tf_config_path = os.path.join(motor_control_dir, 'config', 'tf_config.yaml')

    # 默認 TF 配置（當配置文件不存在時使用）
    default_config = {
        'base_footprint_to_base_link': {
            'parent_frame': 'base_footprint',
            'child_frame': 'base_link',
            'translation': {'x': 0.0, 'y': 0.0, 'z': 0.05},
            'rotation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        },
        'base_link_to_laser': {
            'parent_frame': 'base_link',
            'child_frame': 'laser',
            'translation': {'x': 0.1, 'y': 0.0, 'z': 0.15},
            'rotation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        },
        'base_link_to_imu': {
            'parent_frame': 'base_link',
            'child_frame': 'imu_link',
            'translation': {'x': 0.0, 'y': 0.0, 'z': 0.05},
            'rotation': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        }
    }

    try:
        with open(tf_config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config.get('tf_transforms', default_config)
    except FileNotFoundError:
        print(f'[WARN] TF 配置文件不存在: {tf_config_path}，使用默認配置')
        return default_config
    except yaml.YAMLError as e:
        print(f'[WARN] TF 配置文件解析失敗: {e}，使用默認配置')
        return default_config


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
    simulation = LaunchConfiguration('simulation', default='false')
    sim_scene = LaunchConfiguration('sim_scene', default='room')

    # ========== 套件路徑 ==========
    motor_control_dir = get_package_share_directory('motor_control')
    motor_config = os.path.join(motor_control_dir, 'config', 'hs_motor_config.yaml')

    # ========== 真實硬體節點 ==========

    # HS 協議馬達控制器 (真實硬體)
    motor_node = Node(
        package='motor_control',
        executable='hs_motor_controller',
        name='hs_motor_controller',
        output='screen',
        parameters=[motor_config],
        condition=UnlessCondition(simulation)
    )

    # LiDAR 節點 (真實硬體)
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
        }],
        condition=UnlessCondition(simulation)
    )

    # IMU 節點 (真實硬體)
    imu_node = Node(
        package='imu_bno055',
        executable='bno055_i2c_node',
        name='bno055',
        output='screen',
        parameters=[{
            'device': '/dev/i2c-7',
            'address': 40,
            'frame_id': 'imu_link',
        }],
        condition=UnlessCondition(simulation)
    )

    # ========== 模擬節點 ==========

    # Mock 馬達控制器 (模擬)
    mock_motor_node = Node(
        package='motor_control',
        executable='mock_motor_controller',
        name='mock_motor_controller',
        output='screen',
        parameters=[{
            'wheel_separation': 0.27,
            'wheel_radius': 0.065,
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0,
            'odom_frequency': 50.0,
        }],
        condition=IfCondition(simulation)
    )

    # Mock LiDAR (模擬)
    mock_lidar_node = Node(
        package='motor_control',
        executable='mock_lidar',
        name='mock_lidar',
        output='screen',
        parameters=[{
            'frame_id': 'laser',
            'scan_frequency': 10.0,
            'range_min': 0.15,
            'range_max': 12.0,
            'scene': sim_scene,
        }],
        condition=IfCondition(simulation)
    )

    # Mock IMU (模擬)
    mock_imu_node = Node(
        package='motor_control',
        executable='mock_imu',
        name='mock_imu',
        output='screen',
        parameters=[{
            'frame_id': 'imu_link',
            'publish_frequency': 100.0,
        }],
        condition=IfCondition(simulation)
    )

    # ========== EKF 定位融合 ==========
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

    # API Server (控制建圖/導航模式)
    api_server = ExecuteProcess(
        cmd=['ros2', 'run', 'robot_api_server', 'api_server'],
        output='screen',
        condition=IfCondition(enable_web)
    )

    # ========== 啟動順序說明 ==========
    # 1. 靜態 TF (立即啟動) - 不依賴其他節點
    # 2. 感測器節點 (立即啟動) - motor, lidar, imu (真實或模擬)
    # 3. EKF (延遲 2 秒) - 需要等待 /odom_raw 和 /imu/data 準備好
    # 4. Web 服務 (延遲 3 秒) - 需要等待核心節點準備好

    # EKF 延遲啟動（等待 odom_raw 和 imu/data 準備好）
    delayed_ekf = TimerAction(
        period=2.0,
        actions=[ekf_node]
    )

    # Web 服務延遲啟動（等待核心節點準備好）
    delayed_web_services = TimerAction(
        period=3.0,
        actions=[
            rosbridge_node,
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
        DeclareLaunchArgument('simulation', default_value='false',
                             description='啟用模擬模式 (不需要實際硬體)'),
        DeclareLaunchArgument('sim_scene', default_value='room',
                             description='模擬場景: empty, room, corridor'),

        # 模式提示
        LogInfo(
            condition=IfCondition(simulation),
            msg='=== 模擬模式啟動 (Simulation Mode) ==='
        ),
        LogInfo(
            condition=UnlessCondition(simulation),
            msg='=== 真實硬體模式啟動 (Hardware Mode) ==='
        ),

        # 靜態 TF (立即啟動)
        base_footprint_to_base_link,
        base_link_to_laser,
        base_link_to_imu,

        # 真實硬體節點 (當 simulation:=false)
        motor_node,
        lidar_node,
        imu_node,

        # 模擬節點 (當 simulation:=true)
        mock_motor_node,
        mock_lidar_node,
        mock_imu_node,

        # EKF (延遲 2 秒)
        delayed_ekf,

        # Web 服務 (延遲 3 秒)
        delayed_web_services,
    ])
