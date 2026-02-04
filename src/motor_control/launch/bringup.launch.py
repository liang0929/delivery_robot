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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


# ========== Jetson Orin NX CPU 親和性配置 ==========
# 8 核心分配策略：
#   核心 0-1: 馬達控制（實時性最高）
#   核心 2-3: LiDAR/IMU 感測器處理
#   核心 4-5: EKF/AMCL 定位
#   核心 6-7: Web 服務/API（優先級最低）
CPU_AFFINITY = {
    'motor': '0-1',
    'sensor': '2-3',
    'localization': '4-5',
    'web': '6-7',
}


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

    # static_transform_publisher 參數順序: x y z yaw pitch roll frame_id child_frame_id
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(t.get('x', 0.0)),
            '--y', str(t.get('y', 0.0)),
            '--z', str(t.get('z', 0.0)),
            '--roll', str(r.get('roll', 0.0)),
            '--pitch', str(r.get('pitch', 0.0)),
            '--yaw', str(r.get('yaw', 0.0)),
            '--frame-id', tf_config.get('parent_frame', 'base_link'),
            '--child-frame-id', tf_config.get('child_frame', 'child')
        ]
    )


def get_cpu_prefix(affinity_type: str, enabled: bool) -> str:
    """獲取 CPU 親和性 prefix（用於 Node 的 prefix 參數）"""
    if enabled and affinity_type in CPU_AFFINITY:
        return f'taskset -c {CPU_AFFINITY[affinity_type]}'
    return ''


def get_cpu_prefix_list(affinity_type: str, enabled: bool) -> list:
    """獲取 CPU 親和性 prefix 列表（用於 ExecuteProcess 的 cmd 參數）"""
    if enabled and affinity_type in CPU_AFFINITY:
        return ['taskset', '-c', CPU_AFFINITY[affinity_type]]
    return []


def launch_setup(context, *args, **kwargs):
    """動態生成啟動配置（支持運行時參數解析）"""
    # 解析運行時參數
    cpu_affinity_enabled = LaunchConfiguration('cpu_affinity').perform(context).lower() == 'true'
    enable_web = LaunchConfiguration('enable_web').perform(context).lower() == 'true'
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'true'
    simulation = LaunchConfiguration('simulation').perform(context).lower() == 'true'
    sim_scene = LaunchConfiguration('sim_scene').perform(context)
    lidar_port = LaunchConfiguration('lidar_port').perform(context)
    imu_device = LaunchConfiguration('imu_device').perform(context)

    # 套件路徑
    motor_control_dir = get_package_share_directory('motor_control')
    motor_config = os.path.join(motor_control_dir, 'config', 'hs_motor_config.yaml')

    # URDF 路徑
    try:
        robot_description_dir = get_package_share_directory('robot_description')
        xacro_file = os.path.join(robot_description_dir, 'urdf', 'robot.urdf.xacro')
        use_urdf = os.path.exists(xacro_file)
    except Exception:
        use_urdf = False
        xacro_file = None

    # TF 配置（僅在沒有 URDF 時使用）
    tf_config = load_tf_config()

    nodes = []

    # ========== 模式提示 ==========
    if simulation:
        nodes.append(LogInfo(msg='=== 模擬模式啟動 (Simulation Mode) ==='))
    else:
        nodes.append(LogInfo(msg='=== 真實硬體模式啟動 (Hardware Mode) ==='))

    if cpu_affinity_enabled:
        nodes.append(LogInfo(msg='=== CPU 親和性已啟用 (Jetson Orin NX 優化) ==='))

    # ========== 機器人描述 (URDF) 或靜態 TF ==========
    if use_urdf:
        # 使用 robot_state_publisher 發布 URDF 和 TF
        nodes.append(LogInfo(msg='=== 使用 URDF 機器人模型 ==='))
        robot_description = Command(['xacro ', xacro_file])
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
                'publish_frequency': 50.0,
            }]
        ))
    else:
        # 回退：使用靜態 TF（當 robot_description 套件不存在時）
        nodes.append(LogInfo(msg='=== 使用靜態 TF（無 URDF）==='))
        nodes.append(create_static_tf_node(
            'base_footprint_to_base_link',
            tf_config.get('base_footprint_to_base_link', {})
        ))
        nodes.append(create_static_tf_node(
            'base_link_to_laser',
            tf_config.get('base_link_to_laser', {})
        ))
        nodes.append(create_static_tf_node(
            'base_link_to_imu',
            tf_config.get('base_link_to_imu', {})
        ))

    # ========== 真實硬體節點 ==========
    if not simulation:
        # HS 協議馬達控制器 (核心 0-1)
        nodes.append(Node(
            package='motor_control',
            executable='hs_motor_controller',
            name='hs_motor_controller',
            output='screen',
            parameters=[motor_config, {'use_sim_time': use_sim_time}],
            prefix=get_cpu_prefix('motor', cpu_affinity_enabled)
        ))

        # LiDAR 節點 (核心 2-3)
        nodes.append(Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 256000,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            prefix=get_cpu_prefix('sensor', cpu_affinity_enabled)
        ))

        # IMU 節點 (核心 2-3, 50Hz - EKF 只需 30Hz)
        nodes.append(Node(
            package='imu_bno055',
            executable='bno055_i2c_node',
            name='bno055',
            namespace='imu',
            output='screen',
            parameters=[{
                'device': imu_device,
                'address': 40,
                'frame_id': 'imu_link',
                'rate': 50.0,
            }],
            prefix=get_cpu_prefix('sensor', cpu_affinity_enabled)
        ))

    # ========== 模擬節點 ==========
    if simulation:
        # Mock 馬達控制器 (核心 0-1)
        nodes.append(Node(
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
                'use_sim_time': use_sim_time,
            }],
            prefix=get_cpu_prefix('motor', cpu_affinity_enabled)
        ))

        # Mock LiDAR (核心 2-3)
        nodes.append(Node(
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
            prefix=get_cpu_prefix('sensor', cpu_affinity_enabled)
        ))

        # Mock IMU (核心 2-3, 50Hz - EKF 只需 30Hz)
        nodes.append(Node(
            package='motor_control',
            executable='mock_imu',
            name='mock_imu',
            output='screen',
            parameters=[{
                'frame_id': 'imu_link',
                'publish_frequency': 50.0,
            }],
            prefix=get_cpu_prefix('sensor', cpu_affinity_enabled)
        ))

    # ========== EKF 定位融合 (延遲 2 秒，核心 4-5) ==========
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[motor_config, {'use_sim_time': use_sim_time}],
        prefix=get_cpu_prefix('localization', cpu_affinity_enabled)
    )
    nodes.append(TimerAction(period=2.0, actions=[ekf_node]))

    # ========== Web 服務 (延遲 3 秒，核心 6-7) ==========
    if enable_web:
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
            prefix=get_cpu_prefix('web', cpu_affinity_enabled)
        )

        # API Server
        web_prefix_list = get_cpu_prefix_list('web', cpu_affinity_enabled)
        api_cmd = web_prefix_list + ['ros2', 'run', 'robot_api_server', 'api_server']
        api_server = ExecuteProcess(
            cmd=api_cmd,
            output='screen'
        )

        nodes.append(TimerAction(period=3.0, actions=[rosbridge_node, api_server]))

    return nodes


def generate_launch_description():
    """生成啟動描述（使用 OpaqueFunction 支持動態 CPU 親和性配置）"""
    return LaunchDescription([
        # ========== Launch 參數宣告 ==========
        DeclareLaunchArgument('enable_web', default_value='true',
                             description='啟用 Web 服務 (rosbridge + API)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='使用模擬時間'),
        DeclareLaunchArgument('simulation', default_value='false',
                             description='啟用模擬模式 (不需要實際硬體)'),
        DeclareLaunchArgument('sim_scene', default_value='room',
                             description='模擬場景: empty, room, corridor'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/lidar',
                             description='LiDAR 串口設備路徑'),
        DeclareLaunchArgument('imu_device', default_value='/dev/i2c-7',
                             description='IMU I2C 設備路徑'),
        DeclareLaunchArgument('cpu_affinity', default_value='true',
                             description='啟用 CPU 親和性綁定 (Jetson Orin NX 優化)'),

        # ========== 動態生成節點 ==========
        OpaqueFunction(function=launch_setup),
    ])
