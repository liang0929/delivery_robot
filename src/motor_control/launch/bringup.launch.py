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

  # 無 IMU 模式（BNO055 未接線或故障時）
  ros2 launch motor_control bringup.launch.py enable_imu:=false

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

from motor_control.cpu_affinity import (
    resolve_cpu_affinity,
    get_cpu_prefix as _shared_get_cpu_prefix,
    get_cpu_prefix_list as _shared_get_cpu_prefix_list,
)


# enable_imu:=false 時的提示。EKF 的 odom0 提供 vx 與 vyaw，
# 因此缺少 IMU 仍可推算航向，但航向僅來自輪差、打滑無法修正。
IMU_DISABLED_NOTICE = LogInfo(msg=(
    '[bringup] IMU 已停用 (enable_imu:=false)：EKF 將僅以輪式里程計推算航向，'
    '打滑造成的航向誤差無法被修正。建圖/導航時由 SLAM 或 AMCL 修正 map->odom。'
))

# enable_estop:=false 時的警告。這會移除唯一的自動停止機制。
ESTOP_BYPASSED_NOTICE = LogInfo(msg=(
    '[bringup] ⚠️  急停已旁路 (enable_estop:=false)：/e_stop 將固定回報未觸發。'
    '此模式下機器人沒有任何緊急停止手段，且手動遙控本來就沒有障礙物偵測，'
    '操作時務必全程目視監控。接上實體急停按鈕後請移除此參數。'
))

# ========== 硬體/服務預設常數 ==========
# 以下皆可用同名 launch argument 覆寫（見 generate_launch_description）；
# 集中於此方便查閱與調整預設值。
DEFAULT_ESTOP_GPIO_PIN = 7
DEFAULT_IMU_I2C_ADDRESS = 40
DEFAULT_LIDAR_BAUDRATE = 256000
DEFAULT_ROSBRIDGE_PORT = 9090

# CPU_AFFINITY 與實際線上核心取交集後的結果，由 resolve_cpu_affinity() 於
# launch_setup 開頭填入。key 不存在代表該類節點不綁定 CPU。
_RESOLVED_AFFINITY = {}


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
            # 與 tf_config.yaml / URDF 一致（LiDAR 實際安裝高度 0.336m）
            'translation': {'x': 0.0, 'y': 0.0, 'z': 0.336},
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


# parse_cpu_list / get_online_cpus / resolve_cpu_affinity 的實作已抽到
# motor_control.cpu_affinity（與 nav2 launch 共用，行為保持不變）。


def get_cpu_prefix(affinity_type: str, enabled: bool) -> str:
    """獲取 CPU 親和性 prefix（用於 Node 的 prefix 參數）"""
    cpus = _RESOLVED_AFFINITY.get(affinity_type) if enabled else None
    return _shared_get_cpu_prefix(cpus)


def get_cpu_prefix_list(affinity_type: str, enabled: bool) -> list:
    """獲取 CPU 親和性 prefix 列表（用於 ExecuteProcess 的 cmd 參數）"""
    cpus = _RESOLVED_AFFINITY.get(affinity_type) if enabled else None
    return _shared_get_cpu_prefix_list(cpus)


class _BringupConfig:
    """launch_setup 解析出的執行期設定，供 _build_* 函式共用（僅解析參數，
    不建立任何 Node/Action，行為與抽取前 launch_setup 開頭逐行相同）。"""

    def __init__(self, context):
        # 解析運行時參數
        self.cpu_affinity_enabled = (
            LaunchConfiguration('cpu_affinity').perform(context).lower() == 'true')
        global _RESOLVED_AFFINITY
        _RESOLVED_AFFINITY, self.affinity_warnings = resolve_cpu_affinity()
        self.enable_web = LaunchConfiguration('enable_web').perform(context).lower() == 'true'
        use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
        self.use_sim_time = use_sim_time_str.lower() == 'true'
        self.simulation = LaunchConfiguration('simulation').perform(context).lower() == 'true'
        self.sim_scene = LaunchConfiguration('sim_scene').perform(context)
        self.lidar_port = LaunchConfiguration('lidar_port').perform(context)
        self.imu_device = LaunchConfiguration('imu_device').perform(context)
        self.enable_imu = LaunchConfiguration('enable_imu').perform(context).lower() == 'true'
        self.enable_estop = LaunchConfiguration('enable_estop').perform(context).lower() == 'true'
        self.gpio_pin = int(LaunchConfiguration('gpio_pin').perform(context))
        self.imu_address = int(LaunchConfiguration('imu_address').perform(context))
        self.lidar_baudrate = int(LaunchConfiguration('lidar_baudrate').perform(context))
        self.rosbridge_port = int(LaunchConfiguration('rosbridge_port').perform(context))

        # 套件路徑
        motor_control_dir = get_package_share_directory('motor_control')
        self.motor_config = os.path.join(motor_control_dir, 'config', 'hs_motor_config.yaml')

        # URDF 路徑
        try:
            robot_description_dir = get_package_share_directory('robot_description')
            xacro_file = os.path.join(robot_description_dir, 'urdf', 'robot.urdf.xacro')
            self.use_urdf = os.path.exists(xacro_file)
        except Exception:
            self.use_urdf = False
            xacro_file = None
        self.xacro_file = xacro_file

        # TF 配置（僅在沒有 URDF 時使用）
        self.tf_config = load_tf_config()


def _build_description_nodes(cfg: _BringupConfig) -> list:
    """模式/CPU 親和性提示 + 機器人描述 (URDF 或靜態 TF)。"""
    nodes = []

    # ========== 模式提示 ==========
    if cfg.simulation:
        nodes.append(LogInfo(msg='=== 模擬模式啟動 (Simulation Mode) ==='))
    else:
        nodes.append(LogInfo(msg='=== 真實硬體模式啟動 (Hardware Mode) ==='))

    if cfg.cpu_affinity_enabled:
        nodes.append(LogInfo(msg='=== CPU 親和性已啟用 (Jetson Orin NX 優化) ==='))
        for warning in cfg.affinity_warnings:
            nodes.append(LogInfo(msg=warning))

    # ========== 機器人描述 (URDF) 或靜態 TF ==========
    if cfg.use_urdf:
        # 使用 robot_state_publisher 發布 URDF 和 TF
        nodes.append(LogInfo(msg='=== 使用 URDF 機器人模型 ==='))
        robot_description = Command(['xacro ', cfg.xacro_file])
        nodes.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': cfg.use_sim_time,
                'publish_frequency': 50.0,
            }]
        ))
    else:
        # 回退：使用靜態 TF（當 robot_description 套件不存在時）
        nodes.append(LogInfo(msg='=== 使用靜態 TF（無 URDF）==='))
        nodes.append(create_static_tf_node(
            'base_footprint_to_base_link',
            cfg.tf_config.get('base_footprint_to_base_link', {})
        ))
        nodes.append(create_static_tf_node(
            'base_link_to_laser',
            cfg.tf_config.get('base_link_to_laser', {})
        ))
        nodes.append(create_static_tf_node(
            'base_link_to_imu',
            cfg.tf_config.get('base_link_to_imu', {})
        ))

    return nodes


def _build_hardware_nodes(cfg: _BringupConfig) -> list:
    """真實硬體節點：HS 馬達控制器、LiDAR、IMU、E-Stop。"""
    nodes = []

    # HS 協議馬達控制器 (核心 0-1)
    nodes.append(Node(
        package='motor_control',
        executable='hs_motor_controller',
        name='hs_motor_controller',
        output='screen',
        parameters=[cfg.motor_config, {'use_sim_time': cfg.use_sim_time}],
        prefix=get_cpu_prefix('motor', cfg.cpu_affinity_enabled),
        respawn=True,
        respawn_delay=2.0
    ))

    # LiDAR 節點 (核心 2-3)
    nodes.append(Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': cfg.lidar_port,
            'serial_baudrate': cfg.lidar_baudrate,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
        prefix=get_cpu_prefix('sensor', cfg.cpu_affinity_enabled),
        respawn=True,
        respawn_delay=2.0
    ))

    # IMU 節點 (核心 2-3, 50Hz - EKF 只需 30Hz)
    if cfg.enable_imu:
        nodes.append(Node(
            package='imu_bno055',
            executable='bno055_i2c_node',
            name='bno055',
            namespace='imu',
            output='screen',
            parameters=[{
                'device': cfg.imu_device,
                'address': cfg.imu_address,
                'frame_id': 'imu_link',
                'rate': 50.0,
            }],
            prefix=get_cpu_prefix('sensor', cfg.cpu_affinity_enabled)
        ))
    else:
        nodes.append(IMU_DISABLED_NOTICE)

    # E-Stop GPIO 監控節點 (核心 2-3)
    # enable_estop:=false 時仍啟動節點但固定發布「未觸發」，
    # 保留 /e_stop 的 topic 契約，訂閱端不需要區分兩種情況。
    if not cfg.enable_estop:
        nodes.append(ESTOP_BYPASSED_NOTICE)
    nodes.append(Node(
        package='motor_control',
        executable='e_stop_node',
        name='e_stop_node',
        output='screen',
        parameters=[{
            'gpio_pin': cfg.gpio_pin,
            'active_low': True,
            'poll_rate': 100.0,
            'debounce_ms': 50,
            'simulation': not cfg.enable_estop,
        }],
        prefix=get_cpu_prefix('sensor', cfg.cpu_affinity_enabled),
        respawn=True,
        respawn_delay=2.0
    ))

    return nodes


def _build_sim_nodes(cfg: _BringupConfig) -> list:
    """模擬節點：Mock 馬達控制器、LiDAR、IMU、E-Stop。"""
    nodes = []

    # Mock 馬達控制器 (核心 0-1)
    # 參數與 hs_motor_config.yaml 真機設定一致，
    # 模擬中才能重現速度上限與 min_rpm 死區行為
    nodes.append(Node(
        package='motor_control',
        executable='mock_motor_controller',
        name='mock_motor_controller',
        output='screen',
        parameters=[{
            'wheel_separation': 0.3514,
            'wheel_radius': 0.065,
            'max_linear_vel': 0.16875,
            'max_angular_vel': 0.6,
            'gear_ratio': 20.0,
            'min_rpm': 100.0,
            'max_rpm': 3000.0,
            'odom_frequency': 50.0,
            'use_sim_time': cfg.use_sim_time,
        }],
        prefix=get_cpu_prefix('motor', cfg.cpu_affinity_enabled)
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
            'scene': cfg.sim_scene,
        }],
        prefix=get_cpu_prefix('sensor', cfg.cpu_affinity_enabled)
    ))

    # Mock IMU (核心 2-3, 50Hz - EKF 只需 30Hz)
    # 同樣受 enable_imu 控制，讓「無 IMU」路徑能在模擬中驗證
    if cfg.enable_imu:
        nodes.append(Node(
            package='motor_control',
            executable='mock_imu',
            name='mock_imu',
            output='screen',
            parameters=[{
                'frame_id': 'imu_link',
                'publish_frequency': 50.0,
            }],
            prefix=get_cpu_prefix('sensor', cfg.cpu_affinity_enabled)
        ))
    else:
        nodes.append(IMU_DISABLED_NOTICE)

    # E-Stop 模擬節點 (核心 2-3, 永遠不觸發)
    nodes.append(Node(
        package='motor_control',
        executable='e_stop_node',
        name='e_stop_node',
        output='screen',
        parameters=[{
            'simulation': True,
        }],
        prefix=get_cpu_prefix('sensor', cfg.cpu_affinity_enabled)
    ))

    return nodes


def _build_localization_nodes(cfg: _BringupConfig) -> list:
    """EKF 定位融合 (延遲 2 秒，核心 4-5)。"""
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[cfg.motor_config, {'use_sim_time': cfg.use_sim_time}],
        prefix=get_cpu_prefix('localization', cfg.cpu_affinity_enabled),
        respawn=True,
        respawn_delay=2.0
    )
    return [TimerAction(period=2.0, actions=[ekf_node])]


def _build_web_nodes(cfg: _BringupConfig) -> list:
    """Web 服務 (延遲 3 秒，核心 6-7)：rosbridge WebSocket + API Server。"""
    # rosbridge WebSocket
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': cfg.rosbridge_port,
            'call_services_in_new_thread': True,
            'send_action_goals_in_new_thread': True,
            'default_call_service_timeout': 10.0,
            'max_message_size': 10000000,
            'unregister_timeout': 10.0,
        }],
        prefix=get_cpu_prefix('web', cfg.cpu_affinity_enabled)
    )

    # rosapi: rosbridge 客戶端（如 ros-mcp）靠 /rosapi/* services 查詢
    # topics/nodes，只跑 rosbridge_websocket 不會有這些 services
    rosapi_node = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
        prefix=get_cpu_prefix('web', cfg.cpu_affinity_enabled)
    )

    # API Server
    web_prefix_list = get_cpu_prefix_list('web', cfg.cpu_affinity_enabled)
    api_cmd = web_prefix_list + ['ros2', 'run', 'robot_api_server', 'api_server']
    # respawn: API server 被外部殺掉（如 cleanup_ros.sh）時自動復活，
    # 否則 ExecuteProcess 結束後 launch 不會補起
    api_server = ExecuteProcess(
        cmd=api_cmd,
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    return [TimerAction(period=3.0, actions=[rosbridge_node, rosapi_node, api_server])]


def launch_setup(context, *args, **kwargs):
    """動態生成啟動配置（支持運行時參數解析）

    只負責把參數解析成 _BringupConfig，再依序串起各 _build_* 函式產生的
    節點集合；節點集合與參數與拆分前完全等價。
    """
    cfg = _BringupConfig(context)

    nodes = []
    nodes.extend(_build_description_nodes(cfg))

    # ========== 真實硬體節點 ==========
    if not cfg.simulation:
        nodes.extend(_build_hardware_nodes(cfg))

    # ========== 模擬節點 ==========
    if cfg.simulation:
        nodes.extend(_build_sim_nodes(cfg))

    # ========== EKF 定位融合 ==========
    nodes.extend(_build_localization_nodes(cfg))

    # ========== Web 服務 ==========
    if cfg.enable_web:
        nodes.extend(_build_web_nodes(cfg))

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
        DeclareLaunchArgument('enable_imu', default_value='true',
                             description='啟用 IMU (false 時 EKF 僅用輪式里程計推算航向)'),
        DeclareLaunchArgument('enable_estop', default_value='true',
                             description='啟用實體急停按鈕 (false 時旁路，未接按鈕才可使用)'),
        DeclareLaunchArgument('cpu_affinity', default_value='true',
                             description='啟用 CPU 親和性綁定 (Jetson Orin NX 優化)'),
        DeclareLaunchArgument('gpio_pin', default_value=str(DEFAULT_ESTOP_GPIO_PIN),
                             description='E-Stop GPIO pin 編號'),
        DeclareLaunchArgument('imu_address', default_value=str(DEFAULT_IMU_I2C_ADDRESS),
                             description='IMU I2C 位址'),
        DeclareLaunchArgument('lidar_baudrate', default_value=str(DEFAULT_LIDAR_BAUDRATE),
                             description='LiDAR 串口鮑率'),
        DeclareLaunchArgument('rosbridge_port', default_value=str(DEFAULT_ROSBRIDGE_PORT),
                             description='rosbridge WebSocket port'),

        # ========== 動態生成節點 ==========
        OpaqueFunction(function=launch_setup),
    ])
