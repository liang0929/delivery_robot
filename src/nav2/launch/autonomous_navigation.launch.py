from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch.actions import EmitEvent
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os


# ========== Jetson Orin NX CPU 親和性配置 ==========
# 導航相關節點使用核心 4-5（定位層）
CPU_AFFINITY_LOCALIZATION = '4-5'


def get_cpu_prefix(enabled: bool) -> list:
    """獲取 CPU 親和性 prefix"""
    if enabled:
        return ['taskset', '-c', CPU_AFFINITY_LOCALIZATION]
    return []


def get_default_map_path():
    """獲取默認地圖路徑，檢查文件是否存在"""
    default_path = os.path.join(os.path.expanduser('~'), 'base_dev/src/map/map.yaml')

    if os.path.exists(default_path):
        return default_path

    # 如果默認路徑不存在，嘗試查找其他地圖
    map_dir = os.path.join(os.path.expanduser('~'), 'base_dev/src/map')
    if os.path.isdir(map_dir):
        for f in os.listdir(map_dir):
            if f.endswith('.yaml'):
                alt_path = os.path.join(map_dir, f)
                print(f'[WARN] 默認地圖 map.yaml 不存在，使用: {alt_path}')
                return alt_path

    # 返回默認路徑（即使不存在），讓 map_server 報告錯誤
    print(f'[WARN] 未找到地圖文件，導航可能無法正常啟動')
    return default_path


def launch_setup(context, *args, **kwargs):
    """動態生成啟動配置（支持運行時參數解析）"""
    # 解析運行時參數
    cpu_affinity_enabled = LaunchConfiguration('cpu_affinity').perform(context).lower() == 'true'
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'true'
    map_yaml_file = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    cpu_prefix = get_cpu_prefix(cpu_affinity_enabled)
    nodes = []

    if cpu_affinity_enabled:
        nodes.append(LogInfo(msg='=== 導航節點 CPU 親和性已啟用 (核心 4-5) ==='))

    # Map Server - 載入地圖 (核心 4-5)
    nodes.append(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }],
        prefix=cpu_prefix
    ))

    # Map Relay (核心 4-5)
    nodes.append(Node(
        package='motor_control',
        executable='map_relay',
        name='map_relay',
        output='screen',
        prefix=cpu_prefix
    ))

    # AMCL - 定位 (核心 4-5)
    nodes.append(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'global_frame_id': 'map',
            'scan_topic': '/scan',
            'robot_model_type': 'nav2_amcl::DifferentialMotionModel',
            'set_initial_pose': True,
            'initial_pose.x': 0.0,
            'initial_pose.y': 0.0,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': 0.0,
            'max_particles': 1000,
            'min_particles': 200,
            'max_beams': 30,
        }],
        prefix=cpu_prefix
    ))

    # Lifecycle Manager for map_server and amcl (延遲 2 秒)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
        prefix=cpu_prefix
    )
    nodes.append(TimerAction(period=2.0, actions=[lifecycle_manager]))

    # Nav2 launch (延遲 3 秒)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time_str,
            'map': map_yaml_file,
            'params_file': params_file
        }.items()
    )
    nodes.append(TimerAction(period=3.0, actions=[nav2_launch]))

    return nodes


def generate_launch_description():
    """生成啟動描述"""
    nav2_dir = get_package_share_directory('nav2')

    return LaunchDescription([
        # ========== Launch 參數宣告 ==========
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=get_default_map_path(),
            description='Full path to map yaml file to load'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav2_dir, 'config', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file'
        ),
        DeclareLaunchArgument(
            'cpu_affinity',
            default_value='true',
            description='啟用 CPU 親和性綁定 (Jetson Orin NX 優化)'
        ),

        # ========== 動態生成節點 ==========
        OpaqueFunction(function=launch_setup),
    ])
