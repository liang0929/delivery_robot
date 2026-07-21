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


def get_cpu_prefix(enabled: bool) -> str:
    """獲取 CPU 親和性 prefix（返回字符串格式，用於 Node 的 prefix 參數）"""
    if enabled:
        return f'taskset -c {CPU_AFFINITY_LOCALIZATION}'
    return ''


def get_default_map_path():
    """獲取默認地圖路徑，檢查文件是否存在"""
    # 修正：地圖目錄在 base_dev/map/ 而不是 base_dev/src/map/
    default_path = os.path.join(os.path.expanduser('~'), 'base_dev/map/map.yaml')

    if os.path.exists(default_path):
        return default_path

    # 如果默認路徑不存在，嘗試查找其他地圖
    map_dir = os.path.join(os.path.expanduser('~'), 'base_dev/map')
    if os.path.isdir(map_dir):
        # 優先使用 sim0.yaml（常用的模擬地圖）
        sim_path = os.path.join(map_dir, 'sim0.yaml')
        if os.path.exists(sim_path):
            print(f'[INFO] 使用默認地圖: {sim_path}')
            return sim_path

        # 如果 sim0 不存在，查找其他地圖
        for f in sorted(os.listdir(map_dir)):
            if f.endswith('.yaml'):
                alt_path = os.path.join(map_dir, f)
                print(f'[WARN] 默認地圖 map.yaml 不存在，使用: {alt_path}')
                return alt_path

    # 返回默認路徑（即使不存在），讓 map_server 報告錯誤
    print(f'[ERROR] 未找到地圖文件，導航無法正常啟動')
    return default_path


def get_default_keepout_mask(map_yaml_file: str) -> str:
    """由地圖路徑推導預設 keepout mask 路徑：<map 同目錄>/<map>.keepout.yaml"""
    map_dir = os.path.dirname(os.path.abspath(map_yaml_file))
    map_name = os.path.splitext(os.path.basename(map_yaml_file))[0]
    return os.path.join(map_dir, f'{map_name}.keepout.yaml')


def ensure_keepout_mask(mask_yaml_file: str) -> bool:
    """確認 keepout mask 存在；不存在時嘗試由虛擬牆重新產生。

    回傳 mask 是否可用；不可用時呼叫端應跳過 costmap filter 節點。
    """
    if os.path.isfile(mask_yaml_file):
        return True

    map_dir = os.path.dirname(mask_yaml_file)
    map_name = os.path.basename(mask_yaml_file)[:-len('.keepout.yaml')]
    try:
        from nav2.keepout import regenerate_keepout
        if regenerate_keepout(map_name, map_dir):
            print(f'[INFO] 已產生 keepout mask: {mask_yaml_file}')
            return True
    except Exception as exc:  # noqa: BLE001
        print(f'[WARN] 產生 keepout mask 失敗: {exc}')

    print(f'[WARN] 找不到 keepout mask ({mask_yaml_file})，虛擬牆功能停用')
    return False


def launch_setup(context, *args, **kwargs):
    """動態生成啟動配置（支持運行時參數解析）"""
    # 解析運行時參數
    cpu_affinity_enabled = LaunchConfiguration('cpu_affinity').perform(context).lower() == 'true'
    namespace = LaunchConfiguration('namespace').perform(context)
    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time = use_sim_time_str.lower() == 'true'
    map_yaml_file = LaunchConfiguration('map').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)
    keepout_mask_file = LaunchConfiguration('keepout_mask').perform(context)
    if not keepout_mask_file:
        keepout_mask_file = get_default_keepout_mask(map_yaml_file)

    cpu_prefix = get_cpu_prefix(cpu_affinity_enabled)
    nodes = []

    if cpu_affinity_enabled:
        nodes.append(LogInfo(msg='=== 導航節點 CPU 親和性已啟用 (核心 4-5) ==='))

    # 靜態 TF 由 bringup.launch.py 提供，此處不重複發布

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
    # 參數以 nav2_params.yaml 為單一真相來源，
    # 僅 launch 專屬項（use_sim_time、初始位姿）以覆蓋方式提供
    nodes.append(Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'set_initial_pose': True,
                'initial_pose.x': 0.0,
                'initial_pose.y': 0.0,
                'initial_pose.z': 0.0,
                'initial_pose.yaw': 0.0,
            },
        ],
        prefix=cpu_prefix
    ))

    # ===== Costmap Filters：虛擬牆 keepout =====
    # filter_mask_server 發布 mask（OccupancyGrid），
    # costmap_filter_info_server 發布對應的 CostmapFilterInfo。
    # mask 不存在（例如尚未建立虛擬牆或仍在建圖）時整組跳過，不影響一般導航。
    lifecycle_nodes = ['map_server', 'amcl']

    if ensure_keepout_mask(keepout_mask_file):
        nodes.append(LogInfo(msg=f'=== keepout mask: {keepout_mask_file} ==='))
        nodes.append(Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            parameters=[
                params_file,
                {
                    'use_sim_time': use_sim_time,
                    'yaml_filename': keepout_mask_file,
                },
            ],
            prefix=cpu_prefix
        ))
        nodes.append(Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[
                params_file,
                {'use_sim_time': use_sim_time},
            ],
            prefix=cpu_prefix
        ))
        # 順序重要：mask 要先 active，info server 才有意義
        lifecycle_nodes += ['filter_mask_server', 'costmap_filter_info_server']

    # Lifecycle Manager for map_server and amcl (延遲 2 秒)
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': lifecycle_nodes
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
            'keepout_mask',
            default_value='',
            description='虛擬牆 keepout mask yaml（留空則用 <map 同目錄>/<map>.keepout.yaml）'
        ),
        DeclareLaunchArgument(
            'cpu_affinity',
            default_value='true',
            description='啟用 CPU 親和性綁定 (Jetson Orin NX 優化)'
        ),

        # ========== 動態生成節點 ==========
        OpaqueFunction(function=launch_setup),
    ])
