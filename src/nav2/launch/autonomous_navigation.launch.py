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


def generate_launch_description():
    # Get the launch directory
    nav2_dir = get_package_share_directory('nav2')

    # Create launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=get_default_map_path(),
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav2_dir, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file'
    )

    # 注意：以下節點 (robot_state_publisher, lidar, motor_control, imu)
    # 已被 bringup.launch.py 啟動，這裡不再重複啟動
    # 如果需要獨立運行導航（不用 bringup），可以取消註解

    # Map Server - 載入地圖
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )

    # Map Relay (解決 map_server 與 rosbridge QoS 不相容問題)
    map_relay_node = Node(
        package='motor_control',
        executable='map_relay',
        name='map_relay',
        output='screen'
    )

    # AMCL - 定位
    amcl_node = Node(
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
            'max_particles': 2000,
            'min_particles': 500,
        }]
    )

    # Lifecycle Manager for map_server and amcl
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # Include Nav2 launch
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
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'params_file': params_file
        }.items()
    )

    # ========== 啟動順序說明 ==========
    # 1. map_server, amcl (立即啟動)
    # 2. lifecycle_manager_localization (延遲 2 秒) - 等待 map_server 和 amcl 準備好
    # 3. nav2_launch (延遲 3 秒) - 等待定位準備好

    # Lifecycle Manager 延遲啟動
    delayed_lifecycle_manager = TimerAction(
        period=2.0,
        actions=[lifecycle_manager_localization]
    )

    # Nav2 延遲啟動
    delayed_nav2 = TimerAction(
        period=3.0,
        actions=[nav2_launch]
    )

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_sim_time_cmd,
        declare_map_yaml_cmd,
        declare_params_file_cmd,
        # 以下節點已被 bringup.launch.py 啟動，不再重複
        # robot_state_publisher_launch,
        # lidar_launch,
        # motor_control_launch,
        # imu_node,
        # base_link_to_imu,

        # 定位節點 (立即啟動)
        map_server_node,
        map_relay_node,
        amcl_node,

        # Lifecycle Manager (延遲 2 秒)
        delayed_lifecycle_manager,

        # Nav2 (延遲 3 秒)
        delayed_nav2,
    ])
