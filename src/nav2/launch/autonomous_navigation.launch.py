from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
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
        default_value=os.path.join(os.path.expanduser('~'), 'base_dev/src/map/map.yaml'),
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
        map_server_node,
        map_relay_node,
        amcl_node,
        lifecycle_manager_localization,
        nav2_launch
    ])
