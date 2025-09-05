from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    motor_control_dir = get_package_share_directory('motor_control')
    
    # 配置文件路徑
    config = os.path.join(motor_control_dir, 'config', 'motor_config.yaml')
    
    # ESP32 馬達控制器
    esp32_motor_node = Node(
        package='motor_control',
        executable='uart_send_to_esp32',
        name='esp32_motor_controller',
        output='screen',
        parameters=[config]
    )

    # EKF 定位融合
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config]
    )

    # 雷達節點
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }]
    )

    # IMU 節點
    imu_node = Node(
        package='ros_imu_bno055',
        executable='bno055',
        name='bno055',
        output='screen',
        parameters=[{
            'connection_type': 'i2c',
            'i2c_bus': 1,
            'i2c_addr': 0x28,
            'frame_id': 'imu_link',
            'frequency': 100.0
        }],
        remappings=[('/bno055/imu', '/imu/data')]
    )

    # 靜態 TF 發布器
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
        esp32_motor_node,
        robot_localization_node,
        lidar_node,
        imu_node,
        base_footprint_to_base_link,
        base_link_to_laser,
        base_link_to_imu
    ])