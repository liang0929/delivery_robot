"""
Mock IMU 節點 - 用於模擬測試

模擬 IMU 感測器數據，訂閱 odom_raw 來獲取角速度資訊，
發布對應的 IMU 數據。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header


class MockImu(Node):
    """Mock IMU 節點 - 模擬 IMU 感測器"""

    def __init__(self):
        super().__init__('mock_imu')

        # 宣告參數
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_frequency', 100.0)

        # 獲取參數
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_frequency = self.get_parameter('publish_frequency').value

        # ROS2 發布者和訂閱者
        qos = QoSProfile(depth=10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos)
        self.odom_sub = self.create_subscription(
            Odometry, 'odom_raw', self.odom_callback, qos)

        # 當前狀態 (從 odom 獲取)
        self.current_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.current_angular_velocity = Vector3(x=0.0, y=0.0, z=0.0)

        # 定時器
        publish_period = 1.0 / self.publish_frequency
        self.imu_timer = self.create_timer(publish_period, self.publish_imu)

        self.get_logger().info(
            f'Mock IMU initialized (simulation mode)'
        )
        self.get_logger().info(
            f'  frame_id: {self.frame_id}, frequency: {self.publish_frequency}Hz'
        )

    def odom_callback(self, msg: Odometry):
        """從里程計獲取方向和角速度"""
        self.current_orientation = msg.pose.pose.orientation
        self.current_angular_velocity = msg.twist.twist.angular

    def publish_imu(self):
        """發布 IMU 訊息"""
        imu = Imu()

        # Header
        imu.header = Header()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.frame_id

        # 方向 (從 odom 獲取)
        imu.orientation = self.current_orientation

        # 方向協方差 (較小，表示較準確)
        imu.orientation_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]

        # 角速度 (從 odom 獲取，添加小噪聲)
        import random
        noise_scale = 0.001
        imu.angular_velocity = Vector3(
            x=self.current_angular_velocity.x + random.gauss(0, noise_scale),
            y=self.current_angular_velocity.y + random.gauss(0, noise_scale),
            z=self.current_angular_velocity.z + random.gauss(0, noise_scale)
        )

        # 角速度協方差
        imu.angular_velocity_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001
        ]

        # 線性加速度 (模擬重力 + 小噪聲)
        imu.linear_acceleration = Vector3(
            x=random.gauss(0, 0.01),
            y=random.gauss(0, 0.01),
            z=9.81 + random.gauss(0, 0.01)  # 重力
        )

        # 線性加速度協方差
        imu.linear_acceleration_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]

        self.imu_pub.publish(imu)


def main(args=None):
    rclpy.init(args=args)
    imu = None

    try:
        imu = MockImu()
        rclpy.spin(imu)
    except KeyboardInterrupt:
        pass
    finally:
        if imu:
            imu.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
