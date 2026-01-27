"""
Mock 馬達控制器 - 用於模擬測試

模擬差動驅動機器人的運動學，不需要實際硬體。
接收 cmd_vel 命令，計算並發布里程計數據。
"""

import math
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, Quaternion, Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from motor_control.odom_constants import POSE_COVARIANCE_SIM, TWIST_COVARIANCE_SIM


class MockMotorController(Node):
    """Mock 馬達控制器 - 模擬差動驅動"""

    def __init__(self):
        super().__init__('mock_motor_controller')

        # 宣告參數
        self.declare_parameter('wheel_separation', 0.27)
        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('max_linear_vel', 0.5)
        self.declare_parameter('max_angular_vel', 1.0)
        self.declare_parameter('odom_frequency', 50.0)

        # 獲取參數
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.odom_frequency = self.get_parameter('odom_frequency').value

        # ROS2 發布者和訂閱者
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', qos)

        # 里程計狀態
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        # 當前速度命令
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # 時間追蹤
        self.last_time = self.get_clock().now()

        # 定時器 - 里程計更新
        odom_period = 1.0 / self.odom_frequency
        self.odom_timer = self.create_timer(odom_period, self.update_odometry)

        self.get_logger().info(
            f'Mock Motor Controller initialized (simulation mode)'
        )
        self.get_logger().info(
            f'  wheel_separation: {self.wheel_separation}m, '
            f'wheel_radius: {self.wheel_radius}m'
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        """速度命令回調"""
        # 驗證輸入值（防止 NaN 或無窮大）
        if math.isnan(msg.linear.x) or math.isinf(msg.linear.x):
            self.get_logger().warning('Invalid linear.x value (NaN/Inf), ignoring command')
            return
        if math.isnan(msg.angular.z) or math.isinf(msg.angular.z):
            self.get_logger().warning('Invalid angular.z value (NaN/Inf), ignoring command')
            return

        # 限制速度
        self.current_linear_x = max(
            min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        self.current_angular_z = max(
            min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)

        self.get_logger().debug(
            f'Cmd: linear={self.current_linear_x:.3f}, '
            f'angular={self.current_angular_z:.3f}'
        )

    def update_odometry(self) -> None:
        """更新里程計 - 模擬理想運動學"""
        # 計算時間差
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # 使用當前速度命令計算位移
        vx = self.current_linear_x
        vth = self.current_angular_z

        # 積分更新位置
        delta_x = vx * math.cos(self.odom_theta) * dt
        delta_y = vx * math.sin(self.odom_theta) * dt
        delta_theta = vth * dt

        self.odom_x += delta_x
        self.odom_y += delta_y
        self.odom_theta += delta_theta

        # 正規化角度到 [-pi, pi]
        while self.odom_theta > math.pi:
            self.odom_theta -= 2 * math.pi
        while self.odom_theta < -math.pi:
            self.odom_theta += 2 * math.pi

        # 發布里程計
        self.publish_odometry(vx, vth)

    def publish_odometry(self, vx: float, vth: float) -> None:
        """發布里程計訊息"""
        odom = Odometry()

        # Header
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        # 位置
        odom.pose.pose.position = Point(
            x=self.odom_x,
            y=self.odom_y,
            z=0.0
        )

        # 方向 (四元數)
        odom.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(self.odom_theta / 2.0),
            w=math.cos(self.odom_theta / 2.0)
        )

        # 速度
        odom.twist.twist.linear = Vector3(x=vx, y=0.0, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=vth)

        # 協方差矩陣 (模擬模式使用較小的協方差)
        odom.pose.covariance = POSE_COVARIANCE_SIM.copy()
        odom.twist.covariance = TWIST_COVARIANCE_SIM.copy()

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    try:
        controller = MockMotorController()
        rclpy.spin(controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
