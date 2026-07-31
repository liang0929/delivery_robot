"""
Mock 馬達控制器 - 用於模擬測試

模擬差動驅動機器人的運動學，不需要實際硬體。
接收 cmd_vel 命令，計算並發布里程計數據。
"""

import math
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist, Quaternion, Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Bool

from motor_control.base_motor_node import BaseMotorNode
from motor_control.odom_constants import POSE_COVARIANCE_SIM, TWIST_COVARIANCE_SIM
from motor_control.kinematics import DifferentialDriveKinematics


class MockMotorController(BaseMotorNode):
    """Mock 馬達控制器 - 模擬差動驅動

    行為對齊 HSMotorController：速度上限、min_rpm 死區 clamp、
    1 秒 cmd_vel watchdog，避免模擬測試通過但真機失敗。
    """

    # 低於此馬達 RPM 的命令視為零命令（與 HSMotorController 一致）
    ZERO_RPM_EPSILON = 1.0

    # 參數宣告表：{參數名: 預設值}，預設值與 hs_motor_config.yaml 保持一致。
    # 參數名與載入後的屬性名（self.<name>）完全一致，見 __init__ 的資料驅動迴圈。
    #   wheel_separation/wheel_radius: 輪距/輪半徑 (m)
    #   max_linear_vel/max_angular_vel: 最大線速度 (m/s) / 最大角速度 (rad/s)
    #   odom_frequency: 里程計發布頻率 (Hz)
    #   gear_ratio: 減速比
    #   min_rpm: 模擬驅動器低速死區 / max_rpm: 驅動器最大 RPM
    PARAMS = {
        'wheel_separation': 0.3514,
        'wheel_radius': 0.065,
        'max_linear_vel': 0.16875,
        'max_angular_vel': 0.6,
        'odom_frequency': 50.0,
        'gear_ratio': 20.0,
        'min_rpm': 100.0,
        'max_rpm': 3000.0,
    }

    def __init__(self):
        super().__init__('mock_motor_controller')

        # 宣告參數並取值（資料驅動：見類別頂部 PARAMS，
        # 參數名稱/預設值/型別與屬性名皆與抽取前完全一致）
        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)

        # 運動學計算（純模組，與 HSMotorController 共用等價部分，數值與抽取前完全相同）
        self.kinematics = DifferentialDriveKinematics(
            wheel_separation=self.wheel_separation,
            wheel_radius=self.wheel_radius,
            gear_ratio=self.gear_ratio,
            min_rpm=self.min_rpm,
            max_rpm=self.max_rpm,
            zero_rpm_epsilon=self.ZERO_RPM_EPSILON,
        )

        # ROS2 發布者和訂閱者
        qos = self._make_reliable_qos()
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', qos)

        # E-Stop 訂閱 (TRANSIENT_LOCAL 確保收到 latched 狀態；含 e_stop_active 初始化)
        self._setup_e_stop_subscription()

        # 里程計狀態
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0

        # 當前速度命令
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # 時間追蹤
        self.last_time = self.get_clock().now()
        self.last_cmd_time = self.get_clock().now()

        # 定時器 - 里程計更新
        odom_period = 1.0 / self.odom_frequency
        self.odom_timer = self.create_timer(odom_period, self.update_odometry)

        # 安全定時器 - cmd_vel watchdog（與真機一致，1 秒逾時歸零）
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info(
            f'Mock Motor Controller initialized (simulation mode)'
        )
        self.get_logger().info(
            f'  wheel_separation: {self.wheel_separation}m, '
            f'wheel_radius: {self.wheel_radius}m'
        )

    def e_stop_callback(self, msg: Bool) -> None:
        """E-Stop 狀態回調"""
        prev = self.e_stop_active
        self.e_stop_active = msg.data

        if msg.data:
            # 急停啟動：立即歸零速度
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0
            if not prev:
                self.get_logger().warn('E-STOP ACTIVATED - velocities zeroed')
        elif prev:
            self.get_logger().info('E-Stop released - accepting commands')

    def cmd_vel_callback(self, msg: Twist) -> None:
        """速度命令回調"""
        # E-Stop 啟動時拒絕所有速度命令
        if self.e_stop_active:
            return

        # 驗證輸入值（防止 NaN 或無窮大；共用基底的驗證，訊息與行為與抽取前相同）
        if not self._validate_cmd_vel(msg):
            return

        # 驗證通過後才更新 watchdog 時間（與真機一致）
        self.last_cmd_time = self.get_clock().now()

        # 限制速度
        linear_x = max(
            min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        angular_z = max(
            min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)

        # 差動運動學 + min_rpm 死區 clamp（模擬真機驅動器行為）
        left_vel, right_vel = self.kinematics.twist_to_wheel_vel(linear_x, angular_z)

        left_vel = self._quantize_wheel_vel(left_vel)
        right_vel = self._quantize_wheel_vel(right_vel)

        # 換算回機器人速度
        self.current_linear_x, self.current_angular_z = self.kinematics.wheel_vel_to_twist(
            left_vel, right_vel)

        self.get_logger().debug(
            f'Cmd: linear={self.current_linear_x:.3f}, '
            f'angular={self.current_angular_z:.3f}'
        )

    def _quantize_wheel_vel(self, wheel_vel: float) -> float:
        """模擬驅動器 RPM 量化：非零命令低於 min_rpm 時 clamp 到 min_rpm

        與 HSMotorController._quantize_rpm 行為一致（clamp 邏輯與換算公式
        皆委派給共用的 kinematics 純函式，數值與抽取前完全相同）：
        - 低於 ZERO_RPM_EPSILON 的馬達 RPM 視為零命令 → 0
        - 非零但低於 min_rpm → clamp 到 min_rpm（保留方向）
        - 其餘 clamp 到 max_rpm
        """
        motor_rpm = self.kinematics.wheel_vel_to_motor_rpm(wheel_vel)
        quantized_rpm = self.kinematics.quantize_motor_rpm(motor_rpm)
        if quantized_rpm == 0.0:
            return 0.0
        quantized = self.kinematics.motor_rpm_to_wheel_vel(quantized_rpm)
        return math.copysign(quantized, wheel_vel)

    def safety_check(self) -> None:
        """cmd_vel watchdog - 1 秒未收到命令即歸零（逾時判斷共用基底，
        歸零動作為 mock 特有的開迴路行為，不共用）"""
        if self._is_cmd_vel_stale():
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0

    def update_odometry(self) -> None:
        """更新里程計 - 模擬理想運動學"""
        # 計算時間差
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # E-Stop 啟動時速度歸零
        if self.e_stop_active:
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0

        # 使用當前速度命令計算位移
        vx = self.current_linear_x
        vth = self.current_angular_z

        # 積分更新位置（與 HSMotorController 共用的 unicycle 積分公式，數值相同）
        self.odom_x, self.odom_y, self.odom_theta = self.kinematics.integrate_odometry(
            self.odom_x, self.odom_y, self.odom_theta, vx, vth, dt)

        # 正規化角度到 [-pi, pi]（mock 特有行為，HSMotorController 未做此正規化，
        # 不納入共用的 integrate_odometry，避免改變真機行為）
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
