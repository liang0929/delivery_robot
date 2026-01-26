"""
Modbus 馬達控制器 - 直接通過 RS-232/RS-485 連接馬達驅動器
取代 ESP32 中間層，由 Jetson 直接控制馬達

適用於：AGV-BLD-2S 馬達驅動器
通訊協議：Modbus RTU over RS-232
"""

import math
import time
import threading
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException



class ModbusMotorController(Node):
    """Modbus 馬達控制節點 - 直接連接馬達驅動器"""

    # Modbus 寄存器地址 (根據 AGV-BLD-2S 手冊)
    ADDR_RESET_FAULT = 2000
    ADDR_MOTOR_A_STATE = 2002
    ADDR_MOTOR_B_STATE = 2003
    ADDR_MOTOR_A_DIR = 2004
    ADDR_MOTOR_B_DIR = 2005
    ADDR_MOTOR_A_SPEED_SP = 2006
    ADDR_MOTOR_B_SPEED_SP = 2007
    ADDR_MOTOR_A_SPEED_PV = 1002
    ADDR_MOTOR_B_SPEED_PV = 1003
    ADDR_FAULT_CODE = 1005

    def __init__(self):
        super().__init__('modbus_motor_controller')

        # 宣告參數
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('slave_id', 1)
        self.declare_parameter('wheel_separation', 0.27)
        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('max_linear_vel', 0.05)
        self.declare_parameter('max_angular_vel', 0.4)
        self.declare_parameter('gear_ratio', 20.0)
        self.declare_parameter('min_rpm', 100.0)
        self.declare_parameter('max_rpm', 3000.0)
        self.declare_parameter('odom_frequency', 50.0)

        # 獲取參數
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.slave_id = self.get_parameter('slave_id').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.min_rpm = self.get_parameter('min_rpm').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.odom_frequency = self.get_parameter('odom_frequency').value

        # 驗證參數
        self._validate_parameters()

        # Modbus 客戶端
        self.client: Optional[ModbusSerialClient] = None
        self.connect_modbus()

        # ROS2 發布者和訂閱者
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', qos)

        # TF 廣播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 里程計狀態
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_time = self.get_clock().now()

        # 馬達方向記錄
        self.last_dir_a = 0
        self.last_dir_b = 0

        # 安全控制
        self.last_cmd_time = time.time()
        self.running = True

        # Modbus 鎖 (避免同時讀寫)
        self.modbus_lock = threading.Lock()

        # 啟用馬達
        self.enable_motors()

        # 定時器 - 里程計更新
        odom_period = 1.0 / self.odom_frequency
        self.odom_timer = self.create_timer(odom_period, self.update_odometry)

        # 定時器 - 安全檢查
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info(
            f'Modbus Motor Controller initialized on {self.serial_port}'
        )

    def _validate_parameters(self):
        """驗證參數有效性"""
        errors = []

        # 驗證 odom_frequency（避免除零錯誤）
        if self.odom_frequency <= 0:
            errors.append(f'odom_frequency 必須大於 0，當前值: {self.odom_frequency}')

        # 驗證物理參數
        if self.wheel_radius <= 0:
            errors.append(f'wheel_radius 必須大於 0，當前值: {self.wheel_radius}')

        if self.wheel_separation <= 0:
            errors.append(f'wheel_separation 必須大於 0，當前值: {self.wheel_separation}')

        if self.gear_ratio <= 0:
            errors.append(f'gear_ratio 必須大於 0，當前值: {self.gear_ratio}')

        # 驗證 RPM 範圍
        if self.min_rpm < 0:
            errors.append(f'min_rpm 不能為負數，當前值: {self.min_rpm}')

        if self.max_rpm <= 0:
            errors.append(f'max_rpm 必須大於 0，當前值: {self.max_rpm}')

        if self.max_rpm <= self.min_rpm:
            errors.append(f'max_rpm ({self.max_rpm}) 必須大於 min_rpm ({self.min_rpm})')

        # 驗證速度限制
        if self.max_linear_vel <= 0:
            errors.append(f'max_linear_vel 必須大於 0，當前值: {self.max_linear_vel}')

        if self.max_angular_vel <= 0:
            errors.append(f'max_angular_vel 必須大於 0，當前值: {self.max_angular_vel}')

        # 如果有錯誤，拋出異常
        if errors:
            error_msg = '參數驗證失敗:\n' + '\n'.join(f'  - {e}' for e in errors)
            self.get_logger().fatal(error_msg)
            raise ValueError(error_msg)

        self.get_logger().info('參數驗證通過')

    def connect_modbus(self) -> bool:
        """連接 Modbus 設備"""
        try:
            self.client = ModbusSerialClient(
                port=self.serial_port,
                baudrate=self.baudrate,
                parity='N',
                stopbits=1,
                bytesize=8,
                timeout=1.0
            )
            if self.client.connect():
                self.get_logger().info(f'Connected to Modbus device on {self.serial_port}')
                return True
            else:
                self.get_logger().error(f'Failed to connect to {self.serial_port}')
                return False
        except Exception as e:
            self.get_logger().error(f'Modbus connection error: {e}')
            return False

    def enable_motors(self):
        """啟用馬達"""
        if not self.client or not self.client.connected:
            return

        with self.modbus_lock:
            try:
                # 設定馬達為控制模式 (1 = 控制運轉)
                self.client.write_register(
                    self.ADDR_MOTOR_A_STATE, 1, device_id=self.slave_id)
                self.client.write_register(
                    self.ADDR_MOTOR_B_STATE, 1, device_id=self.slave_id)
                self.get_logger().info('Motors enabled')
            except ModbusException as e:
                self.get_logger().error(f'Failed to enable motors: {e}')

    def disable_motors(self):
        """停用馬達"""
        if not self.client or not self.client.connected:
            return

        with self.modbus_lock:
            try:
                # 設定馬達速度為 0
                self.client.write_register(
                    self.ADDR_MOTOR_A_SPEED_SP, 0, device_id=self.slave_id)
                self.client.write_register(
                    self.ADDR_MOTOR_B_SPEED_SP, 0, device_id=self.slave_id)
                # 設定馬達為制動模式 (3 = 立即制動)
                self.client.write_register(
                    self.ADDR_MOTOR_A_STATE, 3, device_id=self.slave_id)
                self.client.write_register(
                    self.ADDR_MOTOR_B_STATE, 3, device_id=self.slave_id)
                self.get_logger().info('Motors disabled')
            except ModbusException as e:
                self.get_logger().error(f'Failed to disable motors: {e}')

    def cmd_vel_callback(self, msg: Twist):
        """速度命令回調"""
        self.last_cmd_time = time.time()

        # 驗證輸入值（防止 NaN 或無窮大）
        if math.isnan(msg.linear.x) or math.isinf(msg.linear.x):
            self.get_logger().warning('Invalid linear.x value (NaN/Inf), ignoring command')
            return
        if math.isnan(msg.angular.z) or math.isinf(msg.angular.z):
            self.get_logger().warning('Invalid angular.z value (NaN/Inf), ignoring command')
            return

        # 限制速度
        linear_x = max(min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        angular_z = max(min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)

        # 差動驅動運動學
        left_vel = linear_x - (angular_z * self.wheel_separation / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_separation / 2.0)

        # 發送馬達命令
        self.set_motor_speeds(left_vel, right_vel)

        self.get_logger().debug(
            f'Cmd: linear={linear_x:.3f}, angular={angular_z:.3f} '
            f'-> left={left_vel:.3f}, right={right_vel:.3f}'
        )

    def set_motor_speeds(self, left_vel: float, right_vel: float):
        """設定馬達速度 (m/s)"""
        if not self.client or not self.client.connected:
            return

        # 轉換為輪子 RPM
        left_wheel_rpm = abs(left_vel) / (2 * math.pi * self.wheel_radius) * 60.0
        right_wheel_rpm = abs(right_vel) / (2 * math.pi * self.wheel_radius) * 60.0

        # 輪子 RPM 轉換為馬達 RPM (乘以減速比)
        left_motor_rpm = left_wheel_rpm * self.gear_ratio
        right_motor_rpm = right_wheel_rpm * self.gear_ratio

        # 方向
        self.last_dir_a = 0 if left_vel >= 0 else 1
        self.last_dir_b = 0 if right_vel >= 0 else 1

        # 處理死區 (使用馬達 RPM 比較)
        target_rpm_a = 0
        target_rpm_b = 0
        if left_motor_rpm >= self.min_rpm:
            target_rpm_a = int(min(left_motor_rpm, self.max_rpm))
        if right_motor_rpm >= self.min_rpm:
            target_rpm_b = int(min(right_motor_rpm, self.max_rpm))

        with self.modbus_lock:
            try:
                # 寫入方向
                self.client.write_register(
                    self.ADDR_MOTOR_A_DIR, self.last_dir_a, device_id=self.slave_id)
                self.client.write_register(
                    self.ADDR_MOTOR_B_DIR, self.last_dir_b, device_id=self.slave_id)
                # 寫入速度
                self.client.write_register(
                    self.ADDR_MOTOR_A_SPEED_SP, target_rpm_a, device_id=self.slave_id)
                self.client.write_register(
                    self.ADDR_MOTOR_B_SPEED_SP, target_rpm_b, device_id=self.slave_id)
            except ModbusException as e:
                self.get_logger().error(f'Failed to set motor speeds: {e}')

    def read_motor_speeds(self) -> tuple:
        """讀取馬達實際轉速 (RPM)"""
        if not self.client or not self.client.connected:
            return 0.0, 0.0

        with self.modbus_lock:
            try:
                result = self.client.read_input_registers(
                    self.ADDR_MOTOR_A_SPEED_PV, count=2, device_id=self.slave_id)
                if result is None or result.isError():
                    return 0.0, 0.0
                return float(result.registers[0]), float(result.registers[1])
            except ModbusException as e:
                self.get_logger().error(f'Failed to read motor speeds: {e}')
                return 0.0, 0.0

    def read_fault_code(self) -> int:
        """讀取故障代碼"""
        if not self.client or not self.client.connected:
            return -1

        with self.modbus_lock:
            try:
                result = self.client.read_input_registers(
                    self.ADDR_FAULT_CODE, count=1, device_id=self.slave_id)
                if result is None or result.isError():
                    return -1
                return result.registers[0]
            except ModbusException:
                return -1

    def update_odometry(self):
        """更新里程計"""
        # 讀取馬達轉速 (馬達 RPM)
        motor_rpm_a, motor_rpm_b = self.read_motor_speeds()

        # 馬達 RPM 轉換為輪子 RPM (除以減速比)
        wheel_rpm_a = motor_rpm_a / self.gear_ratio
        wheel_rpm_b = motor_rpm_b / self.gear_ratio

        # 轉換為線速度 (m/s)
        vel_a = (wheel_rpm_a / 60.0) * (2 * math.pi * self.wheel_radius)
        vel_b = (wheel_rpm_b / 60.0) * (2 * math.pi * self.wheel_radius)

        # 加上方向
        if self.last_dir_a == 1:
            vel_a = -vel_a
        if self.last_dir_b == 1:
            vel_b = -vel_b

        # 計算機器人速度
        vx = (vel_a + vel_b) / 2.0
        vth = (vel_b - vel_a) / self.wheel_separation

        # 計算時間差
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 時間差為零或負值時跳過更新（避免除零或時間回跳）
        if dt <= 0:
            return

        # 積分更新位置
        delta_x = vx * math.cos(self.odom_theta) * dt
        delta_y = vx * math.sin(self.odom_theta) * dt
        delta_theta = vth * dt

        self.odom_x += delta_x
        self.odom_y += delta_y
        self.odom_theta += delta_theta

        # 發布里程計 (TF 由 EKF 發布，避免重複)
        self.publish_odometry(vx, vth)
        # self.publish_tf()  # 已移除：TF 由 EKF (robot_localization) 發布

    def publish_odometry(self, vx: float, vth: float):
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

        # 協方差矩陣
        odom.pose.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.03
        ]
        odom.twist.covariance = [
            0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  1e6,  0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  1e6,  0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  1e6,  0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.03
        ]

        self.odom_pub.publish(odom)

    def publish_tf(self):
        """發布 TF 變換"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.odom_x
        t.transform.translation.y = self.odom_y
        t.transform.translation.z = 0.0

        t.transform.rotation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(self.odom_theta / 2.0),
            w=math.cos(self.odom_theta / 2.0)
        )

        self.tf_broadcaster.sendTransform(t)

    def safety_check(self):
        """安全檢查 - 超時停止馬達"""
        if time.time() - self.last_cmd_time > 1.0:
            self.set_motor_speeds(0.0, 0.0)

        # 檢查故障代碼
        fault = self.read_fault_code()
        if fault > 0:
            self.get_logger().warning(f'Motor fault detected: {fault}')

    def destroy_node(self):
        """節點銷毀"""
        self.running = False
        try:
            self.disable_motors()
        except Exception as e:
            self.get_logger().warning(f'Error disabling motors: {e}')
        try:
            if self.client:
                self.client.close()
        except Exception as e:
            self.get_logger().warning(f'Error closing Modbus client: {e}')
        finally:
            super().destroy_node()


def main(args=None):
    try:
        with rclpy.init(args=args):
            controller = ModbusMotorController()
            rclpy.spin(controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
