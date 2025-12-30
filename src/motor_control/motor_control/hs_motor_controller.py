"""
HS 協議馬達控制器 - 使用 AGV-BLD-2S 自定義 HS 協議
直接通過 RS-232 連接馬達驅動器

HS 協議封包格式：
詢問封包 (Master -> Slave): AA + 地址 + 數據類型 + 故障清除 + 保留 + A控制 + A方向 + A轉速(2B) + B控制 + B方向 + B轉速(2B) + 55 + CRC16
應答封包 (Slave -> Master): 55 + 地址 + A電流(2B) + A轉速(2B) + B電流(2B) + B轉速(2B) + 電壓(2B) + 故障 + AA + CRC16
"""

import math
import time
import struct
import threading
from typing import Optional, Tuple

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


class HSMotorController(Node):
    """HS 協議馬達控制節點"""

    # HS 協議常量
    START_BYTE_MASTER = 0xAA
    END_BYTE_MASTER = 0x55
    START_BYTE_SLAVE = 0x55
    END_BYTE_SLAVE = 0xAA

    # 馬達控制狀態
    MOTOR_DISABLE = 0x00
    MOTOR_ENABLE = 0x01
    MOTOR_BRAKE = 0x03

    def __init__(self):
        super().__init__('hs_motor_controller')

        # 宣告參數
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('device_id', 1)
        self.declare_parameter('wheel_separation', 0.381)
        self.declare_parameter('wheel_radius', 0.065)
        self.declare_parameter('gear_ratio', 1.0)  # 減速比
        self.declare_parameter('max_linear_vel', 1.11)
        self.declare_parameter('max_angular_vel', 2.0)
        self.declare_parameter('min_rpm', 100.0)
        self.declare_parameter('max_rpm', 3000.0)
        self.declare_parameter('control_frequency', 20.0)
        self.declare_parameter('invert_motor_a', True)  # A馬達反轉
        self.declare_parameter('invert_motor_b', False)

        # 獲取參數
        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.device_id = self.get_parameter('device_id').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.gear_ratio = self.get_parameter('gear_ratio').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        self.min_rpm = self.get_parameter('min_rpm').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.control_frequency = self.get_parameter('control_frequency').value
        self.invert_motor_a = self.get_parameter('invert_motor_a').value
        self.invert_motor_b = self.get_parameter('invert_motor_b').value

        # 串口連接
        self.serial_conn: Optional[serial.Serial] = None
        self.connect_serial()

        # ROS2 發布者和訂閱者
        qos = QoSProfile(depth=10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', qos)

        # TF 廣播器
        self.tf_broadcaster = TransformBroadcaster(self)

        # 馬達狀態
        self.target_rpm_a = 0
        self.target_rpm_b = 0
        self.dir_a = 0  # 0=正向, 1=反向 (物理方向，發送給驅動器)
        self.dir_b = 0
        self.logical_dir_a = 0  # 邏輯方向 (用於里程計，反轉前)
        self.logical_dir_b = 0
        self.motor_enabled = True

        # 回饋數據
        self.actual_rpm_a = 0.0
        self.actual_rpm_b = 0.0
        self.current_a = 0.0
        self.current_b = 0.0
        self.voltage = 0.0
        self.fault_code = 0

        # 里程計狀態
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_time = self.get_clock().now()

        # 安全控制
        self.last_cmd_time = time.time()
        self.running = True

        # 串口鎖
        self.serial_lock = threading.Lock()

        # 控制定時器
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(control_period, self.control_loop)

        # 安全定時器
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info(
            f'HS Motor Controller initialized on {self.serial_port}'
        )

    def connect_serial(self) -> bool:
        """連接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.get_logger().info(f'Connected to {self.serial_port}')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect: {e}')
            return False

    def crc16(self, data: bytes) -> int:
        """計算 CRC16 校驗碼 (Modbus CRC16)"""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def build_command_packet(self, clear_fault: int = 0) -> bytes:
        """
        建立 HS 協議命令封包 (16 bytes)
        格式: AA + 地址 + 返回類型 + 故障清除 + 保留 + A控制 + B控制 + A方向 + B方向 + A轉速(2B) + B轉速(2B) + 55 + CRC16
        """
        packet = bytearray()

        # Byte 1: 起始碼 (AA)
        packet.append(self.START_BYTE_MASTER)

        # Byte 2: 地址碼 (1-127)
        packet.append(self.device_id & 0x7F)

        # Byte 3: 返回數據類型 (00: 不返回, 01: 返回運行數據)
        packet.append(0x01)  # 需要回傳數據

        # Byte 4: 故障清除 (00: 默認, 01→00: 復位操作)
        packet.append(clear_fault & 0x01)

        # Byte 5: 保留位 (00)
        packet.append(0x00)

        # Byte 6: A電機控制 (00: 失能, 01: 使能, 03: 制動)
        packet.append(self.MOTOR_ENABLE if self.motor_enabled else self.MOTOR_DISABLE)

        # Byte 7: B電機控制 (00: 失能, 01: 使能, 03: 制動)
        packet.append(self.MOTOR_ENABLE if self.motor_enabled else self.MOTOR_DISABLE)

        # Byte 8: A電機運行方向 (00: 正轉, 01: 反轉)
        packet.append(self.dir_a & 0x01)

        # Byte 9: B電機運行方向 (00: 正轉, 01: 反轉)
        packet.append(self.dir_b & 0x01)

        # Byte 10-11: A電機轉速值 (高位在前, 低位在後) 100-3000 RPM
        packet.append((self.target_rpm_a >> 8) & 0xFF)  # 高位
        packet.append(self.target_rpm_a & 0xFF)         # 低位

        # Byte 12-13: B電機轉速值 (高位在前, 低位在後) 100-3000 RPM
        packet.append((self.target_rpm_b >> 8) & 0xFF)  # 高位
        packet.append(self.target_rpm_b & 0xFF)         # 低位

        # Byte 14: 結束碼 (55)
        packet.append(self.END_BYTE_MASTER)

        # Byte 15-16: CRC16 校驗碼 (低位在前, 高位在後)
        # CRC 計算範圍: 起始碼到結束碼 (Byte 1-14, 即 packet[0:14])
        crc = self.crc16(bytes(packet[0:14]))
        packet.append(crc & 0xFF)         # CRC 低位
        packet.append((crc >> 8) & 0xFF)  # CRC 高位

        return bytes(packet)

    def parse_response_packet(self, data: bytes) -> bool:
        """
        解析 HS 協議回應封包 (16 bytes)
        格式: 55 + 地址 + A電流(2B) + B電流(2B) + A轉速(2B) + B轉速(2B) + 電壓(2B) + 故障 + AA + CRC16
        """
        if len(data) < 16:
            return False

        # 找到起始碼 0x55
        try:
            start_idx = data.index(self.START_BYTE_SLAVE)
        except ValueError:
            return False

        if len(data) - start_idx < 16:
            return False

        packet = data[start_idx:start_idx + 16]

        # 驗證結束碼 (Byte 14 = 0xAA)
        if packet[13] != self.END_BYTE_SLAVE:
            self.get_logger().debug(f'Invalid end byte: {packet[13]:02X}')
            return False

        # 驗證 CRC16 (Byte 15-16, 低位在前)
        received_crc = packet[14] | (packet[15] << 8)
        calculated_crc = self.crc16(packet[0:14])  # CRC 計算範圍: 起始碼到結束碼
        if received_crc != calculated_crc:
            self.get_logger().debug(f'CRC mismatch: recv={received_crc:04X} calc={calculated_crc:04X}')
            return False

        # 解析數據 (高位在前 Big-endian)
        # Byte 2: 地址
        addr = packet[1]

        # Byte 3-4: A電機電流 (解析度 0.1A)
        self.current_a = int.from_bytes(packet[2:4], byteorder='big') * 0.1

        # Byte 5-6: B電機電流 (解析度 0.1A)
        self.current_b = int.from_bytes(packet[4:6], byteorder='big') * 0.1

        # Byte 7-8: A電機轉速 (0-3000 RPM)
        self.actual_rpm_a = float(int.from_bytes(packet[6:8], byteorder='big'))

        # Byte 9-10: B電機轉速 (0-3000 RPM)
        self.actual_rpm_b = float(int.from_bytes(packet[8:10], byteorder='big'))

        # Byte 11-12: 電源電壓 (解析度 0.01V)
        self.voltage = int.from_bytes(packet[10:12], byteorder='big') * 0.01

        # Byte 13: 故障狀態 (00 = 正常)
        self.fault_code = packet[12]

        return True

    def send_and_receive(self, clear_fault: int = 0) -> bool:
        """發送命令並接收回應"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return False

        with self.serial_lock:
            try:
                # 清空接收緩衝區
                self.serial_conn.reset_input_buffer()

                # 發送命令
                packet = self.build_command_packet(clear_fault)
                self.serial_conn.write(packet)
                self.get_logger().debug(f'TX: {packet.hex()}')

                # 等待回應
                time.sleep(0.02)

                # 讀取回應
                response = self.serial_conn.read(32)
                if len(response) > 0:
                    self.get_logger().debug(f'RX: {response.hex()}')
                    if self.parse_response_packet(response):
                        return True
                    else:
                        self.get_logger().warn(f'Failed to parse response: {response.hex()}')
                else:
                    self.get_logger().debug('No data received')

                return False

            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                return False

    def control_loop(self):
        """控制循環 - 發送命令並更新里程計"""
        # 發送命令並接收回應
        success = self.send_and_receive()

        if success:
            # 更新里程計
            self.update_odometry()

            # 檢查故障
            if self.fault_code > 0:
                self.get_logger().warn(f'Motor fault: {self.fault_code}')
        else:
            self.get_logger().warn('No response from motor driver')

    def update_odometry(self):
        """更新里程計"""
        # 獲取實際轉速並轉換為 m/s
        # 忽略低於死區的 RPM (避免靜止時漂移)
        rpm_deadzone = 10.0
        motor_rpm_a = self.actual_rpm_a if self.actual_rpm_a > rpm_deadzone else 0.0
        motor_rpm_b = self.actual_rpm_b if self.actual_rpm_b > rpm_deadzone else 0.0

        # 馬達 RPM 轉換為輪子 RPM (除以減速比)
        wheel_rpm_a = motor_rpm_a / self.gear_ratio
        wheel_rpm_b = motor_rpm_b / self.gear_ratio

        vel_a = (wheel_rpm_a / 60.0) * (2 * math.pi * self.wheel_radius)
        vel_b = (wheel_rpm_b / 60.0) * (2 * math.pi * self.wheel_radius)

        # 使用邏輯方向 (反轉前的方向) 來決定速度符號
        if motor_rpm_a > 0 and self.logical_dir_a == 1:
            vel_a = -vel_a
        if motor_rpm_b > 0 and self.logical_dir_b == 1:
            vel_b = -vel_b

        # 計算機器人速度
        vx = (vel_a + vel_b) / 2.0
        vth = (vel_b - vel_a) / self.wheel_separation

        # 方向修正 (實測需要反轉)
        vx = -vx
        vth = -vth

        # 計算時間差
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 積分更新位置
        self.odom_x += vx * math.cos(self.odom_theta) * dt
        self.odom_y += vx * math.sin(self.odom_theta) * dt
        self.odom_theta += vth * dt

        # 發布里程計和 TF
        self.publish_odometry(vx, vth)
        self.publish_tf()

    def cmd_vel_callback(self, msg: Twist):
        """速度命令回調"""
        self.last_cmd_time = time.time()

        # 限制速度
        linear_x = max(min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        angular_z = max(min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)

        # 差動驅動運動學
        left_vel = linear_x - (angular_z * self.wheel_separation / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_separation / 2.0)

        # 設定馬達
        self.set_motor_speeds(left_vel, right_vel)

    def set_motor_speeds(self, left_vel: float, right_vel: float):
        """設定馬達速度 (m/s)"""
        # 轉換為輪子 RPM
        left_wheel_rpm = abs(left_vel) / (2 * math.pi * self.wheel_radius) * 60.0
        right_wheel_rpm = abs(right_vel) / (2 * math.pi * self.wheel_radius) * 60.0

        # 輪子 RPM 轉換為馬達 RPM (乘以減速比)
        left_motor_rpm = left_wheel_rpm * self.gear_ratio
        right_motor_rpm = right_wheel_rpm * self.gear_ratio

        # 邏輯方向 (用於里程計，反轉前)
        self.logical_dir_a = 0 if left_vel >= 0 else 1
        self.logical_dir_b = 0 if right_vel >= 0 else 1

        # 物理方向 (發送給驅動器，考慮馬達反轉設定)
        dir_a = self.logical_dir_a
        dir_b = self.logical_dir_b

        # 應用反轉
        if self.invert_motor_a:
            dir_a = 1 - dir_a
        if self.invert_motor_b:
            dir_b = 1 - dir_b

        self.dir_a = dir_a
        self.dir_b = dir_b

        # 處理死區 (使用馬達 RPM)
        if left_motor_rpm >= self.min_rpm:
            self.target_rpm_a = int(min(left_motor_rpm, self.max_rpm))
        else:
            self.target_rpm_a = 0

        if right_motor_rpm >= self.min_rpm:
            self.target_rpm_b = int(min(right_motor_rpm, self.max_rpm))
        else:
            self.target_rpm_b = 0

    def publish_odometry(self, vx: float, vth: float):
        """發布里程計"""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position = Point(x=self.odom_x, y=self.odom_y, z=0.0)
        odom.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(self.odom_theta / 2.0),
            w=math.cos(self.odom_theta / 2.0)
        )

        odom.twist.twist.linear = Vector3(x=vx, y=0.0, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=vth)

        # 協方差
        odom.pose.covariance = [
            0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e6, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e6, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e6, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.03
        ]
        odom.twist.covariance = odom.pose.covariance.copy()

        self.odom_pub.publish(odom)

    def publish_tf(self):
        """發布 TF"""
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
        """安全檢查"""
        if time.time() - self.last_cmd_time > 1.0:
            self.target_rpm_a = 0
            self.target_rpm_b = 0

    def destroy_node(self):
        """節點銷毀"""
        self.running = False
        self.motor_enabled = False
        self.target_rpm_a = 0
        self.target_rpm_b = 0

        # 發送停止命令
        if self.serial_conn and self.serial_conn.is_open:
            self.send_and_receive()
            self.serial_conn.close()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        controller = HSMotorController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
