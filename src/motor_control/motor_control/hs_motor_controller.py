"""
HS 協議馬達控制器 - 使用 AGV-BLD-2S 自定義 HS 協議
直接通過 RS-232 連接馬達驅動器

HS 協議封包格式：
詢問封包 (Master -> Slave): AA + 地址 + 數據類型 + 故障清除 + 保留 + A控制 + B控制 + A方向 + B方向 + A轉速(2B) + B轉速(2B) + 55 + CRC16
應答封包 (Slave -> Master): 55 + 地址 + A電流(2B) + B電流(2B) + A轉速(2B) + B轉速(2B) + 電壓(2B) + 故障 + AA + CRC16

注意：欄位順序以實測硬體行為為準，如與手冊不符請先驗證再修改。
"""

import math
import time
import threading
from typing import Optional

import serial
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, Quaternion, Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32, Bool
from std_srvs.srv import Trigger

from motor_control.odom_constants import POSE_COVARIANCE, TWIST_COVARIANCE


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

    # 里程計參數
    RPM_DEADZONE = 10.0  # 低於此值的 RPM 視為靜止，避免漂移

    # 低於此馬達 RPM 的命令視為零命令（吸收運動學計算的浮點殘差）
    ZERO_RPM_EPSILON = 1.0

    # 嚴重故障碼：短路 (1, 2)、霍爾感測器錯誤 (7, 8) → 立即停止馬達
    SEVERE_FAULT_CODES = frozenset({1, 2, 7, 8})

    def __init__(self):
        super().__init__('hs_motor_controller')

        # 宣告參數（預設值與 hs_motor_config.yaml 保持一致）
        self.declare_parameter('serial_port', '/dev/motor')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('device_id', 127)  # 廣播地址
        self.declare_parameter('wheel_separation', 0.27)  # 輪距 (m)
        self.declare_parameter('wheel_radius', 0.065)  # 輪半徑 (m)
        self.declare_parameter('gear_ratio', 20.0)  # 減速比 (馬達轉20圈=輪子轉1圈)
        self.declare_parameter('max_linear_vel', 0.05)  # 最大線速度 (m/s)
        self.declare_parameter('max_angular_vel', 0.4)  # 最大角速度 (rad/s)
        self.declare_parameter('min_rpm', 100.0)  # 最小馬達 RPM (根據 AGV-BLD-2S 手冊)
        self.declare_parameter('max_rpm', 3000.0)  # 最大馬達 RPM
        self.declare_parameter('control_frequency', 50.0)  # 控制頻率 (Hz)
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

        # 驗證參數
        self._validate_parameters()

        # 串口連接
        self.serial_conn: Optional[serial.Serial] = None
        self.connect_serial()

        # ROS2 發布者和訂閱者
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', qos)
        self.voltage_pub = self.create_publisher(Float32, 'motor/voltage', qos)
        self.current_a_pub = self.create_publisher(Float32, 'motor/current_a', qos)
        self.current_b_pub = self.create_publisher(Float32, 'motor/current_b', qos)
        self.fault_pub = self.create_publisher(Int32, 'motor/fault', qos)

        # E-Stop 訂閱 (TRANSIENT_LOCAL 確保收到 latched 狀態)
        e_stop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.e_stop_sub = self.create_subscription(
            Bool, '/e_stop', self.e_stop_callback, e_stop_qos)
        self.e_stop_active = False

        # 故障清除 service（呼叫後下一包帶 clear_fault=1）
        self.pending_clear_fault = False
        self.clear_fault_srv = self.create_service(
            Trigger, '~/clear_fault', self.clear_fault_callback)

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

        # 安全控制（使用 ROS2 時鐘以支援模擬環境）
        self.last_cmd_time = self.get_clock().now()
        self.running = True

        # 串口鎖和重連控制
        self.serial_lock = threading.Lock()
        self.consecutive_failures = 0
        self.max_failures_before_reconnect = 5  # 連續失敗 5 次後嘗試重連
        self.reconnect_cooldown = 3.0  # 重連冷卻期（秒），避免阻塞 executor
        self.last_reconnect_attempt = float('-inf')  # 上次重連嘗試時間 (monotonic)

        # 狀態鎖 - 保護馬達狀態和里程計狀態的並發訪問
        self.state_lock = threading.Lock()

        # 控制定時器
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(control_period, self.control_loop)

        # 安全定時器
        self.safety_timer = self.create_timer(0.1, self.safety_check)

        self.get_logger().info(
            f'HS Motor Controller initialized on {self.serial_port}'
        )

    def _validate_parameters(self) -> None:
        """驗證參數有效性"""
        errors = []

        if self.wheel_radius <= 0:
            errors.append(f"wheel_radius must be positive, got {self.wheel_radius}")
        if self.wheel_separation <= 0:
            errors.append(f"wheel_separation must be positive, got {self.wheel_separation}")
        if self.control_frequency <= 0:
            errors.append(f"control_frequency must be positive, got {self.control_frequency}")
        if self.max_rpm <= self.min_rpm:
            errors.append(f"max_rpm ({self.max_rpm}) must be greater than min_rpm ({self.min_rpm})")
        if self.max_linear_vel <= 0:
            errors.append(f"max_linear_vel must be positive, got {self.max_linear_vel}")
        if self.max_angular_vel <= 0:
            errors.append(f"max_angular_vel must be positive, got {self.max_angular_vel}")
        if self.gear_ratio <= 0:
            errors.append(f"gear_ratio must be positive, got {self.gear_ratio}")
        if self.device_id < 0 or self.device_id > 255:
            errors.append(f"device_id must be 0-255, got {self.device_id}")

        if errors:
            for error in errors:
                self.get_logger().error(f"Parameter validation failed: {error}")
            raise ValueError(f"Invalid parameters: {'; '.join(errors)}")

        self.get_logger().info("All parameters validated successfully")

    def _close_serial_safely(self) -> None:
        """安全關閉串口連接，確保資源完全釋放"""
        if self.serial_conn is None:
            return

        try:
            # 先檢查連接狀態
            if self.serial_conn.is_open:
                # 清空緩衝區
                try:
                    self.serial_conn.reset_input_buffer()
                    self.serial_conn.reset_output_buffer()
                except Exception:
                    pass
                # 關閉連接
                self.serial_conn.close()
                self.get_logger().info('Serial port closed successfully')
        except Exception as e:
            self.get_logger().warning(f'Error closing serial port: {e}')
        finally:
            # 確保引用被清除
            self.serial_conn = None
            # 短暫等待讓系統釋放端口
            time.sleep(0.1)

    def connect_serial(self, max_retries: int = 3, retry_delay: float = 1.0) -> bool:
        """連接串口，支援重試機制

        Args:
            max_retries: 最大重試次數
            retry_delay: 重試間隔（秒），使用指數退避

        Returns:
            bool: 連接是否成功
        """
        # 先確保關閉任何現有連接
        self._close_serial_safely()

        for attempt in range(max_retries):
            try:
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.03
                )
                self.get_logger().info(f'Connected to {self.serial_port}')
                return True
            except serial.SerialException as e:
                if attempt < max_retries - 1:
                    wait_time = retry_delay * (2 ** attempt)  # 指數退避
                    self.get_logger().warning(
                        f'Connection attempt {attempt + 1}/{max_retries} failed: {e}. '
                        f'Retrying in {wait_time:.1f}s...'
                    )
                    time.sleep(wait_time)
                else:
                    self.get_logger().error(
                        f'Failed to connect after {max_retries} attempts: {e}'
                    )
        return False

    def reconnect_serial(self) -> bool:
        """嘗試重新連接串口（單次嘗試，無退避 sleep，避免阻塞 executor）"""
        self.get_logger().info('Attempting to reconnect serial port...')
        # 使用安全關閉方法，然後單次重新連接
        return self.connect_serial(max_retries=1)

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

        注意：欄位順序以實測硬體行為為準，如與手冊不符請先驗證再修改。
        """
        # 使用鎖保護讀取共享狀態，獲取快照
        with self.state_lock:
            e_stop = self.e_stop_active
            dir_a = self.dir_a
            dir_b = self.dir_b
            target_rpm_a = self.target_rpm_a
            target_rpm_b = self.target_rpm_b

        # E-Stop 啟動時：強制制動，RPM 歸零
        if e_stop:
            motor_control_byte = self.MOTOR_BRAKE
            target_rpm_a = 0
            target_rpm_b = 0
        elif self.motor_enabled:
            motor_control_byte = self.MOTOR_ENABLE
        else:
            motor_control_byte = self.MOTOR_DISABLE

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
        packet.append(motor_control_byte)

        # Byte 7: B電機控制 (00: 失能, 01: 使能, 03: 制動)
        packet.append(motor_control_byte)

        # Byte 8: A電機運行方向 (00: 正轉, 01: 反轉)
        packet.append(dir_a & 0x01)

        # Byte 9: B電機運行方向 (00: 正轉, 01: 反轉)
        packet.append(dir_b & 0x01)

        # Byte 10-11: A電機轉速值 (高位在前, 低位在後) 100-3000 RPM
        packet.append((target_rpm_a >> 8) & 0xFF)  # 高位
        packet.append(target_rpm_a & 0xFF)         # 低位

        # Byte 12-13: B電機轉速值 (高位在前, 低位在後) 100-3000 RPM
        packet.append((target_rpm_b >> 8) & 0xFF)  # 高位
        packet.append(target_rpm_b & 0xFF)         # 低位

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

        注意：欄位順序以實測硬體行為為準，如與手冊不符請先驗證再修改。
        """
        if len(data) < 16:
            return False

        # 掃描所有 0x55 候選起始碼，直到找到驗證通過的封包
        # （0x55 可能出現在數據內容中，只試第一個會漏掉緊接在雜訊後的有效封包）
        start_idx = data.find(self.START_BYTE_SLAVE)
        while start_idx != -1 and len(data) - start_idx >= 16:
            packet = data[start_idx:start_idx + 16]
            if self._try_parse_packet(packet):
                return True
            start_idx = data.find(self.START_BYTE_SLAVE, start_idx + 1)

        return False

    def _try_parse_packet(self, packet: bytes) -> bool:
        """驗證並解析單一 16-byte 候選封包，成功時更新回饋狀態"""
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

        # 驗證地址 (Byte 2)；device_id 127 為廣播地址，跳過檢查
        if self.device_id != 127 and packet[1] != self.device_id:
            self.get_logger().debug(
                f'Address mismatch: recv={packet[1]} expect={self.device_id}')
            return False

        # 解析數據 (高位在前 Big-endian)

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
        """發送命令並接收回應，支援自動重連

        重連不在 serial_lock 內執行，避免持鎖期間阻塞其他串口操作。
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            self._handle_serial_failure("Serial port not open")
            return False

        failure_reason = None
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

                # 讀取回應 (應答封包固定 16 bytes；讀超過 16 會等滿 timeout 造成阻塞)
                response = self.serial_conn.read(16)
                if len(response) > 0:
                    self.get_logger().debug(f'RX: {response.hex()}')
                    if self.parse_response_packet(response):
                        self.consecutive_failures = 0  # 成功時重置計數
                        return True
                    else:
                        self.get_logger().warning(f'Failed to parse response: {response.hex()}')

                failure_reason = "No valid response"

            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}')
                failure_reason = str(e)

        # 在鎖外處理失敗（可能觸發重連）
        self._handle_serial_failure(failure_reason)
        return False

    def _handle_serial_failure(self, reason: str) -> None:
        """處理串口通訊失敗，必要時嘗試重連

        重連採冷卻機制：連續失敗達門檻後，每個冷卻期最多嘗試一次
        單次連線（無退避 sleep 迴圈），避免在 executor callback 內
        長時間阻塞導致 e_stop/cmd_vel 無法處理。
        """
        self.consecutive_failures += 1

        if self.consecutive_failures < self.max_failures_before_reconnect:
            return

        now = time.monotonic()
        if now - self.last_reconnect_attempt < self.reconnect_cooldown:
            return  # 冷卻期內不重試，等下個冷卻期

        self.last_reconnect_attempt = now
        self.get_logger().warning(
            f'Serial communication failed {self.consecutive_failures} times ({reason}), attempting reconnect...'
        )
        if self.reconnect_serial():
            self.consecutive_failures = 0
            self.get_logger().info('Serial reconnection successful')
        else:
            self.get_logger().error(
                f'Serial reconnection failed, next attempt in {self.reconnect_cooldown:.0f}s'
            )

    def clear_fault_callback(self, request, response):
        """故障清除 service 回調 - 下一包命令帶 clear_fault=1"""
        with self.state_lock:
            self.pending_clear_fault = True
        response.success = True
        response.message = 'Fault clear scheduled for next command packet'
        self.get_logger().info('Fault clear requested via service')
        return response

    def control_loop(self) -> None:
        """控制循環 - 發送命令並更新里程計"""
        # 取出待處理的故障清除旗標（協議為 01→00 復位，僅送一包）
        with self.state_lock:
            clear_fault = 1 if self.pending_clear_fault else 0
            self.pending_clear_fault = False

        # 發送命令並接收回應
        success = self.send_and_receive(clear_fault)

        if success:
            # 更新里程計
            self.update_odometry()

            # 發布電壓
            voltage_msg = Float32()
            voltage_msg.data = self.voltage
            self.voltage_pub.publish(voltage_msg)

            # 發布電流
            current_a_msg = Float32()
            current_a_msg.data = self.current_a
            self.current_a_pub.publish(current_a_msg)

            current_b_msg = Float32()
            current_b_msg.data = self.current_b
            self.current_b_pub.publish(current_b_msg)

            # 發布故障狀態
            fault_msg = Int32()
            fault_msg.data = self.fault_code
            self.fault_pub.publish(fault_msg)

            # 檢查故障
            if self.fault_code > 0:
                self.get_logger().warning(f'Motor fault code {self.fault_code}: {self.get_fault_description(self.fault_code)}')
                # 嚴重故障（短路/霍爾錯誤）：立即歸零目標轉速
                if self.fault_code in self.SEVERE_FAULT_CODES:
                    with self.state_lock:
                        self.target_rpm_a = 0
                        self.target_rpm_b = 0
                    self.get_logger().error(
                        f'Severe fault {self.fault_code}, target RPM zeroed'
                    )
        else:
            self.get_logger().warning('No response from motor driver')

    def update_odometry(self) -> None:
        """更新里程計"""
        # 獲取實際轉速並轉換為 m/s
        # 忽略低於死區的 RPM (避免靜止時漂移)
        motor_rpm_a = self.actual_rpm_a if self.actual_rpm_a > self.RPM_DEADZONE else 0.0
        motor_rpm_b = self.actual_rpm_b if self.actual_rpm_b > self.RPM_DEADZONE else 0.0

        # 馬達 RPM 轉換為輪子 RPM (除以減速比)
        wheel_rpm_a = motor_rpm_a / self.gear_ratio
        wheel_rpm_b = motor_rpm_b / self.gear_ratio

        vel_a = (wheel_rpm_a / 60.0) * (2 * math.pi * self.wheel_radius)
        vel_b = (wheel_rpm_b / 60.0) * (2 * math.pi * self.wheel_radius)

        # 使用鎖保護讀取邏輯方向和更新里程計
        with self.state_lock:
            # 使用邏輯方向 (反轉前的方向) 來決定速度符號
            if motor_rpm_a > 0 and self.logical_dir_a == 1:
                vel_a = -vel_a
            if motor_rpm_b > 0 and self.logical_dir_b == 1:
                vel_b = -vel_b

            # 計算機器人速度 (Motor A = 物理右輪, Motor B = 物理左輪)
            vx = (vel_a + vel_b) / 2.0
            vth = (vel_a - vel_b) / self.wheel_separation

            # 計算時間差
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # 時間差為零或負值時跳過更新（避免除零或時間回跳）
            if dt <= 0:
                return

            # 積分更新位置
            self.odom_x += vx * math.cos(self.odom_theta) * dt
            self.odom_y += vx * math.sin(self.odom_theta) * dt
            self.odom_theta += vth * dt

        # 發布里程計 (TF 由 EKF 發布，避免重複)
        self.publish_odometry(vx, vth)

    def e_stop_callback(self, msg: Bool) -> None:
        """E-Stop 狀態回調"""
        with self.state_lock:
            prev = self.e_stop_active
            self.e_stop_active = msg.data
            if msg.data:
                # 立即清零目標轉速，避免 e_stop 短暫觸發又釋放時
                # 恢復舊命令造成機器人竄動
                self.target_rpm_a = 0
                self.target_rpm_b = 0

        if msg.data and not prev:
            self.get_logger().warn('E-STOP ACTIVATED - motors will brake')
        elif not msg.data and prev:
            self.get_logger().info('E-Stop released - motors resuming')

    def cmd_vel_callback(self, msg: Twist) -> None:
        """速度命令回調"""
        # E-Stop 啟動時拒絕所有速度命令
        with self.state_lock:
            if self.e_stop_active:
                return

        # 驗證輸入值（防止 NaN 或無窮大）
        if math.isnan(msg.linear.x) or math.isinf(msg.linear.x):
            self.get_logger().warning('Invalid linear.x value (NaN/Inf), ignoring command')
            return
        if math.isnan(msg.angular.z) or math.isinf(msg.angular.z):
            self.get_logger().warning('Invalid angular.z value (NaN/Inf), ignoring command')
            return

        # 驗證通過後才更新 watchdog 時間，避免無效命令流打穿逾時保護
        self.last_cmd_time = self.get_clock().now()

        # 限制速度
        linear_x = max(min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        angular_z = max(min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)

        # 差動驅動運動學
        left_vel = linear_x - (angular_z * self.wheel_separation / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_separation / 2.0)

        # 設定馬達 (Motor A = 物理右輪, Motor B = 物理左輪)
        self.set_motor_speeds(left_vel, right_vel)

    def _quantize_rpm(self, motor_rpm: float) -> int:
        """將馬達 RPM 量化到驅動器有效範圍 [min_rpm, max_rpm]

        - 低於 ZERO_RPM_EPSILON 視為零命令 → 0
        - 非零但低於 min_rpm → clamp 到 min_rpm（避免低速死區導致不動）
        - 其餘 clamp 到 max_rpm
        """
        if motor_rpm < self.ZERO_RPM_EPSILON:
            return 0
        return int(max(min(motor_rpm, self.max_rpm), self.min_rpm))

    def set_motor_speeds(self, left_vel: float, right_vel: float) -> None:
        """設定馬達速度 (m/s)

        Args:
            left_vel: 左輪目標速度 → Motor B
            right_vel: 右輪目標速度 → Motor A
        """
        # 轉換為輪子 RPM
        left_wheel_rpm = abs(left_vel) / (2 * math.pi * self.wheel_radius) * 60.0
        right_wheel_rpm = abs(right_vel) / (2 * math.pi * self.wheel_radius) * 60.0

        # 輪子 RPM 轉換為馬達 RPM (乘以減速比)
        left_motor_rpm = left_wheel_rpm * self.gear_ratio
        right_motor_rpm = right_wheel_rpm * self.gear_ratio

        # 計算邏輯方向 (用於里程計，反轉前)
        # Motor A = 物理右輪, Motor B = 物理左輪
        logical_dir_a = 0 if right_vel >= 0 else 1
        logical_dir_b = 0 if left_vel >= 0 else 1

        # 物理方向 (發送給驅動器，考慮馬達反轉設定)
        dir_a = logical_dir_a
        dir_b = logical_dir_b

        # 應用反轉
        if self.invert_motor_a:
            dir_a = 1 - dir_a
        if self.invert_motor_b:
            dir_b = 1 - dir_b

        # 計算目標 RPM
        # 非零命令低於 min_rpm 時 clamp 到 min_rpm（驅動器有效範圍下限），
        # 避免低速命令完全不動；完全為零的命令仍設 0。
        # Motor A = 右輪
        target_rpm_a = self._quantize_rpm(right_motor_rpm)

        # Motor B = 左輪
        target_rpm_b = self._quantize_rpm(left_motor_rpm)

        # 使用鎖保護共享狀態的寫入
        with self.state_lock:
            self.logical_dir_a = logical_dir_a
            self.logical_dir_b = logical_dir_b
            self.dir_a = dir_a
            self.dir_b = dir_b
            self.target_rpm_a = target_rpm_a
            self.target_rpm_b = target_rpm_b

    def publish_odometry(self, vx: float, vth: float) -> None:
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

        # 協方差 (使用共用常數)
        odom.pose.covariance = POSE_COVARIANCE.copy()
        odom.twist.covariance = TWIST_COVARIANCE.copy()

        self.odom_pub.publish(odom)

    def get_fault_description(self, fault_code: int) -> str:
        """取得故障代碼描述"""
        fault_descriptions = {
            0: "正常",
            1: "A電機短路保護",
            2: "B電機短路保護",
            3: "A電機超載保護",
            4: "B電機超載保護",
            5: "A電機堵轉保護",
            6: "B電機堵轉保護",
            7: "A電機霍爾感測器錯誤",
            8: "B電機霍爾感測器錯誤",
            10: "欠壓保護",
            11: "過壓保護",
        }
        return fault_descriptions.get(fault_code, f"未知故障({fault_code})")

    def safety_check(self) -> None:
        """安全檢查"""
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_since_cmd > 1.0:
            with self.state_lock:
                self.target_rpm_a = 0
                self.target_rpm_b = 0

    def destroy_node(self) -> None:
        """節點銷毀"""
        self.running = False
        with self.state_lock:
            self.e_stop_active = True  # 觸發 MOTOR_BRAKE 而非 MOTOR_DISABLE
            self.target_rpm_a = 0
            self.target_rpm_b = 0

        # 發送停止命令（重試 3 次，確保煞車封包送達）
        if self.serial_conn and self.serial_conn.is_open:
            for attempt in range(3):
                try:
                    if self.send_and_receive():
                        break
                    self.get_logger().warning(
                        f'Stop command attempt {attempt + 1}/3 failed')
                except Exception as e:
                    self.get_logger().warning(
                        f'Error sending stop command (attempt {attempt + 1}/3): {e}')

        # 使用安全關閉方法
        self._close_serial_safely()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    controller = None
    try:
        controller = HSMotorController()
        rclpy.spin(controller)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if controller is not None:
            controller.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
