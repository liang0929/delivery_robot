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
from geometry_msgs.msg import Twist, Quaternion, Point, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Int32, Bool
from std_srvs.srv import Trigger

from motor_control.odom_constants import POSE_COVARIANCE, TWIST_COVARIANCE
from motor_control import hs_protocol
from motor_control.base_motor_node import BaseMotorNode
from motor_control.kinematics import DifferentialDriveKinematics


class HSMotorController(BaseMotorNode):
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

    # 參數宣告表：{參數名: 預設值}，預設值與 hs_motor_config.yaml 保持一致。
    # 參數名與載入後的屬性名（self.<name>）完全一致，見 __init__ 的資料驅動迴圈。
    #   serial_port/baudrate/device_id: 串口與從機位址（device_id 為廣播地址）
    #   wheel_separation/wheel_radius: 輪距/輪半徑 (m)
    #   gear_ratio: 減速比 (馬達轉20圈=輪子轉1圈)
    #   max_linear_vel/max_angular_vel: 最大線速度 (m/s) / 最大角速度 (rad/s)
    #   min_rpm/max_rpm: 驅動器有效 RPM 範圍 (依 AGV-BLD-2S 手冊)
    #   control_frequency: 控制頻率 (Hz)
    #   invert_motor_a/invert_motor_b: 馬達方向反轉
    PARAMS = {
        'serial_port': '/dev/motor',
        'baudrate': 115200,
        'device_id': 127,
        'wheel_separation': 0.27,
        'wheel_radius': 0.065,
        'gear_ratio': 20.0,
        'max_linear_vel': 0.05,
        'max_angular_vel': 0.4,
        'min_rpm': 100.0,
        'max_rpm': 3000.0,
        'control_frequency': 50.0,
        'invert_motor_a': True,
        'invert_motor_b': False,
    }

    def __init__(self):
        super().__init__('hs_motor_controller')

        # 建構子只剩流程骨架：三段拆分後的呼叫順序與抽取前
        # __init__ 內對應程式碼的執行順序完全相同（含 serial 連線時機、
        # pub/sub 建立順序）。
        self._declare_and_load_params()
        self._setup_ros_interfaces()
        self._init_runtime_state()

        self.get_logger().info(
            f'HS Motor Controller initialized on {self.serial_port}'
        )

    def _declare_and_load_params(self) -> None:
        """宣告並載入參數、驗證，再建立運動學計算物件。

        資料驅動：見類別頂部 PARAMS，參數名稱/預設值/型別與屬性名
        皆與抽取前完全一致。
        """
        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)

        # 驗證參數
        self._validate_parameters()

        # 運動學計算（純模組，數值與抽取前完全相同）
        self.kinematics = DifferentialDriveKinematics(
            wheel_separation=self.wheel_separation,
            wheel_radius=self.wheel_radius,
            gear_ratio=self.gear_ratio,
            min_rpm=self.min_rpm,
            max_rpm=self.max_rpm,
            zero_rpm_epsilon=self.ZERO_RPM_EPSILON,
        )

    def _setup_ros_interfaces(self) -> None:
        """建立串口連線與所有 ROS2 pub/sub/service/qos/timer。

        串口連線時機（在任何 pub/sub 之前）與各 pub/sub/service 的
        建立順序皆與抽取前完全相同。
        """
        # 串口連接
        self.serial_conn: Optional[serial.Serial] = None
        self.connect_serial()

        # ROS2 發布者和訂閱者
        qos = self._make_reliable_qos()
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', qos)
        self.voltage_pub = self.create_publisher(Float32, 'motor/voltage', qos)
        self.current_a_pub = self.create_publisher(Float32, 'motor/current_a', qos)
        self.current_b_pub = self.create_publisher(Float32, 'motor/current_b', qos)
        self.fault_pub = self.create_publisher(Int32, 'motor/fault', qos)

        # E-Stop 訂閱 (TRANSIENT_LOCAL 確保收到 latched 狀態；含 e_stop_active 初始化)
        self._setup_e_stop_subscription()

        # 故障清除 service（呼叫後下一包帶 clear_fault=1）
        self.pending_clear_fault = False
        self.clear_fault_srv = self.create_service(
            Trigger, '~/clear_fault', self.clear_fault_callback)

        # 控制定時器
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(control_period, self.control_loop)

        # 安全定時器
        self.safety_timer = self.create_timer(0.1, self.safety_check)

    def _init_runtime_state(self) -> None:
        """初始化馬達/回饋/里程計/安全控制等執行期狀態變數。

        定時器已在 _setup_ros_interfaces 建立但 executor 尚未開始 spin，
        不會在這些狀態變數就緒前被觸發，因此與抽取前「timer 建立在最後」
        相比不影響任何可觀察行為。
        """
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
        """計算 CRC16 校驗碼 (Modbus CRC16)

        委派給 hs_protocol.crc16（純函式，數值與行為完全相同）。
        保留此 method 供既有呼叫端（若有）相容使用。
        """
        return hs_protocol.crc16(data)

    def build_command_packet(self, clear_fault: int = 0) -> bytes:
        """
        建立 HS 協議命令封包 (16 bytes)
        格式: AA + 地址 + 返回類型 + 故障清除 + 保留 + A控制 + B控制 + A方向 + B方向 + A轉速(2B) + B轉速(2B) + 55 + CRC16

        E-Stop / motor_enabled 狀態判斷與共享狀態快照留在此處（Node 業務邏輯），
        純粹的 byte 封裝 + CRC 計算委派給 hs_protocol.encode_command。

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

        return hs_protocol.encode_command(
            device_id=self.device_id,
            clear_fault=clear_fault,
            motor_control_byte=motor_control_byte,
            dir_a=dir_a,
            dir_b=dir_b,
            rpm_a=target_rpm_a,
            rpm_b=target_rpm_b,
        )

    def parse_response_packet(self, data: bytes) -> bool:
        """
        解析 HS 協議回應封包 (16 bytes)
        格式: 55 + 地址 + A電流(2B) + B電流(2B) + A轉速(2B) + B轉速(2B) + 電壓(2B) + 故障 + AA + CRC16

        注意：欄位順序以實測硬體行為為準，如與手冊不符請先驗證再修改。
        """
        if len(data) < hs_protocol.PACKET_LENGTH:
            return False

        # 掃描所有 0x55 候選起始碼，直到找到驗證通過的封包
        # （0x55 可能出現在數據內容中，只試第一個會漏掉緊接在雜訊後的有效封包）
        start_idx = data.find(hs_protocol.START_BYTE_SLAVE)
        while start_idx != -1 and len(data) - start_idx >= hs_protocol.PACKET_LENGTH:
            packet = data[start_idx:start_idx + hs_protocol.PACKET_LENGTH]
            if self._try_parse_packet(packet):
                return True
            start_idx = data.find(hs_protocol.START_BYTE_SLAVE, start_idx + 1)

        return False

    def _try_parse_packet(self, packet: bytes) -> bool:
        """驗證並解析單一 16-byte 候選封包，成功時更新回饋狀態

        byte offset 解析與 CRC 驗證委派給 hs_protocol.decode_packet
        （純函式，數值與驗證順序完全相同），這裡只負責失敗時記 debug log
        與成功時把結果寫回 Node 狀態，與抽取前行為一致。
        """
        result = hs_protocol.decode_packet(packet, self.device_id)
        if not result.ok:
            self.get_logger().debug(result.error)
            return False

        response = result.response
        self.current_a = response.current_a
        self.current_b = response.current_b
        self.actual_rpm_a = response.actual_rpm_a
        self.actual_rpm_b = response.actual_rpm_b
        self.voltage = response.voltage
        self.fault_code = response.fault_code

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
        # 注意：此處的死區過濾是真機回饋特有的邏輯，mock 沒有對應行為，
        # 不納入共用 kinematics；死區判斷後的轉換公式本身才共用。
        motor_rpm_a = self.actual_rpm_a if self.actual_rpm_a > self.RPM_DEADZONE else 0.0
        motor_rpm_b = self.actual_rpm_b if self.actual_rpm_b > self.RPM_DEADZONE else 0.0

        # 馬達 RPM 轉換為輪速 (m/s)
        vel_a = self.kinematics.motor_rpm_to_wheel_vel(motor_rpm_a)
        vel_b = self.kinematics.motor_rpm_to_wheel_vel(motor_rpm_b)

        # 使用鎖保護讀取邏輯方向和更新里程計
        with self.state_lock:
            # 使用邏輯方向 (反轉前的方向) 來決定速度符號
            if motor_rpm_a > 0 and self.logical_dir_a == 1:
                vel_a = -vel_a
            if motor_rpm_b > 0 and self.logical_dir_b == 1:
                vel_b = -vel_b

            # 計算機器人速度 (Motor A = 物理右輪, Motor B = 物理左輪)
            vx, vth = self.kinematics.wheel_vel_to_twist(vel_b, vel_a)

            # 計算時間差
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time

            # 時間差為零或負值時跳過更新（避免除零或時間回跳）
            if dt <= 0:
                return

            # 積分更新位置（不含角度正規化，與抽取前行為一致）
            self.odom_x, self.odom_y, self.odom_theta = self.kinematics.integrate_odometry(
                self.odom_x, self.odom_y, self.odom_theta, vx, vth, dt)

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

        # 驗證輸入值（防止 NaN 或無窮大；共用基底的驗證，訊息與行為與抽取前相同）
        if not self._validate_cmd_vel(msg):
            return

        # 驗證通過後才更新 watchdog 時間，避免無效命令流打穿逾時保護
        self.last_cmd_time = self.get_clock().now()

        # 限制速度
        linear_x = max(min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        angular_z = max(min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)

        # 差動驅動運動學
        left_vel, right_vel = self.kinematics.twist_to_wheel_vel(linear_x, angular_z)

        # 設定馬達 (Motor A = 物理右輪, Motor B = 物理左輪)
        self.set_motor_speeds(left_vel, right_vel)

    def _quantize_rpm(self, motor_rpm: float) -> int:
        """將馬達 RPM 量化到驅動器有效範圍 [min_rpm, max_rpm]

        - 低於 ZERO_RPM_EPSILON 視為零命令 → 0
        - 非零但低於 min_rpm → clamp 到 min_rpm（避免低速死區導致不動）
        - 其餘 clamp 到 max_rpm

        clamp 邏輯委派給 kinematics.quantize_motor_rpm，這裡只負責轉成 int
        （與抽取前 int(max(min(...))) 完全相同）。
        """
        return int(self.kinematics.quantize_motor_rpm(motor_rpm))

    def set_motor_speeds(self, left_vel: float, right_vel: float) -> None:
        """設定馬達速度 (m/s)

        Args:
            left_vel: 左輪目標速度 → Motor B
            right_vel: 右輪目標速度 → Motor A
        """
        # 輪速轉換為馬達 RPM (取絕對值，方向另外處理)
        left_motor_rpm = self.kinematics.wheel_vel_to_motor_rpm(left_vel)
        right_motor_rpm = self.kinematics.wheel_vel_to_motor_rpm(right_vel)

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
        """安全檢查（逾時判斷共用基底，歸零動作與加鎖為真機特有，不共用）"""
        if self._is_cmd_vel_stale():
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
