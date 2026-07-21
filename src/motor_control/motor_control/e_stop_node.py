"""
E-Stop GPIO 監控節點

監控硬體急停按鈕的 GPIO 狀態，發布 /e_stop topic。
按鈕使用 NC（常閉）觸點接 GPIO→GND，按下鎖定時 GPIO = LOW = 急停啟動。

非對稱去彈跳策略：
  - 啟動方向（安全優先）：立即響應
  - 釋放方向：50ms 去彈跳，避免誤解除
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool

# 嘗試載入 Jetson GPIO，不可用時降級為模擬模式
try:
    import Jetson.GPIO as GPIO
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


class EStopNode(Node):
    """E-Stop GPIO 監控節點"""

    def __init__(self):
        super().__init__('e_stop_node')

        # 宣告參數
        self.declare_parameter('gpio_pin', 7)        # BOARD 模式 pin 號
        self.declare_parameter('active_low', True)    # LOW = 急停啟動
        self.declare_parameter('poll_rate', 100.0)    # 輪詢頻率 (Hz)
        self.declare_parameter('debounce_ms', 50)     # 釋放方向去彈跳 (ms)
        self.declare_parameter('simulation', False)   # 強制模擬模式

        # 獲取參數
        self.gpio_pin = self.get_parameter('gpio_pin').value
        self.active_low = self.get_parameter('active_low').value
        self.poll_rate = self.get_parameter('poll_rate').value
        self.debounce_ms = self.get_parameter('debounce_ms').value
        force_simulation = self.get_parameter('simulation').value

        # 決定是否使用模擬模式
        self.simulation = force_simulation or not _GPIO_AVAILABLE
        if self.simulation and not force_simulation:
            self.get_logger().warning(
                'Jetson.GPIO not available, falling back to simulation mode'
            )

        # 發布者 - 使用 TRANSIENT_LOCAL 確保 latched 語意
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.e_stop_pub = self.create_publisher(Bool, '/e_stop', qos)

        # E-Stop 狀態
        self.e_stop_active = False
        self._release_stable_count = 0
        self._debounce_samples = max(1, int(self.debounce_ms * self.poll_rate / 1000.0))

        # 初始化 GPIO
        if not self.simulation:
            self._setup_gpio()

        # 先讀取 GPIO 真實狀態再發布初始值
        # （避免按鈕已鎖定時，latched topic 先出現短暫的 False）
        # simulation 模式 _read_gpio 固定回傳 False
        self.e_stop_active = self._read_gpio()

        # 發布初始狀態
        self._publish_state()

        # 輪詢定時器
        poll_period = 1.0 / self.poll_rate
        self.poll_timer = self.create_timer(poll_period, self._poll_gpio)

        mode_str = 'SIMULATION' if self.simulation else f'GPIO pin {self.gpio_pin}'
        self.get_logger().info(
            f'E-Stop node initialized ({mode_str}), '
            f'active_low={self.active_low}, '
            f'debounce={self.debounce_ms}ms ({self._debounce_samples} samples)'
        )

    def _setup_gpio(self):
        """設定 GPIO（BOARD 模式 + 內部上拉）"""
        try:
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(self.gpio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self.get_logger().info(f'GPIO {self.gpio_pin} configured (BOARD, PUD_UP)')
        except Exception as e:
            self.get_logger().error(f'Failed to setup GPIO: {e}, falling back to simulation')
            self.simulation = True

    def _read_gpio(self) -> bool:
        """讀取 GPIO 並返回是否急停啟動

        Returns:
            True = 急停啟動, False = 正常
        """
        if self.simulation:
            return False  # 模擬模式永遠不觸發

        try:
            pin_state = GPIO.input(self.gpio_pin)
            # active_low: LOW (0) = 急停啟動
            if self.active_low:
                return pin_state == GPIO.LOW
            else:
                return pin_state == GPIO.HIGH
        except Exception as e:
            self.get_logger().error(f'GPIO read error: {e}')
            # 讀取失敗時保持當前狀態（fail-safe: 不意外解除急停）
            return self.e_stop_active

    def _poll_gpio(self):
        """輪詢 GPIO 狀態，非對稱去彈跳"""
        raw_active = self._read_gpio()

        if raw_active and not self.e_stop_active:
            # 啟動方向：立即響應（安全優先）
            self.e_stop_active = True
            self._release_stable_count = 0
            self._publish_state()
            self.get_logger().warn('E-STOP ACTIVATED')

        elif not raw_active and self.e_stop_active:
            # 釋放方向：去彈跳
            self._release_stable_count += 1
            if self._release_stable_count >= self._debounce_samples:
                self.e_stop_active = False
                self._release_stable_count = 0
                self._publish_state()
                self.get_logger().info('E-Stop released')

        elif raw_active and self.e_stop_active:
            # 仍在急停中，重置釋放計數
            self._release_stable_count = 0

    def _publish_state(self):
        """發布當前 E-Stop 狀態"""
        msg = Bool()
        msg.data = self.e_stop_active
        self.e_stop_pub.publish(msg)

    def destroy_node(self):
        """節點銷毀時清理 GPIO"""
        if not self.simulation and _GPIO_AVAILABLE:
            try:
                GPIO.cleanup(self.gpio_pin)
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EStopNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
