"""battery_guard — 7S 鋰電低電壓保護節點

訂閱兩路互相獨立的電壓來源，判定警告 / 停機，並在停機時把馬達停下來。

  /motor/voltage (Float32, V)  AGV-BLD-2S 驅動器回報，hs_motor_controller 發布
  /pico/voltage  (Float32, V)  Pico 集線板 INA226 實測；**ok=0 時整組不發布**，
                               所以「topic 靜默」是正常且必須被容忍的情況
    ↓  BatteryPolicy（濾波 → 仲裁 → 持續時間 + 遲滯 → 鎖存，見 battery_policy.py）
  /battery/state (diagnostic_msgs/DiagnosticStatus)  狀態 + 電壓，給 UI / 診斷
  /safety/stop   (std_msgs/Bool, latched)            停機命令，馬達節點訂閱

## 為什麼是 /safety/stop 而不是 /e_stop

/e_stop 是 e_stop_node 專屬的 latched topic：它會在按鈕釋放時發 False。
若本節點也發 /e_stop，兩個 latched publisher 的 True/False 會互相覆蓋——
按鈕按一下再放開，就把低電壓停機一起解除了。因此另開 /safety/stop，
馬達節點兩條線各自訂閱、以 OR 合併。

**本節點只發布 True，永遠不發 False。** 停機鎖存的解除方式是充電後重啟節點。

QoS 與 /e_stop 相同（RELIABLE + TRANSIENT_LOCAL, depth 1）：hs_motor_controller
在 bringup 裡是 respawn=True，重啟後必須立刻收到先前已鎖存的停機命令，
靠 TRANSIENT_LOCAL 的 latched 語意保證；另外每個週期重發一次當作雙保險。
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from std_msgs.msg import Bool, Float32

from motor_control.battery_policy import (
    BatteryPolicy,
    BatteryPolicyConfig,
    SOURCE_MOTOR,
    SOURCE_PICO,
    STATE_OK,
    STATE_SHUTDOWN,
    STATE_UNKNOWN,
    STATE_WARNING,
)

MOTOR_VOLTAGE_TOPIC = '/motor/voltage'
PICO_VOLTAGE_TOPIC = '/pico/voltage'
BATTERY_STATE_TOPIC = '/battery/state'
SAFETY_STOP_TOPIC = '/safety/stop'

# 狀態 → DiagnosticStatus.level。UNKNOWN 用 STALE，語意剛好是「資料過期」。
_STATE_TO_LEVEL = {
    STATE_OK: DiagnosticStatus.OK,
    STATE_WARNING: DiagnosticStatus.WARN,
    STATE_SHUTDOWN: DiagnosticStatus.ERROR,
    STATE_UNKNOWN: DiagnosticStatus.STALE,
}


class BatteryGuardNode(Node):
    """低電壓保護節點。

    判定邏輯全在 motor_control.battery_policy（純模組、離線可測），
    這裡只負責訂閱、取時鐘、發布與 log。
    """

    # 參數宣告表：{參數名: 預設值}，預設值與 config/battery_guard.yaml 一致。
    # 參數名與載入後的屬性名（self.<name>）完全一致，見 _load_params。
    PARAMS = {
        'warn_voltage': 22.4,
        'shutdown_voltage': 21.7,
        'hysteresis_voltage': 0.3,
        'warn_duration_sec': 2.0,
        'shutdown_duration_sec': 3.0,
        'filter_window_sec': 1.0,
        'source_timeout_sec': 2.0,
        'min_valid_voltage': 5.0,
        'max_valid_voltage': 60.0,
        'publish_rate_hz': 2.0,
        'log_throttle_sec': 10.0,
    }

    def __init__(self):
        super().__init__('battery_guard')

        self._load_params()
        self._setup_ros_interfaces()

        self._last_state = None

        self.get_logger().info(
            f'battery_guard 啟動：警告 {self.warn_voltage}V / '
            f'停機 {self.shutdown_voltage}V（持續 {self.shutdown_duration_sec:.0f}s、'
            f'遲滯 {self.hysteresis_voltage}V）'
        )
        self.get_logger().info(
            f'  電壓來源: {MOTOR_VOLTAGE_TOPIC}, {PICO_VOLTAGE_TOPIC}'
            f'（取新鮮來源的最小值；來源逾時 {self.source_timeout_sec:.1f}s）'
        )
        self.get_logger().info(
            f'  停機命令: {SAFETY_STOP_TOPIC}（latched，只發 True）'
        )

    def _load_params(self) -> None:
        """宣告並載入參數，交給 BatteryPolicyConfig.validate 做一致性檢查。"""
        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)

        if self.publish_rate_hz <= 0:
            raise ValueError(
                f'publish_rate_hz must be positive, got {self.publish_rate_hz}')

        config = BatteryPolicyConfig(
            warn_voltage=self.warn_voltage,
            shutdown_voltage=self.shutdown_voltage,
            hysteresis_voltage=self.hysteresis_voltage,
            warn_duration_sec=self.warn_duration_sec,
            shutdown_duration_sec=self.shutdown_duration_sec,
            filter_window_sec=self.filter_window_sec,
            source_timeout_sec=self.source_timeout_sec,
            min_valid_voltage=self.min_valid_voltage,
            max_valid_voltage=self.max_valid_voltage,
        )
        # 參數不自洽時直接拋（訊息由 validate 組），不要帶著壞門檻上線
        try:
            self.policy = BatteryPolicy(config)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            raise

    def _setup_ros_interfaces(self) -> None:
        # 兩路電壓來源都是 RELIABLE/VOLATILE depth 10（見 hs_motor_controller
        # 的 _make_reliable_qos 與 pico publishers 的 QoSProfile(depth=10)）
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.motor_voltage_sub = self.create_subscription(
            Float32, MOTOR_VOLTAGE_TOPIC, self._on_motor_voltage, sensor_qos)
        self.pico_voltage_sub = self.create_subscription(
            Float32, PICO_VOLTAGE_TOPIC, self._on_pico_voltage, sensor_qos)

        # latched：晚加入的訂閱者（前端、respawn 後的馬達節點）立刻拿到現況
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.state_pub = self.create_publisher(
            DiagnosticStatus, BATTERY_STATE_TOPIC, latched_qos)
        self.stop_pub = self.create_publisher(
            Bool, SAFETY_STOP_TOPIC, latched_qos)

        period = 1.0 / self.publish_rate_hz
        self.eval_timer = self.create_timer(period, self._tick)

    # ------------------------------------------------------------------
    # 訂閱回調
    # ------------------------------------------------------------------

    def _now_sec(self) -> float:
        """統一走 ROS 時鐘，讓 use_sim_time 與離線回放也能正確計時。"""
        return self.get_clock().now().nanoseconds / 1e9

    def _on_motor_voltage(self, msg: Float32) -> None:
        self._submit(SOURCE_MOTOR, msg.data, MOTOR_VOLTAGE_TOPIC)

    def _on_pico_voltage(self, msg: Float32) -> None:
        self._submit(SOURCE_PICO, msg.data, PICO_VOLTAGE_TOPIC)

    def _submit(self, source: str, value: float, topic: str) -> None:
        if not self.policy.submit(source, value, self._now_sec()):
            # 無效值不能靜默丟掉：讀值長期不合理代表某一路壞了，
            # 而少一路會讓保護退化成單源，必須看得見。
            self.get_logger().warning(
                f'{topic} 讀值 {value} 不在有效範圍 '
                f'[{self.min_valid_voltage}, {self.max_valid_voltage}]V，已忽略'
                f'（累計 {self.policy.rejected_count(source)} 筆）',
                throttle_duration_sec=self.log_throttle_sec)

    # ------------------------------------------------------------------
    # 週期判定與發布
    # ------------------------------------------------------------------

    def _tick(self) -> None:
        decision = self.policy.evaluate(self._now_sec())
        self._publish_state(decision)

        # 只發 True，且鎖存後每個週期重發（TRANSIENT_LOCAL 之外的雙保險）
        if decision.stop_latched:
            self.stop_pub.publish(Bool(data=True))

        self._log_decision(decision)

    def _publish_state(self, decision) -> None:
        msg = DiagnosticStatus()
        msg.level = _STATE_TO_LEVEL[decision.state]
        msg.name = 'battery_guard'
        msg.message = decision.reason
        msg.hardware_id = '7S_LIION'

        values = [
            KeyValue(key='state', value=decision.state),
            KeyValue(key='voltage',
                     value='' if decision.voltage is None
                           else f'{decision.voltage:.3f}'),
            KeyValue(key='sources_used', value=','.join(decision.sources_used)),
            KeyValue(key='stop_latched', value=str(decision.stop_latched).lower()),
            KeyValue(key='warn_voltage', value=f'{self.warn_voltage:.2f}'),
            KeyValue(key='shutdown_voltage', value=f'{self.shutdown_voltage:.2f}'),
        ]
        # 各來源濾波後的值分開列出，方便現場判斷是哪一路在拉低仲裁值
        for source, voltage in decision.source_voltages.items():
            values.append(KeyValue(
                key=f'voltage_{source}',
                value='' if voltage is None else f'{voltage:.3f}'))
        msg.values = values

        self.state_pub.publish(msg)

    def _log_decision(self, decision) -> None:
        """狀態變化時各 log 一次，停留在警告/停機時再週期提醒（throttled）。"""
        changed = decision.state != self._last_state
        self._last_state = decision.state

        if decision.state == STATE_SHUTDOWN:
            if changed:
                self.get_logger().error(
                    f'🔴 低電壓停機：{decision.reason}。'
                    f'{SAFETY_STOP_TOPIC} 已發出，馬達將制動；'
                    f'充電後重啟 battery_guard 才會解除。')
            else:
                self.get_logger().error(
                    f'低電壓停機鎖存中：{decision.reason}',
                    throttle_duration_sec=self.log_throttle_sec)
        elif decision.state == STATE_WARNING:
            if changed:
                self.get_logger().warning(f'⚠️  低電壓警告：{decision.reason}')
            else:
                self.get_logger().warning(
                    f'低電壓警告持續：{decision.reason}',
                    throttle_duration_sec=self.log_throttle_sec)
        elif decision.state == STATE_UNKNOWN:
            self.get_logger().warning(
                decision.reason, throttle_duration_sec=self.log_throttle_sec)
        elif changed:
            self.get_logger().info(f'電池狀態正常：{decision.reason}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BatteryGuardNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
