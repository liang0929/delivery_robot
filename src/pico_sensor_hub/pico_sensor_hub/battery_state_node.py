"""battery_state — 把 Pico 的裸電源值打包成標準 `sensor_msgs/BatteryState`。

  /pico/voltage (Float32, V)   ─┐
  /pico/current (Float32, A)   ─┤ BatteryStateAdapter（反號 + 逾時退化）
                                ↓
  battery_state (sensor_msgs/BatteryState)

存在的理由：opennav_docking 判定「已在充電」只認標準 BatteryState
（`SimpleChargingDock`，humble 分支 `src/simple_charging_dock.cpp:107-111`），
而本車的電量資訊只有 Pico 裸值。轉換規則與符號慣例見
``battery_state_adapter`` 的模組說明——**電流必須反號**。

## 為什麼是週期發布，不是「收到才發」

消費端只在收到訊息時更新自己的狀態（上述 lambda 就是全部邏輯）。
來源死掉時若我們跟著靜默，對面會凍結在最後一筆——剛好停在「充電中」
就再也不會離開 dock。所以這裡改成固定頻率發布：來源新鮮就送實值，
逾時就送 NaN + ``present=False``，讓「不知道」也是一則會抵達的訊息。

## QoS

發布端 RELIABLE / VOLATILE / KeepLast(10)。訂閱端
``create_subscription<BatteryState>("battery_state", 1, ...)``
（`simple_charging_dock.cpp:107-108`）用的是 rclcpp 預設 QoS，即
RELIABLE / VOLATILE / KeepLast(1)，兩邊相容（depth 不影響相容性）。
不用 TRANSIENT_LOCAL：週期發布本來就會讓晚加入的訂閱者在半秒內拿到現況，
而 latched 的舊值在這裡是有害的——重連瞬間先收到一筆過期的「充電中」。
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32

from pico_sensor_hub.battery_state_adapter import (
    BatteryStateAdapter,
    SOURCE_CURRENT,
    SOURCE_VOLTAGE,
    STATUS_CHARGING,
    STATUS_DISCHARGING,
    STATUS_NOT_CHARGING,
    STATUS_UNKNOWN,
)
from pico_sensor_hub.publishers import CURRENT_TOPIC, VOLTAGE_TOPIC

BATTERY_STATE_TOPIC = 'battery_state'

#: adapter 的中介狀態 → msg 常數。
_STATUS_TO_MSG = {
    STATUS_CHARGING: BatteryState.POWER_SUPPLY_STATUS_CHARGING,
    STATUS_DISCHARGING: BatteryState.POWER_SUPPLY_STATUS_DISCHARGING,
    STATUS_NOT_CHARGING: BatteryState.POWER_SUPPLY_STATUS_NOT_CHARGING,
    STATUS_UNKNOWN: BatteryState.POWER_SUPPLY_STATUS_UNKNOWN,
}


def build_battery_state(reading, cell_count=0, stamp=None):
    """把一筆 BatteryReading 填成 BatteryState msg。

    量不到的欄位一律照 msg 註解填 NaN / ``*_UNKNOWN``，不拿電壓去反推。
    尤其 ``percentage``：本車的電量百分比換算在 robot_api_server 那側
    （``conversions.voltage_to_battery``），在這裡再算一份就會出現兩個版本
    的電量互相打架。

    ``cell_voltage`` / ``cell_temperature`` 依 msg 註解——電芯數已知但個別值
    未知時，填出對應長度的 NaN 陣列，讓消費端至少知道這是幾串電池。
    """
    msg = BatteryState()
    if stamp is not None:
        msg.header.stamp = stamp
    # frame_id 留空：電池不是有位姿的感測器，填 base_link 只會誤導 TF 使用者。

    msg.voltage = float(reading.voltage)
    msg.current = float(reading.current)
    msg.present = bool(reading.present)

    msg.temperature = math.nan       # INA226 不量溫度
    msg.charge = math.nan            # 沒有庫侖計，積不出來
    msg.capacity = math.nan
    msg.design_capacity = math.nan
    msg.percentage = math.nan

    msg.power_supply_status = _STATUS_TO_MSG[reading.status]
    msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
    # 7S 18650 鋰離子（docs/power_design.md §2「7S 18650 Li-ion，標稱 25.9V」），
    # 所以是 LION 不是 LIPO。battery_guard 的 hardware_id='6S_LIPO' 與文件、
    # 與 robot_api_server/config.py:54 的 7S 換算都對不上，是既有筆誤；
    # 本單禁區不動 battery_guard，已列入報告的殘留問題。
    msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION

    msg.cell_voltage = [math.nan] * int(cell_count)
    msg.cell_temperature = [math.nan] * int(cell_count)
    msg.location = ''
    msg.serial_number = ''
    return msg


class BatteryStateNode(Node):
    """訂 Pico 裸值、發標準 BatteryState。"""

    # 參數名與載入後的屬性名一致（比照 battery_guard 的 PARAMS 表）。
    PARAMS = {
        # 發布頻率。與 battery_guard 的 publish_rate_hz 取同值：兩者都是
        # 「週期回報現況」的節點，同頻率讓現場 log 對得起來。
        'publish_rate_hz': 2.0,
        # 多久沒收到來源就退化成 NaN + present=False。Pico $PW 是 10 Hz，
        # 5 秒等於連掉 50 幀，不可能是抖動；同時對齊 robot_api_server 的
        # battery_state_timeout_sec，整條鏈的「失聯」定義一致。
        # 設為 0 或負值＝停用逾時判定（保留最後一筆，即無退化行為）。
        'source_timeout_sec': 5.0,
        # 有效讀值範圍，與 battery_guard.yaml 同值。超出範圍者計數後丟棄。
        'min_valid_voltage': 5.0,
        'max_valid_voltage': 60.0,
        # 電流絕對值上限。本車馬達滿載約十幾安培，100 A 只用來擋
        # 明顯的解析錯誤 / 感測器爆走，不是運轉上限。
        'max_valid_current_abs': 100.0,
        # 判 charging / discharging 的死區門檻（A，反號後）。預設對齊
        # opennav_docking 的 charging_threshold（simple_charging_dock.cpp:59）。
        'charging_current_threshold': 0.5,
        # 電芯數，只用來決定 cell_voltage / cell_temperature 的長度。0＝不填。
        # 7S（docs/power_design.md §2）。
        'cell_count': 7,
        'log_throttle_sec': 10.0,
    }

    def __init__(self):
        super().__init__('pico_battery_state')

        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)

        if self.publish_rate_hz <= 0:
            raise ValueError(
                f'publish_rate_hz must be positive, got {self.publish_rate_hz}')

        self.adapter = BatteryStateAdapter(
            source_timeout_sec=self.source_timeout_sec,
            min_valid_voltage=self.min_valid_voltage,
            max_valid_voltage=self.max_valid_voltage,
            max_valid_current_abs=self.max_valid_current_abs,
            charging_current_threshold=self.charging_current_threshold,
        )
        self._last_present = None

        self._setup_ros_interfaces()

        self.get_logger().info('battery_state adapter 啟動')
        self.get_logger().info(
            f'  來源: {VOLTAGE_TOPIC}, {CURRENT_TOPIC}'
            f'（逾時 {self.source_timeout_sec:.1f}s 後退回 NaN/present=False）')
        self.get_logger().info(
            f'  發布: {BATTERY_STATE_TOPIC} @ {self.publish_rate_hz:.1f}Hz'
            f'（電流已反號為 BatteryState 慣例：充電為正）')

    def _setup_ros_interfaces(self):
        # 與 pico publishers 的 QoSProfile(depth=10) 相同（RELIABLE/VOLATILE），
        # 也與 battery_guard 訂 /pico/voltage 用的 profile 一致。
        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.voltage_sub = self.create_subscription(
            Float32, VOLTAGE_TOPIC, self._on_voltage, sensor_qos)
        self.current_sub = self.create_subscription(
            Float32, CURRENT_TOPIC, self._on_current, sensor_qos)

        # 發布端 QoS 的取捨見模組說明。
        self.state_pub = self.create_publisher(
            BatteryState, BATTERY_STATE_TOPIC, sensor_qos)

        self.publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz, self._tick)

    # ------------------------------------------------------------------
    # 時鐘
    # ------------------------------------------------------------------

    def _now(self):
        """逾時判定用單調時鐘。

        量的是「距上次收訊多久」，不能被 NTP 校時拉歪（Jetson 開機後校時
        會讓牆上時間跳幾秒，剛好把新鮮資料判成過期）。收訊時戳與判定時鐘
        同源即可，兩邊都在這個函式裡。header.stamp 另外走 ROS 時鐘，那是
        對外的時間標記，語意不同。
        """
        return time.monotonic()

    # ------------------------------------------------------------------
    # 訂閱回調
    # ------------------------------------------------------------------

    def _on_voltage(self, msg):
        if not self.adapter.submit_voltage(msg.data, self._now()):
            self._warn_rejected(VOLTAGE_TOPIC, msg.data, SOURCE_VOLTAGE)

    def _on_current(self, msg):
        if not self.adapter.submit_current(msg.data, self._now()):
            self._warn_rejected(CURRENT_TOPIC, msg.data, SOURCE_CURRENT)

    def _warn_rejected(self, topic, value, source):
        """無效讀值不靜默丟：長期無效代表 INA226 或韌體出事，必須看得見。"""
        self.get_logger().warning(
            f'{topic} 讀值 {value} 無效，已忽略'
            f'（累計 {self.adapter.rejected_count(source)} 筆）',
            throttle_duration_sec=self.log_throttle_sec)

    # ------------------------------------------------------------------
    # 週期發布
    # ------------------------------------------------------------------

    def _tick(self):
        reading = self.adapter.snapshot(self._now())
        msg = build_battery_state(
            reading,
            cell_count=self.cell_count,
            stamp=self.get_clock().now().to_msg(),
        )
        self.state_pub.publish(msg)
        self._log_presence(reading)

    def _log_presence(self, reading):
        """來源在線 / 失聯的邊緣各 log 一次，穩態不洗版。"""
        if reading.present == self._last_present:
            return
        if self._last_present is None and not reading.present:
            # 啟動後還沒收到第一筆，這是正常的開機過渡，不當成失聯事件
            self._last_present = reading.present
            return
        if reading.present:
            self.get_logger().info(
                f'電源來源已在線，{BATTERY_STATE_TOPIC} 開始送實值')
        else:
            self.get_logger().warning(
                f'{VOLTAGE_TOPIC} / {CURRENT_TOPIC} 已逾時 '
                f'{self.source_timeout_sec:.1f}s，'
                f'{BATTERY_STATE_TOPIC} 退回 NaN / present=False')
        self._last_present = reading.present


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = BatteryStateNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
