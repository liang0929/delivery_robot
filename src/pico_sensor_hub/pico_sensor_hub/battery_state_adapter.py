"""`/pico/voltage` + `/pico/current` → `sensor_msgs/BatteryState` 的判定邏輯。

刻意**不 import 任何 ROS 模組**：這一層是純資料轉換，離線就能測完
（反號、逾時退化、無效值），不必起節點也不必 rclpy.init。

## 這層存在的唯一理由：電流符號慣例是相反的

| 來源 | 慣例 | 出處 |
| --- | --- | --- |
| Pico `$PW` / `/pico/current` | **放電為正、充電為負** | `protocol.py:154`、韌體 README 5.2 |
| `sensor_msgs/BatteryState.current` | **充電為正**（"Negative when discharging"） | msg 定義 |

opennav_docking 的 SimpleChargingDock 直接拿這個欄位判定充電中：
``is_charging_ = state->current > charging_threshold_``（humble 分支
`opennav_docking/src/simple_charging_dock.cpp:110`，門檻預設 0.5 A）。
弄反的後果不是數字難看，是 dock 流程**永遠判定沒在充電**而卡死，
或反過來一貼上就宣告充飽。因此反號是本模組的核心不變量，測試直接盯它。

## 逾時退化

Pico 在 INA226 讀取無效（`$PW ok=0`）時**整組不發布**（見
`pico_sensor_node._on_pw` 的取捨說明），節點自己掛掉也一樣是靜默。
兩者對下游都表現為「topic 不再更新」，所以「多久沒收到」是這裡唯一
可靠的訊號。逾時後必須主動把值換成 NaN 並 ``present=False``：
BatteryState 的消費端只在收到訊息時更新自己的狀態，我們若停止發布或
繼續重發最後一筆，對面就會凍結在「充電中」——比沒有資料更危險。

NaN 在這裡是安全的退化值而不是雷：``NaN > threshold`` 恆為 False，
對面自然落回「沒在充電」。
"""

import math
from dataclasses import dataclass

#: power_supply_status 的中介表示。用字串而不是 msg 常數，是為了讓本模組
#: 與 ROS 完全脫鉤；對映到 ``BatteryState.POWER_SUPPLY_STATUS_*`` 在節點層做。
STATUS_CHARGING = 'charging'
STATUS_DISCHARGING = 'discharging'
STATUS_NOT_CHARGING = 'not_charging'
STATUS_UNKNOWN = 'unknown'

#: 來源標籤，給 ``rejected_count`` 與 log 用。
SOURCE_VOLTAGE = 'voltage'
SOURCE_CURRENT = 'current'


@dataclass(frozen=True)
class BatteryReading:
    """一次快照。欄位語意已經是 BatteryState 的語意，不是 Pico 的。"""

    #: V。來源逾時或從未收過為 NaN。
    voltage: float
    #: A，**已反號成「充電為正」**。來源逾時或從未收過為 NaN。
    current: float
    #: 至少還有一路來源是新鮮的。兩路都失聯時為 False。
    present: bool
    #: STATUS_* 之一。
    status: str


class BatteryStateAdapter:
    """收裸值、吐 BatteryReading。時間由呼叫端傳入，本身不看時鐘。

    時鐘外部注入有兩個好處：測試能瞬間跳過 timeout 不必真的 sleep；
    節點層可以自由決定用 ``time.monotonic()`` 還是 ROS 時鐘，這裡不預設立場
    （節點目前用 monotonic，理由見 ``battery_state_node``）。
    """

    def __init__(self, source_timeout_sec=5.0,
                 min_valid_voltage=5.0, max_valid_voltage=60.0,
                 max_valid_current_abs=100.0,
                 charging_current_threshold=0.5):
        self.source_timeout_sec = float(source_timeout_sec)
        self.min_valid_voltage = float(min_valid_voltage)
        self.max_valid_voltage = float(max_valid_voltage)
        self.max_valid_current_abs = float(max_valid_current_abs)
        self.charging_current_threshold = float(charging_current_threshold)

        self._voltage = math.nan
        self._current = math.nan          # 原始 Pico 慣例（放電為正），未反號
        self._voltage_rx_at = None
        self._current_rx_at = None
        self._rejected = {SOURCE_VOLTAGE: 0, SOURCE_CURRENT: 0}

    # ------------------------------------------------------------------
    # 收值
    # ------------------------------------------------------------------

    def submit_voltage(self, value, now):
        """收一筆電壓（V）。無效值回 False 且**不更新時戳**。

        不更新時戳是刻意的：讀值一直不合理等同這一路已經壞了，讓它照常逾時
        退化成 NaN，比帶著垃圾值裝作健在好。
        """
        value = float(value)
        if not self._is_valid_voltage(value):
            self._rejected[SOURCE_VOLTAGE] += 1
            return False
        self._voltage = value
        self._voltage_rx_at = now
        return True

    def submit_current(self, value, now):
        """收一筆電流（A，**Pico 慣例：放電為正**）。無效值回 False。"""
        value = float(value)
        if not self._is_valid_current(value):
            self._rejected[SOURCE_CURRENT] += 1
            return False
        self._current = value
        self._current_rx_at = now
        return True

    def rejected_count(self, source):
        return self._rejected[source]

    # ------------------------------------------------------------------
    # 取快照
    # ------------------------------------------------------------------

    def snapshot(self, now):
        """組出當下的 BatteryReading（含反號與逾時退化）。"""
        voltage_fresh = self._is_fresh(self._voltage_rx_at, now)
        current_fresh = self._is_fresh(self._current_rx_at, now)

        voltage = self._voltage if voltage_fresh else math.nan
        # 反號：Pico 放電為正 → BatteryState 充電為正。
        # 寫成 `0.0 - x` 而不是 `-x`，是為了讓零電流輸出 +0.0 而非 -0.0；
        # 兩者數值相等，但 `ros2 topic echo` 印出 -0.0 會讓人以為在放電。
        current = (0.0 - self._current) if current_fresh else math.nan

        return BatteryReading(
            voltage=voltage,
            current=current,
            # 任一路還在就算電池在場：兩路同源（同一筆 $PW），只有一路過期
            # 多半是丟包而不是電池被拔走，這時把 present 打成 False 反而是假警報。
            present=voltage_fresh or current_fresh,
            status=self._status(current, current_fresh),
        )

    def _status(self, current, current_fresh):
        """依**反號後**的電流判 power_supply_status。

        門檻與 opennav_docking 的 ``charging_threshold`` 同一個預設值（0.5 A），
        兩邊才不會出現「這裡說充電中、dock 說沒有」的分歧。死區內回
        NOT_CHARGING：接著充電器但還沒真的灌電流就是這個狀態，msg 常數
        本來就是為此而設。
        """
        if not current_fresh:
            return STATUS_UNKNOWN
        if current > self.charging_current_threshold:
            return STATUS_CHARGING
        if current < -self.charging_current_threshold:
            return STATUS_DISCHARGING
        return STATUS_NOT_CHARGING

    def _is_fresh(self, rx_at, now):
        """收訊時戳還在有效期內嗎。

        ``source_timeout_sec <= 0`` ＝停用逾時判定（現場緊急逃生門，行為退回
        「收過就一直用最後一筆」）；從未收過（``rx_at is None``）永遠不新鮮，
        與時鐘無關。
        """
        if rx_at is None:
            return False
        if self.source_timeout_sec <= 0:
            return True
        return (now - rx_at) <= self.source_timeout_sec

    def _is_valid_voltage(self, value):
        # isfinite 一次擋掉 NaN 與 ±Inf：NaN 進了比較式會恆為 False，
        # 安靜地繞過所有門檻檢查，是最難查的那種壞。
        return (math.isfinite(value)
                and self.min_valid_voltage <= value <= self.max_valid_voltage)

    def _is_valid_current(self, value):
        return math.isfinite(value) and abs(value) <= self.max_valid_current_abs
