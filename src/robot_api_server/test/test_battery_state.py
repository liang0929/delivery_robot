"""`/battery/state` 消費端的離線測試（不需要 ROS 執行期，也不起節點）。

涵蓋三件事：

1. ``RosBridge._on_battery_state`` 的解析——這是唯一把上游 DiagnosticStatus
   翻成 API 語意的地方，翻錯的後果是「電池已鎖存停機，UI 卻顯示正常」。
2. 過期（staleness）判定——battery_guard 掛掉後不能停在最後一次的 ``ok``，
   否則操作者以為低電壓保護還在線，實際上早就沒了。
3. ``robot_info`` 推播負載真的帶上新欄位，且舊欄位一個都沒少（前端唯一的
   即時通道，欄位掉了等於整個 UI 空掉）。

msg 用簡單的假物件而非真的 ``diagnostic_msgs``：本模組只讀 ``level`` 與
``values``（key/value 字串），不依賴訊息型別本身，假物件反而讓「欄位缺席」
這類邊界情境好寫。
"""

from types import SimpleNamespace

import pytest

from robot_api_server.bridge_node import RosBridge
from robot_api_server.config import BATTERY_STATE_TIMEOUT_SEC
from robot_api_server.models import (
    BatteryState, Location, OpMode, RobotInfo, RobotStatus,
)
from robot_api_server.ws_server import EventHub

#: 測試用的過期門檻。刻意不吃 config 的實際值（可被環境變數改），
#: 邊界斷言才不會隨環境飄移。
TIMEOUT = 5.0


class FakeClock:
    """可注入的單調時鐘。

    過期判定是時間相關邏輯，但測試不該真的 sleep 5 秒——把時鐘抽成可注入
    的 callable，時間就變成測試可以直接操縱的輸入。
    """

    def __init__(self, now: float = 1000.0):
        self._now = float(now)

    def __call__(self) -> float:
        return self._now

    def advance(self, seconds: float) -> None:
        self._now += seconds


def make_status(level=0, **values):
    """組一個「長得像 DiagnosticStatus」的假訊息。"""
    return SimpleNamespace(
        level=level,
        values=[SimpleNamespace(key=k, value=v) for k, v in values.items()],
    )


@pytest.fixture
def clock():
    return FakeClock()


@pytest.fixture
def bridge(clock):
    """只建物件不 start()——解析邏輯與 ROS 執行期無關。

    時鐘不自動前進，所以解析類測試一律落在「剛收到訊息」的新鮮狀態。
    """
    return RosBridge(battery_state_timeout_sec=TIMEOUT, clock=clock)


# ---------------------------------------------------------------------
# 預設值：沒有 battery_guard 時的語意
# ---------------------------------------------------------------------

def test_defaults_to_unknown_before_any_message(bridge):
    """沒收到訊息＝沒有 battery_guard，語意必須是 UNKNOWN 而不是 OK。"""
    assert bridge.battery_state() == BatteryState.UNKNOWN
    assert bridge.battery_stop_latched() is False
    assert bridge.battery_guard_voltage() is None


# ---------------------------------------------------------------------
# 正常解析：以 state KeyValue 為準
# ---------------------------------------------------------------------

@pytest.mark.parametrize('raw, expected', [
    ('OK', BatteryState.OK),
    ('WARNING', BatteryState.WARNING),
    ('SHUTDOWN', BatteryState.SHUTDOWN),
    ('UNKNOWN', BatteryState.UNKNOWN),
])
def test_state_keyvalue_maps_to_api_value(bridge, raw, expected):
    bridge._on_battery_state(make_status(state=raw, stop_latched='false'))
    assert bridge.battery_state() == expected


def test_parses_stop_latched_and_voltage(bridge):
    bridge._on_battery_state(make_status(
        level=1, state='WARNING', stop_latched='true', voltage='22.150'))
    assert bridge.battery_state() == BatteryState.WARNING
    assert bridge.battery_stop_latched() is True
    assert bridge.battery_guard_voltage() == pytest.approx(22.15)


def test_stop_latched_false_string_is_not_truthy(bridge):
    """'false' 是非空字串，若用 bool() 判斷會變成 True——這是必須擋住的陷阱。"""
    bridge._on_battery_state(make_status(state='OK', stop_latched='false'))
    assert bridge.battery_stop_latched() is False


def test_shutdown_implies_latched_even_if_flag_missing(bridge):
    """上游漏填 stop_latched 時不能讓 UI 少一半資訊：SHUTDOWN 本身即蘊含鎖存。"""
    bridge._on_battery_state(make_status(level=2, state='SHUTDOWN'))
    assert bridge.battery_stop_latched() is True


def test_state_takes_priority_over_level(bridge):
    """兩者衝突時以 state 字串為準（一手判定結果）。"""
    bridge._on_battery_state(make_status(level=0, state='SHUTDOWN'))
    assert bridge.battery_state() == BatteryState.SHUTDOWN


# ---------------------------------------------------------------------
# 退化輸入：不得例外，也不得誤判成 OK
# ---------------------------------------------------------------------

@pytest.mark.parametrize('level, expected', [
    (0, BatteryState.OK),
    (1, BatteryState.WARNING),
    (2, BatteryState.SHUTDOWN),
    (3, BatteryState.UNKNOWN),
])
def test_falls_back_to_level_when_state_missing(bridge, level, expected):
    bridge._on_battery_state(make_status(level=level))
    assert bridge.battery_state() == expected


def test_unrecognised_state_becomes_unknown_not_ok(bridge):
    """上游若新增狀態，寧可顯示未知，也不要猜成 ok 讓操作者以為電池沒事。"""
    bridge._on_battery_state(make_status(level=0, state='DEGRADED'))
    assert bridge.battery_state() == BatteryState.UNKNOWN


def test_unparsable_voltage_is_dropped_without_raising(bridge):
    bridge._on_battery_state(make_status(state='OK', voltage='n/a'))
    assert bridge.battery_state() == BatteryState.OK
    assert bridge.battery_guard_voltage() is None


def test_empty_voltage_string_means_no_reading(bridge):
    """battery_guard 在沒有有效電壓時送空字串（見 _publish_state）。"""
    bridge._on_battery_state(make_status(level=3, state='UNKNOWN', voltage=''))
    assert bridge.battery_guard_voltage() is None


def test_message_without_values_does_not_raise(bridge):
    bridge._on_battery_state(SimpleNamespace(level=1, values=None))
    assert bridge.battery_state() == BatteryState.WARNING


# ---------------------------------------------------------------------
# 過期判定：battery_guard 掛掉後不能停在最後一次的樂觀狀態
# ---------------------------------------------------------------------

def test_never_received_is_unknown_regardless_of_elapsed_time(bridge, clock):
    """從未收訊的行為與過期無關：不管過多久，都維持上一輪的 unknown 語意。"""
    clock.advance(TIMEOUT * 100)
    assert bridge.battery_state() == BatteryState.UNKNOWN
    assert bridge.battery_stop_latched() is False
    assert bridge.battery_guard_voltage() is None


def test_state_survives_until_timeout(bridge, clock):
    """還沒到門檻就是新鮮的；邊界值本身仍算新鮮（<= timeout）。"""
    bridge._on_battery_state(make_status(level=1, state='WARNING'))
    clock.advance(TIMEOUT - 0.1)
    assert bridge.battery_state() == BatteryState.WARNING
    clock.advance(0.1)  # 正好 TIMEOUT
    assert bridge.battery_state() == BatteryState.WARNING


def test_state_expires_after_timeout(bridge, clock):
    """battery_guard 停止發布 → 超過門檻退回 unknown，不是凍結在 ok。"""
    bridge._on_battery_state(make_status(level=0, state='OK'))
    assert bridge.battery_state() == BatteryState.OK
    clock.advance(TIMEOUT + 0.01)
    assert bridge.battery_state() == BatteryState.UNKNOWN


def test_expiry_also_clears_latched_and_voltage(bridge, clock):
    """三個欄位同源，必須一起過期，不能出現「狀態未知但仍鎖存」的矛盾讀數。"""
    bridge._on_battery_state(make_status(
        level=2, state='SHUTDOWN', stop_latched='true', voltage='21.500'))
    assert bridge.battery_stop_latched() is True
    clock.advance(TIMEOUT + 0.01)
    assert bridge.battery_state() == BatteryState.UNKNOWN
    assert bridge.battery_stop_latched() is False
    assert bridge.battery_guard_voltage() is None


def test_recovers_when_publisher_returns(bridge, clock):
    """上游復活後必須立刻恢復正常解析，不能卡在 unknown。"""
    bridge._on_battery_state(make_status(level=1, state='WARNING'))
    clock.advance(TIMEOUT + 1.0)
    assert bridge.battery_state() == BatteryState.UNKNOWN

    bridge._on_battery_state(make_status(
        level=1, state='WARNING', stop_latched='false', voltage='22.150'))
    assert bridge.battery_state() == BatteryState.WARNING
    assert bridge.battery_guard_voltage() == pytest.approx(22.15)

    clock.advance(TIMEOUT + 1.0)  # 再度失聯仍會再過期
    assert bridge.battery_state() == BatteryState.UNKNOWN


def test_timeout_is_configurable(clock):
    bridge = RosBridge(battery_state_timeout_sec=1.0, clock=clock)
    bridge._on_battery_state(make_status(level=0, state='OK'))
    clock.advance(1.5)
    assert bridge.battery_state() == BatteryState.UNKNOWN


def test_non_positive_timeout_disables_expiry(clock):
    """設 0＝停用過期判定（現場緊急關閉用），退回舊的「永久保留」行為。"""
    bridge = RosBridge(battery_state_timeout_sec=0.0, clock=clock)
    bridge._on_battery_state(make_status(level=0, state='OK'))
    clock.advance(86400.0)
    assert bridge.battery_state() == BatteryState.OK


def test_default_timeout_comes_from_config(clock):
    """預設值必須接上 config（可由 ROBOT_BATTERY_STATE_TIMEOUT 調），不是寫死。"""
    assert RosBridge(clock=clock)._battery_state_timeout_sec == BATTERY_STATE_TIMEOUT_SEC


# ---------------------------------------------------------------------
# robot_info 契約
# ---------------------------------------------------------------------

def make_info(**overrides) -> RobotInfo:
    kwargs = dict(
        op_mode=OpMode.NAVIGATE,
        status=RobotStatus.IDLE,
        battery=80,
        voltage=24.5,
        location=Location(x=10, y=20, orientation=90.0),
    )
    kwargs.update(overrides)
    return RobotInfo(**kwargs)


def test_robot_info_defaults_are_unknown_and_unlatched():
    """沒有 battery_guard 時的預設值，不影響既有欄位。"""
    info = make_info()
    assert info.battery_state == BatteryState.UNKNOWN
    assert info.battery_stop_latched is False


def test_info_message_carries_battery_fields():
    msg = EventHub._info_message(make_info(
        battery_state=BatteryState.SHUTDOWN, battery_stop_latched=True))
    assert msg['battery_state'] == 'shutdown'
    assert msg['battery_stop_latched'] is True


def test_info_message_keeps_existing_fields():
    """新欄位是加法：舊 client 依賴的鍵一個都不能少或改型別。"""
    msg = EventHub._info_message(make_info())
    assert msg['event'] == 'robot_info'
    assert msg['op_mode'] == 'navigate'
    assert msg['status'] == 'idle'
    assert msg['battery'] == 80
    assert msg['voltage'] == pytest.approx(24.5)
    assert msg['location'] == {'x': 10, 'y': 20, 'orientation': 90.0}


def test_info_message_battery_state_is_plain_string():
    """必須是 JSON 可序列化的字串，不是 Enum（json.dumps 會炸）。"""
    import json
    msg = EventHub._info_message(make_info(battery_state=BatteryState.WARNING))
    assert type(msg['battery_state']) is str
    assert json.loads(json.dumps(msg))['battery_state'] == 'warning'
