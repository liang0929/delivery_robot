"""battery_state adapter 的離線測試（不起節點、不 rclpy.init）。

盯的是三件會讓 dock 流程直接失效的事：

1. **電流反號**。Pico 放電為正、BatteryState 充電為正，弄反的話
   opennav_docking 的 ``current > charging_threshold`` 永遠得到相反答案。
2. **逾時退化**。來源死掉時必須退成 NaN + ``present=False``；停在最後一筆
   會讓對面凍結在「充電中」。
3. **無效輸入不進管線**。NaN / Inf / 離譜數值一旦流進 msg，下游的門檻比較
   會安靜地失效（NaN 比較恆為 False）。

msg 組裝（``build_battery_state``）另外測欄位填法，需要 ``sensor_msgs``
但仍不需要執行期。
"""

import math

import pytest

from pico_sensor_hub.battery_state_adapter import (
    BatteryStateAdapter,
    SOURCE_CURRENT,
    SOURCE_VOLTAGE,
    STATUS_CHARGING,
    STATUS_DISCHARGING,
    STATUS_NOT_CHARGING,
    STATUS_UNKNOWN,
)

TIMEOUT = 5.0


@pytest.fixture
def adapter():
    return BatteryStateAdapter(source_timeout_sec=TIMEOUT)


def feed(adapter, voltage, current, now=0.0):
    """灌一輪完整的 $PW（電壓＋電流同時到，與 Pico 的實際行為一致）。"""
    assert adapter.submit_voltage(voltage, now)
    assert adapter.submit_current(current, now)


# --------------------------------------------------------------------------
# 電流反號（最高風險路徑）
# --------------------------------------------------------------------------

def test_discharging_input_becomes_negative(adapter):
    """Pico 放電為正 5.0 A → BatteryState 應為 -5.0 A。"""
    feed(adapter, 24.0, 5.0)
    assert adapter.snapshot(0.0).current == pytest.approx(-5.0)


def test_charging_input_becomes_positive(adapter):
    """Pico 充電為負 -3.0 A → BatteryState 應為 +3.0 A。"""
    feed(adapter, 24.0, -3.0)
    assert adapter.snapshot(0.0).current == pytest.approx(3.0)


def test_charging_current_passes_opennav_threshold(adapter):
    """充電 2 A 必須大於 opennav_docking 的 charging_threshold（0.5 A）。

    這條斷言等價於對面那行 ``is_charging_ = state->current > 0.5``；
    反號寫錯時這裡會先炸，不必等到車真的停在 dock 上才發現。
    """
    feed(adapter, 24.0, -2.0)
    assert adapter.snapshot(0.0).current > 0.5


def test_discharging_current_fails_opennav_threshold(adapter):
    feed(adapter, 24.0, 2.0)
    assert not adapter.snapshot(0.0).current > 0.5


def test_zero_current_is_positive_zero(adapter):
    """零電流不可以變成 -0.0（數值相等，但 echo 出來像在放電）。"""
    feed(adapter, 24.0, 0.0)
    current = adapter.snapshot(0.0).current
    assert current == 0.0
    assert not math.copysign(1.0, current) < 0


def test_voltage_is_not_negated(adapter):
    """反號只對電流，電壓照抄。"""
    feed(adapter, 24.5, 5.0)
    assert adapter.snapshot(0.0).voltage == pytest.approx(24.5)


# --------------------------------------------------------------------------
# power_supply_status
# --------------------------------------------------------------------------

def test_status_charging(adapter):
    feed(adapter, 24.0, -2.0)
    assert adapter.snapshot(0.0).status == STATUS_CHARGING


def test_status_discharging(adapter):
    feed(adapter, 24.0, 2.0)
    assert adapter.snapshot(0.0).status == STATUS_DISCHARGING


def test_status_dead_zone_is_not_charging(adapter):
    """門檻內（接著充電器但還沒灌電流）→ NOT_CHARGING，不是 UNKNOWN。"""
    feed(adapter, 24.0, -0.1)
    assert adapter.snapshot(0.0).status == STATUS_NOT_CHARGING


def test_status_unknown_before_any_message(adapter):
    assert adapter.snapshot(0.0).status == STATUS_UNKNOWN


# --------------------------------------------------------------------------
# 逾時退化與自癒
# --------------------------------------------------------------------------

def test_fresh_within_timeout(adapter):
    feed(adapter, 24.0, 5.0, now=100.0)
    reading = adapter.snapshot(100.0 + TIMEOUT)      # 邊界上仍算新鮮
    assert reading.present
    assert reading.voltage == pytest.approx(24.0)


def test_stale_degrades_to_nan_and_absent(adapter):
    feed(adapter, 24.0, 5.0, now=100.0)
    reading = adapter.snapshot(100.0 + TIMEOUT + 0.01)
    assert math.isnan(reading.voltage)
    assert math.isnan(reading.current)
    assert reading.present is False
    assert reading.status == STATUS_UNKNOWN


def test_stale_current_never_reads_as_charging(adapter):
    """退化值必須讓對面判定「沒在充電」——NaN 比較恆為 False。"""
    feed(adapter, 24.0, -5.0, now=100.0)             # 先處於充電中
    stale = adapter.snapshot(100.0 + TIMEOUT + 0.01)
    assert not stale.current > 0.5


def test_recovers_after_source_returns(adapter):
    """來源回來要自癒，不能鎖在退化狀態。"""
    feed(adapter, 24.0, 5.0, now=0.0)
    assert not adapter.snapshot(100.0).present

    feed(adapter, 23.5, -1.0, now=100.0)
    reading = adapter.snapshot(100.0)
    assert reading.present
    assert reading.voltage == pytest.approx(23.5)
    assert reading.current == pytest.approx(1.0)


def test_never_received_is_absent(adapter):
    """從沒收過訊息＝不新鮮，且與時鐘無關（不會因為 now 很小就算新鮮）。"""
    for now in (0.0, 1e9):
        reading = adapter.snapshot(now)
        assert reading.present is False
        assert math.isnan(reading.voltage)


def test_timeout_disabled_keeps_last_value():
    """timeout <= 0＝停用逾時判定（現場逃生門），退回舊行為。"""
    adapter = BatteryStateAdapter(source_timeout_sec=0.0)
    feed(adapter, 24.0, 5.0, now=0.0)
    reading = adapter.snapshot(1e6)
    assert reading.present
    assert reading.voltage == pytest.approx(24.0)


def test_one_source_stale_keeps_present(adapter):
    """只有一路過期時，過期那路填 NaN，但電池仍算在場。

    兩路同源（同一筆 $PW），單路過期多半是丟包；這時把 present 打成 False
    是假警報，反而讓下游以為電池被拔了。
    """
    adapter.submit_voltage(24.0, 100.0)
    adapter.submit_current(5.0, 0.0)
    reading = adapter.snapshot(100.0)
    assert reading.present
    assert reading.voltage == pytest.approx(24.0)
    assert math.isnan(reading.current)
    assert reading.status == STATUS_UNKNOWN


# --------------------------------------------------------------------------
# 無效輸入
# --------------------------------------------------------------------------

@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf, 0.0, 4.9, 60.1, 1e9])
def test_invalid_voltage_rejected(adapter, bad):
    assert adapter.submit_voltage(bad, 0.0) is False
    assert adapter.rejected_count(SOURCE_VOLTAGE) == 1


@pytest.mark.parametrize('bad', [math.nan, math.inf, -math.inf, 1e9, -1e9])
def test_invalid_current_rejected(adapter, bad):
    assert adapter.submit_current(bad, 0.0) is False
    assert adapter.rejected_count(SOURCE_CURRENT) == 1


def test_invalid_value_does_not_refresh_timestamp(adapter):
    """壞值不能當作「這一路還活著」的證據，否則永遠不會退化。"""
    feed(adapter, 24.0, 5.0, now=0.0)
    for t in range(1, 20):
        adapter.submit_voltage(math.nan, float(t))
        adapter.submit_current(math.inf, float(t))
    assert adapter.snapshot(19.0).present is False


def test_invalid_value_does_not_clobber_last_good(adapter):
    """壞值進來時，上一筆好值在逾時之前仍然有效。"""
    feed(adapter, 24.0, 5.0, now=0.0)
    adapter.submit_voltage(math.nan, 1.0)
    reading = adapter.snapshot(1.0)
    assert reading.voltage == pytest.approx(24.0)


def test_boundary_voltages_accepted(adapter):
    """有效範圍是閉區間，邊界值不該被誤殺。"""
    assert adapter.submit_voltage(5.0, 0.0)
    assert adapter.submit_voltage(60.0, 0.0)


# --------------------------------------------------------------------------
# msg 組裝
# --------------------------------------------------------------------------

def test_build_battery_state_fields(adapter):
    from pico_sensor_hub.battery_state_node import build_battery_state
    from sensor_msgs.msg import BatteryState

    feed(adapter, 24.0, -2.0)
    msg = build_battery_state(adapter.snapshot(0.0), cell_count=7)

    assert msg.voltage == pytest.approx(24.0)
    assert msg.current == pytest.approx(2.0)
    assert msg.present is True
    assert msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_CHARGING
    # 7S 18650 鋰離子（docs/power_design.md §2），LION 不是 LIPO
    assert msg.power_supply_technology == BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
    assert msg.power_supply_health == BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
    # 量不到的欄位一律 NaN，不拿電壓反推
    for field in ('temperature', 'charge', 'capacity', 'design_capacity',
                  'percentage'):
        assert math.isnan(getattr(msg, field)), field
    assert len(msg.cell_voltage) == 7
    assert all(math.isnan(v) for v in msg.cell_voltage)


def test_build_battery_state_stale(adapter):
    from pico_sensor_hub.battery_state_node import build_battery_state
    from sensor_msgs.msg import BatteryState

    feed(adapter, 24.0, 5.0, now=0.0)
    msg = build_battery_state(adapter.snapshot(100.0), cell_count=7)

    assert math.isnan(msg.voltage)
    assert math.isnan(msg.current)
    assert msg.present is False
    assert msg.power_supply_status == BatteryState.POWER_SUPPLY_STATUS_UNKNOWN


def test_build_battery_state_without_cell_count(adapter):
    from pico_sensor_hub.battery_state_node import build_battery_state

    feed(adapter, 24.0, 5.0)
    msg = build_battery_state(adapter.snapshot(0.0), cell_count=0)
    assert list(msg.cell_voltage) == []
    assert list(msg.cell_temperature) == []
