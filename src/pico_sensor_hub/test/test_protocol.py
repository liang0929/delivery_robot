"""protocol.py 的離線測試（不需要硬體，也不需要 rclpy）

重點在無效值映射：距離哨兵值**絕不可以變成 0**。
0 在 sensor_msgs/Range 語意裡是「障礙物貼著感測器」，會誤觸發急停。
"""

import math

import pytest

from pico_sensor_hub import protocol


def line(payload: str) -> str:
    """把 payload 補上 $ 與正確校驗，模擬韌體送出的一行。"""
    return protocol.build_line(payload).decode('ascii')


# --------------------------------------------------------------------------
# 無效值映射（最高風險路徑）
# --------------------------------------------------------------------------

def test_timeout_maps_to_positive_infinity():
    """-1 逾時無回波 → +Inf（範圍內沒東西，等同無限遠）"""
    assert protocol.distance_mm_to_range_m(-1) == math.inf


def test_fault_maps_to_nan():
    """-2 通道故障 → NaN（量測壞掉，什麼都不知道）"""
    assert math.isnan(protocol.distance_mm_to_range_m(-2))


def test_sentinels_never_become_zero():
    """哨兵值與未知負值都不可以變成 0。違反這條會導致誤急停。"""
    for mm in (-1, -2, -3, -100):
        value = protocol.distance_mm_to_range_m(mm)
        assert value != 0.0
        assert math.isinf(value) or math.isnan(value)


def test_valid_distance_converts_to_metres():
    assert protocol.distance_mm_to_range_m(1234) == pytest.approx(1.234)
    assert protocol.distance_mm_to_range_m(2) == pytest.approx(0.002)


def test_zero_from_firmware_stays_zero():
    """韌體承諾不送 0；萬一真的送了，不要幫它掩蓋成 Inf——

    0 應該原樣浮上來讓下游／log 看見異常，而不是被這層悄悄改寫。
    """
    assert protocol.distance_mm_to_range_m(0) == 0.0


# --------------------------------------------------------------------------
# 校驗
# --------------------------------------------------------------------------

def test_parse_line_accepts_valid_checksum():
    assert protocol.parse_line(line('CMD,PING')) == 'CMD,PING'


def test_parse_line_rejects_bad_checksum():
    with pytest.raises(protocol.ChecksumError):
        protocol.parse_line('$CMD,PING*00')


@pytest.mark.parametrize('bad', [
    'CMD,PING',              # 沒有 $ 也沒有校驗
    '$CMD,PING',             # 缺校驗欄
    '$CMD,PING*ZZ',          # 校驗欄非十六進位
    '$CMD,PING*1',           # 校驗欄長度不對
    '',
])
def test_parse_line_rejects_malformed(bad):
    with pytest.raises(protocol.ChecksumError):
        protocol.parse_line(bad)


# --------------------------------------------------------------------------
# $US
# --------------------------------------------------------------------------

def test_parse_us_mixed_validity():
    kind, data = protocol.parse_message(
        line('US,42,100,-1,-2,1500,4000,-1,20,3999'))
    assert kind == 'US'
    assert data['seq'] == 42
    r = data['ranges_m']
    assert r[0] == pytest.approx(0.1)
    assert r[1] == math.inf
    assert math.isnan(r[2])
    assert r[3] == pytest.approx(1.5)
    assert 0.0 not in r


def test_parse_us_wrong_field_count():
    with pytest.raises(ValueError):
        protocol.parse_message(line('US,1,10,20,30'))


# --------------------------------------------------------------------------
# $PW（韌體 v0.2.0 的六欄格式）
# --------------------------------------------------------------------------

def test_parse_pw_valid():
    kind, data = protocol.parse_message(line('PW,7,1,25200,3100,78120'))
    assert kind == 'PW'
    assert data['ok'] is True
    assert data['bus_v'] == pytest.approx(25.2)
    assert data['current_a'] == pytest.approx(3.1)
    assert data['power_w'] == pytest.approx(78.12)


def test_parse_pw_invalid_yields_none_not_sentinel():
    """ok=0 時電氣量測回 None，-1 這個哨兵值不可以流到下游。"""
    kind, data = protocol.parse_message(line('PW,8,0,-1,-1,-1'))
    assert data['ok'] is False
    assert data['bus_v'] is None
    assert data['current_a'] is None
    assert data['power_w'] is None


def test_parse_pw_negative_current_is_valid_measurement():
    """充電時 current 為負；-1 mA 是合法值，不該被當成無效。

    這正是 v0.2.0 加 ok 欄的理由：舊格式無法區分這兩種情形。
    """
    _, data = protocol.parse_message(line('PW,9,1,25200,-1,-25'))
    assert data['ok'] is True
    assert data['current_a'] == pytest.approx(-0.001)


def test_parse_pw_rejects_legacy_five_field_format():
    """v0.1.x 的五欄格式必須被拒絕，不能把 bus_mV 誤讀成 ok。"""
    with pytest.raises(ValueError):
        protocol.parse_message(line('PW,10,25200,3100,78120'))


# --------------------------------------------------------------------------
# $ST / $ID / 其他
# --------------------------------------------------------------------------

def test_parse_st():
    _, data = protocol.parse_message(line('ST,51006,001D,4080,16'))
    assert data['uptime_s'] == pytest.approx(51.006)
    assert data['flags'] == 0x1D
    assert data['us_timeout_count'] == 4080
    assert data['i2c_err_count'] == 16


def test_format_flags():
    assert protocol.format_flags(0) == '無'
    assert 'INA226故障' in protocol.format_flags(protocol.FLAG_INA226_FAULT)
    assert '未知位元' in protocol.format_flags(1 << 15)


def test_parse_id():
    _, data = protocol.parse_message(line('ID,0.2.0,2026-08-06T08:17:36Z,pico2_w'))
    assert data['fw_version'] == '0.2.0'
    assert data['board'] == 'pico2_w'


def test_unknown_kind_is_not_an_error():
    """$ACK/$NAK 等除錯回應不該讓解析器拋例外。"""
    kind, data = protocol.parse_message(line('ACK,PING'))
    assert kind == 'ACK'
    assert data['payload'] == 'ACK,PING'


def test_channel_slugs_match_firmware_order():
    """通道順序是韌體 README 6.1 腳位表的權威順序，改動要同步韌體。"""
    assert protocol.CHANNEL_SLUGS[0] == 'front_left'
    assert protocol.CHANNEL_SLUGS[7] == 'left_front'
    assert len(protocol.CHANNEL_SLUGS) == 8
    assert len(protocol.CHANNEL_NAMES_ZH) == 8
