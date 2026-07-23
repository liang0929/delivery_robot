"""
hs_protocol.py 對照測試（不需要 ROS/硬體）

驗證方式：在本測試檔內以「抽取前」HSMotorController 原始程式碼逐字複製一份
reference 實作（crc16 / 命令封包編碼 / 應答封包解碼），與抽取後的
motor_control.hs_protocol 模組互相比對，確保 byte 順序、CRC16 演算法、
量化門檻等數值完全一致（golden sample 對照，而非重新推導演算法）。

用法::

    /usr/bin/python3 -m pytest tests/test_hs_protocol.py -v
"""

import os
import random
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, 'src', 'motor_control', 'motor_control'))

from motor_control import hs_protocol  # noqa: E402


# ---------------------------------------------------------------------------
# Reference 實作：逐字複製自抽取前 HSMotorController 的對應 method
# （HSMotorController.crc16 / build_command_packet / _try_parse_packet）
# ---------------------------------------------------------------------------

def ref_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def ref_build_command_packet(device_id, clear_fault, motor_control_byte, dir_a, dir_b,
                              target_rpm_a, target_rpm_b) -> bytes:
    START_BYTE_MASTER = 0xAA
    END_BYTE_MASTER = 0x55

    packet = bytearray()
    packet.append(START_BYTE_MASTER)
    packet.append(device_id & 0x7F)
    packet.append(0x01)
    packet.append(clear_fault & 0x01)
    packet.append(0x00)
    packet.append(motor_control_byte)
    packet.append(motor_control_byte)
    packet.append(dir_a & 0x01)
    packet.append(dir_b & 0x01)
    packet.append((target_rpm_a >> 8) & 0xFF)
    packet.append(target_rpm_a & 0xFF)
    packet.append((target_rpm_b >> 8) & 0xFF)
    packet.append(target_rpm_b & 0xFF)
    packet.append(END_BYTE_MASTER)
    crc = ref_crc16(bytes(packet[0:14]))
    packet.append(crc & 0xFF)
    packet.append((crc >> 8) & 0xFF)
    return bytes(packet)


def ref_try_parse_packet(packet: bytes, device_id: int):
    """回傳 (current_a, current_b, actual_rpm_a, actual_rpm_b, voltage, fault_code) 或 None"""
    END_BYTE_SLAVE = 0xAA

    if packet[13] != END_BYTE_SLAVE:
        return None

    received_crc = packet[14] | (packet[15] << 8)
    calculated_crc = ref_crc16(packet[0:14])
    if received_crc != calculated_crc:
        return None

    if device_id != 127 and packet[1] != device_id:
        return None

    current_a = int.from_bytes(packet[2:4], byteorder='big') * 0.1
    current_b = int.from_bytes(packet[4:6], byteorder='big') * 0.1
    actual_rpm_a = float(int.from_bytes(packet[6:8], byteorder='big'))
    actual_rpm_b = float(int.from_bytes(packet[8:10], byteorder='big'))
    voltage = int.from_bytes(packet[10:12], byteorder='big') * 0.01
    fault_code = packet[12]
    return (current_a, current_b, actual_rpm_a, actual_rpm_b, voltage, fault_code)


# ---------------------------------------------------------------------------
# CRC16
# ---------------------------------------------------------------------------

def test_crc16_matches_reference_on_random_inputs():
    rng = random.Random(1234)
    for _ in range(200):
        length = rng.randint(0, 32)
        data = bytes(rng.randrange(256) for _ in range(length))
        assert hs_protocol.crc16(data) == ref_crc16(data)


def test_crc16_known_values():
    # CRC 空輸入即為初始值 0xFFFF
    assert hs_protocol.crc16(b'') == 0xFFFF
    # 與 ref 實作在幾個固定輸入上比對
    for sample in (b'\x00', b'\xAA\x7F\x01\x00\x00\x01\x01\x00\x00\x00d\x00d\x55'):
        assert hs_protocol.crc16(sample) == ref_crc16(sample)


# ---------------------------------------------------------------------------
# encode_command (命令封包)
# ---------------------------------------------------------------------------

def test_encode_command_matches_reference_random_inputs():
    rng = random.Random(42)
    for _ in range(200):
        device_id = rng.randint(0, 127)
        clear_fault = rng.randint(0, 1)
        motor_control_byte = rng.choice([0x00, 0x01, 0x03])
        dir_a = rng.randint(0, 1)
        dir_b = rng.randint(0, 1)
        rpm_a = rng.randint(0, 3000)
        rpm_b = rng.randint(0, 3000)

        expected = ref_build_command_packet(
            device_id, clear_fault, motor_control_byte, dir_a, dir_b, rpm_a, rpm_b)
        actual = hs_protocol.encode_command(
            device_id=device_id, clear_fault=clear_fault,
            motor_control_byte=motor_control_byte,
            dir_a=dir_a, dir_b=dir_b, rpm_a=rpm_a, rpm_b=rpm_b)

        assert actual == expected
        assert len(actual) == 16


def test_encode_command_known_packet_fields():
    """驗證欄位位置（依協議文件註解）與封包長度 / 結尾碼"""
    packet = hs_protocol.encode_command(
        device_id=1, clear_fault=0, motor_control_byte=0x01,
        dir_a=0, dir_b=1, rpm_a=1500, rpm_b=800)

    assert len(packet) == 16
    assert packet[0] == 0xAA          # 起始碼
    assert packet[1] == 1             # 地址
    assert packet[2] == 0x01          # 返回數據類型
    assert packet[3] == 0x00          # 故障清除
    assert packet[4] == 0x00          # 保留位
    assert packet[5] == 0x01          # A控制
    assert packet[6] == 0x01          # B控制
    assert packet[7] == 0             # A方向
    assert packet[8] == 1             # B方向
    assert (packet[9] << 8) | packet[10] == 1500  # A轉速
    assert (packet[11] << 8) | packet[12] == 800  # B轉速
    assert packet[13] == 0x55         # 結束碼
    # CRC 範圍為 packet[0:14]
    assert hs_protocol.crc16(packet[0:14]) == (packet[14] | (packet[15] << 8))


def test_encode_command_clear_fault_and_dir_masked():
    """clear_fault / dir 只取最低位 (& 0x01)，device_id 只取 7 bits (& 0x7F)，與原邏輯一致"""
    packet = hs_protocol.encode_command(
        device_id=255, clear_fault=3, motor_control_byte=0x03,
        dir_a=5, dir_b=4, rpm_a=0, rpm_b=0)
    assert packet[1] == 255 & 0x7F  # == 127
    assert packet[3] == 3 & 0x01    # == 1
    assert packet[7] == 5 & 0x01    # == 1
    assert packet[8] == 4 & 0x01    # == 0


# ---------------------------------------------------------------------------
# decode_packet / 應答封包解析
# ---------------------------------------------------------------------------

def _make_response_packet(device_id, current_a_raw, current_b_raw, rpm_a_raw, rpm_b_raw,
                           voltage_raw, fault_code, end_byte=0xAA, corrupt_crc=False):
    """依應答封包格式手動組出 16-byte 封包（獨立於 hs_protocol 實作）"""
    packet = bytearray()
    packet.append(0x55)  # 起始碼
    packet.append(device_id)
    packet += current_a_raw.to_bytes(2, 'big')
    packet += current_b_raw.to_bytes(2, 'big')
    packet += rpm_a_raw.to_bytes(2, 'big')
    packet += rpm_b_raw.to_bytes(2, 'big')
    packet += voltage_raw.to_bytes(2, 'big')
    packet.append(fault_code)
    packet.append(end_byte)
    crc = ref_crc16(bytes(packet[0:14]))
    if corrupt_crc:
        crc ^= 0xFFFF
    packet.append(crc & 0xFF)
    packet.append((crc >> 8) & 0xFF)
    return bytes(packet)


def test_decode_packet_matches_reference_random_inputs():
    rng = random.Random(7)
    for _ in range(200):
        device_id = rng.randint(1, 127)
        packet = _make_response_packet(
            device_id=device_id,
            current_a_raw=rng.randint(0, 0xFFFF),
            current_b_raw=rng.randint(0, 0xFFFF),
            rpm_a_raw=rng.randint(0, 3000),
            rpm_b_raw=rng.randint(0, 3000),
            voltage_raw=rng.randint(0, 3000),
            fault_code=rng.randint(0, 11),
        )
        expected = ref_try_parse_packet(packet, device_id)
        result = hs_protocol.decode_packet(packet, device_id)

        assert expected is not None
        assert result.ok is True
        assert result.response.current_a == expected[0]
        assert result.response.current_b == expected[1]
        assert result.response.actual_rpm_a == expected[2]
        assert result.response.actual_rpm_b == expected[3]
        assert result.response.voltage == expected[4]
        assert result.response.fault_code == expected[5]


def test_decode_packet_broadcast_address_skips_check():
    """device_id 127 為廣播地址，即使回應地址不同也應解析成功（與原邏輯一致）"""
    packet = _make_response_packet(
        device_id=5, current_a_raw=10, current_b_raw=20,
        rpm_a_raw=100, rpm_b_raw=200, voltage_raw=2500, fault_code=0)
    result = hs_protocol.decode_packet(packet, device_id=127)
    assert result.ok is True
    assert result.response.actual_rpm_a == 100.0


def test_decode_packet_address_mismatch_rejected():
    packet = _make_response_packet(
        device_id=5, current_a_raw=0, current_b_raw=0,
        rpm_a_raw=0, rpm_b_raw=0, voltage_raw=0, fault_code=0)
    result = hs_protocol.decode_packet(packet, device_id=6)
    assert result.ok is False
    assert result.response is None
    assert 'Address mismatch' in result.error


def test_decode_packet_invalid_end_byte_rejected():
    packet = _make_response_packet(
        device_id=1, current_a_raw=0, current_b_raw=0,
        rpm_a_raw=0, rpm_b_raw=0, voltage_raw=0, fault_code=0, end_byte=0x00)
    result = hs_protocol.decode_packet(packet, device_id=1)
    assert result.ok is False
    assert 'Invalid end byte' in result.error


def test_decode_packet_crc_mismatch_rejected():
    packet = _make_response_packet(
        device_id=1, current_a_raw=0, current_b_raw=0,
        rpm_a_raw=0, rpm_b_raw=0, voltage_raw=0, fault_code=0, corrupt_crc=True)
    result = hs_protocol.decode_packet(packet, device_id=1)
    assert result.ok is False
    assert 'CRC mismatch' in result.error


def test_decode_packet_resolution_scaling():
    """電流解析度 0.1A、電壓解析度 0.01V"""
    packet = _make_response_packet(
        device_id=1, current_a_raw=123, current_b_raw=45,
        rpm_a_raw=1500, rpm_b_raw=1600, voltage_raw=2456, fault_code=2)
    result = hs_protocol.decode_packet(packet, device_id=1)
    assert result.ok is True
    assert result.response.current_a == 123 * 0.1
    assert result.response.current_b == 45 * 0.1
    assert result.response.voltage == 2456 * 0.01
    assert result.response.fault_code == 2


def test_encode_then_find_end_to_end():
    """encode_command 產生的封包若當成應答封包解析，結束碼會不合法（AA vs 55 起始）
    -- 這裡改用 command 的 END_BYTE_MASTER 驗證 encode/decode 常數不混用。"""
    assert hs_protocol.START_BYTE_MASTER != hs_protocol.START_BYTE_SLAVE
    assert hs_protocol.END_BYTE_MASTER != hs_protocol.END_BYTE_SLAVE
    assert hs_protocol.START_BYTE_MASTER == hs_protocol.END_BYTE_SLAVE
    assert hs_protocol.END_BYTE_MASTER == hs_protocol.START_BYTE_SLAVE
