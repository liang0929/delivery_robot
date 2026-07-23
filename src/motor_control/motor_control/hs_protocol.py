"""
HS 協議 (AGV-BLD-2S) 純編解碼模組 - 無 ROS 依賴

從 HSMotorController 抽出的純邏輯：CRC16 演算法、命令封包編碼、
應答封包解碼。所有 byte offset、CRC 演算法與數值皆與抽取前的
HSMotorController.build_command_packet / parse_response_packet /
_try_parse_packet 完全一致，僅將散落的 magic number 換成具名常數。

封包格式：
詢問封包 (Master -> Slave, 16 bytes):
    AA + 地址 + 數據類型 + 故障清除 + 保留 + A控制 + B控制 +
    A方向 + B方向 + A轉速(2B) + B轉速(2B) + 55 + CRC16(2B)
應答封包 (Slave -> Master, 16 bytes):
    55 + 地址 + A電流(2B) + B電流(2B) + A轉速(2B) + B轉速(2B) +
    電壓(2B) + 故障 + AA + CRC16(2B)

注意：欄位順序以實測硬體行為為準，如與手冊不符請先驗證再修改。
"""

from typing import NamedTuple, Optional

# ---------------------------------------------------------------------------
# 協議常量
# ---------------------------------------------------------------------------

START_BYTE_MASTER = 0xAA
END_BYTE_MASTER = 0x55
START_BYTE_SLAVE = 0x55
END_BYTE_SLAVE = 0xAA

PACKET_LENGTH = 16

# 命令封包 (Master -> Slave) 欄位 offset
CMD_IDX_START = 0
CMD_IDX_ADDRESS = 1
CMD_IDX_RETURN_TYPE = 2
CMD_IDX_CLEAR_FAULT = 3
CMD_IDX_RESERVED = 4
CMD_IDX_MOTOR_A_CTRL = 5
CMD_IDX_MOTOR_B_CTRL = 6
CMD_IDX_DIR_A = 7
CMD_IDX_DIR_B = 8
CMD_IDX_RPM_A_HIGH = 9
CMD_IDX_RPM_A_LOW = 10
CMD_IDX_RPM_B_HIGH = 11
CMD_IDX_RPM_B_LOW = 12
CMD_IDX_END = 13
CMD_IDX_CRC_LOW = 14
CMD_IDX_CRC_HIGH = 15
# CRC 計算範圍：起始碼到結束碼 (Byte 1-14, 即 packet[0:14])
CMD_CRC_RANGE = slice(0, 14)

CMD_RETURN_TYPE_WITH_DATA = 0x01  # 需要回傳數據

# 應答封包 (Slave -> Master) 欄位 offset
RESP_IDX_START = 0
RESP_IDX_ADDRESS = 1
RESP_IDX_CURRENT_A = slice(2, 4)
RESP_IDX_CURRENT_B = slice(4, 6)
RESP_IDX_RPM_A = slice(6, 8)
RESP_IDX_RPM_B = slice(8, 10)
RESP_IDX_VOLTAGE = slice(10, 12)
RESP_IDX_FAULT = 12
RESP_IDX_END = 13
RESP_IDX_CRC_LOW = 14
RESP_IDX_CRC_HIGH = 15
# CRC 計算範圍：起始碼到結束碼 (Byte 1-14, 即 packet[0:14])
RESP_CRC_RANGE = slice(0, 14)

CURRENT_RESOLUTION = 0.1  # A，解析度 0.1A
VOLTAGE_RESOLUTION = 0.01  # V，解析度 0.01V

# 廣播地址：跳過地址檢查
BROADCAST_DEVICE_ID = 127


def crc16(data: bytes) -> int:
    """計算 CRC16 校驗碼 (Modbus CRC16)

    與抽取前 HSMotorController.crc16 演算法/數值完全相同。
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


def encode_command(
    device_id: int,
    clear_fault: int,
    motor_control_byte: int,
    dir_a: int,
    dir_b: int,
    rpm_a: int,
    rpm_b: int,
) -> bytes:
    """建立 HS 協議命令封包 (16 bytes)

    與抽取前 HSMotorController.build_command_packet 的 byte 順序、
    位元運算與數值完全相同（E-Stop / motor_enabled 判斷邏輯留在
    呼叫端，這裡只做純粹的 byte 封裝 + CRC）。

    Args:
        device_id: 驅動器地址 (1-127)
        clear_fault: 故障清除旗標 (0 或 1)
        motor_control_byte: A/B 馬達控制碼 (共用同一個值，與原邏輯一致)
        dir_a: A 馬達方向 (0 或 1)
        dir_b: B 馬達方向 (0 或 1)
        rpm_a: A 馬達目標轉速
        rpm_b: B 馬達目標轉速

    Returns:
        16 bytes 的命令封包
    """
    packet = bytearray()

    # Byte 1: 起始碼 (AA)
    packet.append(START_BYTE_MASTER)

    # Byte 2: 地址碼 (1-127)
    packet.append(device_id & 0x7F)

    # Byte 3: 返回數據類型 (00: 不返回, 01: 返回運行數據)
    packet.append(CMD_RETURN_TYPE_WITH_DATA)

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
    packet.append((rpm_a >> 8) & 0xFF)  # 高位
    packet.append(rpm_a & 0xFF)         # 低位

    # Byte 12-13: B電機轉速值 (高位在前, 低位在後) 100-3000 RPM
    packet.append((rpm_b >> 8) & 0xFF)  # 高位
    packet.append(rpm_b & 0xFF)         # 低位

    # Byte 14: 結束碼 (55)
    packet.append(END_BYTE_MASTER)

    # Byte 15-16: CRC16 校驗碼 (低位在前, 高位在後)
    crc = crc16(bytes(packet[CMD_CRC_RANGE]))
    packet.append(crc & 0xFF)         # CRC 低位
    packet.append((crc >> 8) & 0xFF)  # CRC 高位

    return bytes(packet)


class DecodedResponse(NamedTuple):
    """解析成功後的應答封包內容（與原 _try_parse_packet 更新的欄位一一對應）"""
    current_a: float
    current_b: float
    actual_rpm_a: float
    actual_rpm_b: float
    voltage: float
    fault_code: int


class DecodeResult(NamedTuple):
    """單一候選封包的解析結果，失敗時附帶原因字串供呼叫端記 debug log"""
    ok: bool
    response: Optional[DecodedResponse]
    error: Optional[str]


def decode_packet(packet: bytes, device_id: int) -> DecodeResult:
    """驗證並解析單一 16-byte 候選封包

    與抽取前 HSMotorController._try_parse_packet 的驗證順序、
    byte offset 與數值換算完全相同。

    Args:
        packet: 恰好 16 bytes 的候選封包
        device_id: 用於位址比對；127 為廣播地址，跳過檢查

    Returns:
        DecodeResult(ok=True, response=..., error=None) 表示解析成功；
        ok=False 時 response 為 None，error 為除錯訊息（對應原本的
        self.get_logger().debug(...) 內容）。
    """
    # 驗證結束碼 (Byte 14 = 0xAA)
    if packet[RESP_IDX_END] != END_BYTE_SLAVE:
        return DecodeResult(False, None, f'Invalid end byte: {packet[RESP_IDX_END]:02X}')

    # 驗證 CRC16 (Byte 15-16, 低位在前)
    received_crc = packet[RESP_IDX_CRC_LOW] | (packet[RESP_IDX_CRC_HIGH] << 8)
    calculated_crc = crc16(packet[RESP_CRC_RANGE])  # CRC 計算範圍: 起始碼到結束碼
    if received_crc != calculated_crc:
        return DecodeResult(
            False, None,
            f'CRC mismatch: recv={received_crc:04X} calc={calculated_crc:04X}')

    # 驗證地址 (Byte 2)；device_id 127 為廣播地址，跳過檢查
    if device_id != BROADCAST_DEVICE_ID and packet[RESP_IDX_ADDRESS] != device_id:
        return DecodeResult(
            False, None,
            f'Address mismatch: recv={packet[RESP_IDX_ADDRESS]} expect={device_id}')

    # 解析數據 (高位在前 Big-endian)

    # Byte 3-4: A電機電流 (解析度 0.1A)
    current_a = int.from_bytes(packet[RESP_IDX_CURRENT_A], byteorder='big') * CURRENT_RESOLUTION

    # Byte 5-6: B電機電流 (解析度 0.1A)
    current_b = int.from_bytes(packet[RESP_IDX_CURRENT_B], byteorder='big') * CURRENT_RESOLUTION

    # Byte 7-8: A電機轉速 (0-3000 RPM)
    actual_rpm_a = float(int.from_bytes(packet[RESP_IDX_RPM_A], byteorder='big'))

    # Byte 9-10: B電機轉速 (0-3000 RPM)
    actual_rpm_b = float(int.from_bytes(packet[RESP_IDX_RPM_B], byteorder='big'))

    # Byte 11-12: 電源電壓 (解析度 0.01V)
    voltage = int.from_bytes(packet[RESP_IDX_VOLTAGE], byteorder='big') * VOLTAGE_RESOLUTION

    # Byte 13: 故障狀態 (00 = 正常)
    fault_code = packet[RESP_IDX_FAULT]

    return DecodeResult(
        True,
        DecodedResponse(current_a, current_b, actual_rpm_a, actual_rpm_b, voltage, fault_code),
        None,
    )
