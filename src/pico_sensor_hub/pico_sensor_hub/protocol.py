"""
pico_sensor_hub 序列協定解析（純 Python，不依賴 rclpy 也不依賴序列埠）

刻意與 ROS 無關，才能離線用 pytest 驗證無效值映射這條最高風險路徑——
距離哨兵值一旦被映射成 0，下游避障會把「沒偵測到」當成「貼著障礙物」。

協定權威文件：firmware/pico_sensor_hub/README.md 第 5 節（韌體 v0.2.0）。

    $US,<seq>,<d1>..<d8>*<XX>              距離 mm，-1 逾時、-2 通道故障，絕不為 0
    $PW,<seq>,<ok>,<mV>,<mA>,<mW>*<XX>     ok=0 時後三欄為 -1
    $ST,<uptime_ms>,<flags_hex>,<us_to>,<i2c_err>*<XX>
    $ID,<fw_ver>,<build_date>,<board>*<XX>
"""

import math

# 通道索引 → 車體位置。順序對應 $US 的 d1..d8，
# 與韌體 README 6.1 的腳位表、tools/picoterm.py 的 CHANNEL_NAMES 一致。
CHANNEL_SLUGS = [
    'front_left',   # 0 前左
    'front_right',  # 1 前右
    'right_front',  # 2 右前
    'right_rear',   # 3 右後
    'rear_right',   # 4 後右
    'rear_left',    # 5 後左
    'left_rear',    # 6 左後
    'left_front',   # 7 左前
]

CHANNEL_NAMES_ZH = ['前左', '前右', '右前', '右後', '後右', '後左', '左後', '左前']

# 韌體的距離哨兵值（firmware/pico_sensor_hub/src/main.c send_us 的註解）
DIST_TIMEOUT = -1   # 逾時無回波（含感測器未接線）
DIST_FAULT = -2     # 通道故障：觸發前 ECHO 就是高電位

# $ST flags 位元，與韌體 main.c 的 FLAG_* 定義一致
FLAG_INA226_FAULT = 1 << 0
FLAG_WDT_REBOOT = 1 << 1
FLAG_US_TIMEOUT = 1 << 2
FLAG_USB_DROP = 1 << 3
FLAG_INA226_CAL_ERR = 1 << 4
FLAG_BAD_CMD = 1 << 5
FLAG_US_CH_FAULT = 1 << 6

FLAG_NAMES = [
    (FLAG_INA226_FAULT, 'INA226故障'),
    (FLAG_WDT_REBOOT, '看門狗重開'),
    (FLAG_US_TIMEOUT, '超音波逾時'),
    (FLAG_USB_DROP, 'USB輸出丟棄'),
    (FLAG_INA226_CAL_ERR, 'INA226校正失敗'),
    (FLAG_BAD_CMD, '指令異常'),
    (FLAG_US_CH_FAULT, '通道故障'),
]


class ChecksumError(ValueError):
    """XOR 校驗不符或行格式不成立。"""


def xor_checksum(payload: str) -> int:
    """payload 每個位元組的 XOR。校驗範圍不含 '$' 與 '*'。"""
    x = 0
    for b in payload.encode('utf-8', errors='replace'):
        x ^= b
    return x


def parse_line(line: str) -> str:
    """驗證一行並回傳 payload（不含 '$' 與 '*XX'）。

    格式錯誤或校驗不符一律拋 ChecksumError —— 兩者對消費端是同一件事：
    這行不可信，丟掉。
    """
    line = line.strip()
    if not line.startswith('$'):
        raise ChecksumError('缺少起始 $: %r' % line)
    star = line.rfind('*')
    if star < 0 or star + 3 != len(line):
        raise ChecksumError('缺少或位置錯誤的校驗欄: %r' % line)
    payload = line[1:star]
    try:
        want = int(line[star + 1:], 16)
    except ValueError:
        raise ChecksumError('校驗欄非十六進位: %r' % line)
    if xor_checksum(payload) != want:
        raise ChecksumError('校驗不符: %r' % line)
    return payload


def build_line(payload: str) -> bytes:
    """組出可送給 Pico 的一行（自動補 XOR 校驗）。"""
    return ('$%s*%02X\n' % (payload, xor_checksum(payload))).encode('ascii')


def distance_mm_to_range_m(mm: int) -> float:
    """把韌體的距離值換成 sensor_msgs/Range 的 range（公尺）。

    🔴 這是本 package 最高風險的一行。哨兵值**絕不可以變成 0.0**——
    0 在 Range 語意裡是「障礙物貼在感測器上」，會讓下游避障誤觸發急停。
    映射依 REP-117（感測器讀值的無效值表示法）：

    | 韌體值 | 意義                     | Range.range |
    |--------|--------------------------|-------------|
    | -1     | 逾時無回波（含未接線）   | +Inf        |
    | -2     | 通道故障（ECHO 卡高）    | NaN         |
    | 其他   | 實際距離 mm              | mm / 1000.0 |

    -1 用 +Inf 是因為「量測有效、但範圍內沒東西」＝偵測不到障礙，
    對避障而言等於無限遠；-2 是量測本身壞掉，什麼都不知道，只能給 NaN。
    """
    if mm == DIST_TIMEOUT:
        return math.inf
    if mm == DIST_FAULT:
        return math.nan
    if mm < 0:
        # 未知的負值＝未來韌體新增的哨兵。當成「不知道」而不是距離，
        # 千萬不要讓它掉進下面的 /1000 變成一個很近的負距離。
        return math.nan
    return mm / 1000.0


def parse_us(fields) -> dict:
    """$US → {'seq': int, 'distances_mm': [8 個 int], 'ranges_m': [8 個 float]}"""
    if len(fields) != 10:
        raise ValueError('$US 欄位數應為 10，實際 %d' % len(fields))
    seq = int(fields[1])
    mm = [int(x) for x in fields[2:10]]
    return {
        'seq': seq,
        'distances_mm': mm,
        'ranges_m': [distance_mm_to_range_m(v) for v in mm],
    }


def parse_pw(fields) -> dict:
    """$PW → {'seq', 'ok', 'bus_v', 'current_a', 'power_w'}（韌體 v0.2.0 六欄）

    ok=0 時電氣量測欄一律回 None，不讓 -1 這個哨兵值有機會被當成
    「-1 mV / -1 mA」流到下游。
    """
    if len(fields) != 6:
        # v0.1.x 的五欄格式沒有 ok 欄，硬解會把 bus_mV 當成 ok。
        # 寧可整行丟掉並讓校驗統計叫出來，也不要靜默地送出錯誤電壓。
        raise ValueError('$PW 欄位數應為 6（韌體 v0.2.0+），實際 %d' % len(fields))
    seq = int(fields[1])
    ok = int(fields[2]) != 0
    if not ok:
        return {'seq': seq, 'ok': False,
                'bus_v': None, 'current_a': None, 'power_w': None}
    return {
        'seq': seq,
        'ok': True,
        'bus_v': int(fields[3]) / 1000.0,
        'current_a': int(fields[4]) / 1000.0,   # 放電為正、充電為負
        'power_w': int(fields[5]) / 1000.0,
    }


def parse_st(fields) -> dict:
    """$ST → {'uptime_s', 'flags', 'us_timeout_count', 'i2c_err_count'}"""
    if len(fields) != 5:
        raise ValueError('$ST 欄位數應為 5，實際 %d' % len(fields))
    return {
        'uptime_s': int(fields[1]) / 1000.0,
        'flags': int(fields[2], 16),
        'us_timeout_count': int(fields[3]),
        'i2c_err_count': int(fields[4]),
    }


def parse_id(fields) -> dict:
    """$ID → {'fw_version', 'build_date', 'board'}"""
    if len(fields) != 4:
        raise ValueError('$ID 欄位數應為 4，實際 %d' % len(fields))
    return {'fw_version': fields[1], 'build_date': fields[2], 'board': fields[3]}


def format_flags(flags: int) -> str:
    """flags 位元 → 人看得懂的字串，log 用。"""
    if flags == 0:
        return '無'
    names = [name for bit, name in FLAG_NAMES if flags & bit]
    unknown = flags & ~sum(bit for bit, _ in FLAG_NAMES)
    if unknown:
        names.append('未知位元 0x%X' % unknown)
    return ' | '.join(names)


def parse_message(line: str):
    """一行 → (kind, data)。校驗失敗拋 ChecksumError，欄位不對拋 ValueError。

    無法辨識的 kind（$ACK/$NAK/$JT/$I2C…）回 (kind, {'payload': ...})，
    不當成錯誤——它們是除錯指令的回應，正常運轉不會出現但也無害。
    """
    payload = parse_line(line)
    fields = payload.split(',')
    kind = fields[0]
    if kind == 'US':
        return kind, parse_us(fields)
    if kind == 'PW':
        return kind, parse_pw(fields)
    if kind == 'ST':
        return kind, parse_st(fields)
    if kind == 'ID':
        return kind, parse_id(fields)
    return kind, {'payload': payload}
