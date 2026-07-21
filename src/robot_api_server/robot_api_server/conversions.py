"""座標與角度換算（文件 §2）。

API 對外一律使用「公分整數 + 角度（度）」，ROS 內部使用「公尺浮點 + 弧度」。
換算只在與 ROS 互動的邊界發生。
"""

import math

#: ros 公尺 → api 整數（公分）
COORD_SCALE = 100


def m_to_cm(value_m: float) -> int:
    """公尺（float）→ 公分（int）"""
    return int(round(value_m * COORD_SCALE))


def cm_to_m(value_cm: int) -> float:
    """公分（int）→ 公尺（float）"""
    return float(value_cm) / COORD_SCALE


def yaw_to_deg(yaw_rad: float) -> float:
    """ros yaw（弧度）→ api orientation（度，0–360）"""
    return math.degrees(yaw_rad) % 360.0


def deg_to_yaw(deg: float) -> float:
    """api orientation（度）→ ros yaw（弧度，正規化到 -pi..pi）"""
    r = math.radians(deg)
    return math.atan2(math.sin(r), math.cos(r))


def yaw_to_quaternion(yaw_rad: float) -> tuple:
    """yaw（弧度）→ (x, y, z, w)"""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """四元數 → yaw（弧度）"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def voltage_to_battery(voltage: float, min_v: float, max_v: float) -> int:
    """電壓 → 電量百分比（文件 §9）"""
    if max_v <= min_v:
        return 0
    ratio = (voltage - min_v) / (max_v - min_v)
    return int(round(max(0.0, min(1.0, ratio)) * 100))
