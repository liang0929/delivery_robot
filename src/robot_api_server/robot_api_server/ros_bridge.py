"""rclpy 橋接層 —— **相容 shim**。

原本 1400+ 行的 ``ros_bridge.py`` 已拆成數個模組：

- ``ros_common.py``      — rclpy 全域初始化 + ROS 套件降級 import
- ``process_manager.py`` — ``RobotStateManager`` / ``ManagedProcess`` / ``ProcessError``
- ``navigator.py``       — ``NavigatorManager`` / ``Nav2NotReadyError`` / ``GoalRejectedError``
- ``mission.py``         — ``MissionTracker``
- ``bridge_node.py``     — ``RosBridge`` 節點
- ``ros_facade.py``      — composition root：組裝 ``state``/``bridge``/
  ``nav_manager``/``mission`` 四個單例、對 router 的高階函式
  （``navigate_to``/``stop_motion``/``save_map``/...）
- ``robot_service.py``   — ``RobotService`` 意圖層 facade

本檔案保留純粹是為了向後相容：既有 ``from ..ros_bridge import state,
bridge, mission, nav_manager, NavStatus, ...`` 或 ``ros_bridge.OpMode``
這類散落各處的用法，全部原封不動可用，不要求同步改所有 import 點。
新程式碼請直接從對應的子模組 import。
"""

from .bridge_node import RosBridge
from .conversions import (
    cm_to_m, deg_to_yaw, m_to_cm, quaternion_to_yaw, voltage_to_battery, yaw_to_deg,
    yaw_to_quaternion,
)
from .mission import MissionTracker
from .models import Direction, EventCode, Location, OpMode, RobotStatus, WsEvent
from .navigator import GoalRejectedError, Nav2NotReadyError, NavigatorManager
from .process_manager import (
    ManagedProcess, NavStatus, ProcessError, RobotStateManager, SlamStatus,
)
from .robot_service import RobotService
from .ros_common import (
    NAV2_AVAILABLE, ROS_AVAILABLE, ROS_IMPORT_ERROR, ensure_rclpy_initialized,
)
from .ros_facade import (
    bridge, current_map, handle_navigation_down, mission, nav_manager, navigate_to,
    robot_service, robot_status, save_map, shutdown_system, state, stop_motion,
)

__all__ = [
    # --- 單例 ---
    'state', 'bridge', 'mission', 'nav_manager', 'robot_service',
    # --- 類別 / 例外 ---
    'RobotStateManager', 'ManagedProcess', 'ProcessError',
    'NavigatorManager', 'Nav2NotReadyError', 'GoalRejectedError',
    'MissionTracker', 'RosBridge', 'RobotService',
    # --- 列舉 ---
    'SlamStatus', 'NavStatus', 'OpMode', 'RobotStatus', 'Direction', 'EventCode', 'WsEvent',
    # --- 對 router 的高階介面（facade 函式）---
    'robot_status', 'current_map', 'navigate_to', 'stop_motion', 'save_map',
    'shutdown_system', 'handle_navigation_down',
    # --- ROS 全域初始化 / 降級旗標 ---
    'ROS_AVAILABLE', 'ROS_IMPORT_ERROR', 'NAV2_AVAILABLE', 'ensure_rclpy_initialized',
    # --- 座標換算 ---
    'cm_to_m', 'deg_to_yaw', 'm_to_cm', 'quaternion_to_yaw', 'voltage_to_battery',
    'yaw_to_deg', 'yaw_to_quaternion',
]
