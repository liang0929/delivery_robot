"""composition root：組裝 ``state`` / ``bridge`` / ``nav_manager`` / ``mission``
四個單例，並提供對 router 的高階模組函式（navigate_to / stop_motion /
save_map / robot_status / current_map / shutdown_system）。

``process_manager.py``、``navigator.py``、``bridge_node.py``、``mission.py``
四個模組拆分後只定義類別、不在各自模組內建立全域單例，避免任何一個模組
需要 import 另一個模組的單例而形成循環 import。單例的組裝順序與跨模組的
硬相依（``NavigatorManager`` 需要 ``bridge.is_localized``、``RosBridge``
需要在 e-stop 變化時更新 ``state.e_stop_active``）改為建構子注入，統一在
本模組（唯一的 composition root）完成，行為與原本 ``ros_bridge.py``
逐字相同，只是「怎麼拿到對方」從讀模組全域變成注入。
"""

import os
import subprocess
from typing import Optional

from .bridge_node import RosBridge
from .config import MAP_PATH, WORKSPACE_ROOT
from .conversions import cm_to_m, deg_to_yaw
from .logging_config import get_logger
from .mission import MissionTracker
from .models import Location, RobotStatus
from .navigator import GoalRejectedError, Nav2NotReadyError, NavigatorManager
from .process_manager import NavStatus, ProcessError, RobotStateManager, SlamStatus
from .robot_service import RobotService

logger = get_logger(__name__)


# --- 單例組裝（composition root）---
# 組裝順序刻意固定：state → bridge（注入 state 的 e_stop setter）→
# nav_manager（注入 bridge.is_localized）→ mission（無相依）。
state = RobotStateManager()
bridge = RosBridge(on_e_stop_changed=lambda active: setattr(state, 'e_stop_active', active))
nav_manager = NavigatorManager(is_localized_fn=bridge.is_localized)
mission = MissionTracker()


def handle_navigation_down() -> None:
    """導航停止或崩潰後的共用善後"""
    try:
        nav_manager.reset()
    except Exception as e:
        logger.warning(f"Navigator reset failed: {e}")
    mission.abort()


state.on_navigation_down = handle_navigation_down


# --- 對 router 的高階介面 ---
def robot_status() -> RobotStatus:
    """依子系統狀態推導規格 §7.2 的 Robot Status"""
    if state.is_busy:
        return RobotStatus.SWITCHING_MODE
    if state.slam_status != SlamStatus.IDLE:
        return RobotStatus.IDLE
    if state.nav_status != NavStatus.RUNNING:
        return RobotStatus.INIT
    kind, _, _ = mission.snapshot()
    if kind == 'charging':
        return RobotStatus.GO_CHARGING
    if kind == 'point':
        return RobotStatus.MOVING
    return RobotStatus.IDLE


def current_map() -> Optional[str]:
    return state.current_map


def navigate_to(location: Location, kind: str = 'point') -> int:
    """送出導航目標（阻塞，須以 asyncio.to_thread 包裝）。回傳 mission 世代。"""
    generation = mission.begin(kind)
    if not nav_manager.ensure_nav2_ready():
        mission.abort(generation)
        raise Nav2NotReadyError(
            "Nav2 未就緒；請先設定初始位姿（relocate）讓 AMCL 完成定位"
        )
    try:
        nav_manager.send_goal(
            cm_to_m(location.x), cm_to_m(location.y), deg_to_yaw(location.orientation)
        )
    except Exception:
        mission.abort(generation)
        raise
    mission.dispatched(generation)
    return generation


def stop_motion() -> None:
    """軟停止：取消導航目標並歸零 cmd_vel"""
    mission.abort()
    bridge.stop_motion()
    nav_manager.cancel_task()


def save_map(map_name: str) -> None:
    """用 nav2_map_server 存下目前建圖結果（阻塞）"""
    map_path = os.path.join(MAP_PATH, map_name)
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {WORKSPACE_ROOT}/install/setup.bash && "
        f"ros2 run nav2_map_server map_saver_cli -f {map_path} -t /map_saver "
        f"--ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=10000.0"
    )
    state.begin_map_save()
    try:
        result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=30)
        if result.returncode != 0:
            raise ProcessError(f"Map save failed: {result.stderr.strip()[:200]}")
    except subprocess.TimeoutExpired:
        raise ProcessError("Map save timed out")
    finally:
        state.end_map_save()


def shutdown_system() -> None:
    """執行系統關機（POST /shutdown，事件先送）"""
    try:
        subprocess.Popen(["sudo", "shutdown", "-h", "now"])
    except Exception as e:
        logger.error(f"Failed to shut down: {e}")
        raise ProcessError(str(e))


# --- RobotService（意圖層 facade）組裝 ---
# 依賴上面已組好的四個單例與模組函式，必須排在它們之後組裝。
robot_service = RobotService(
    state, bridge, mission, nav_manager,
    navigate_to_fn=navigate_to,
    stop_motion_fn=stop_motion,
    save_map_fn=save_map,
    robot_status_fn=robot_status,
    shutdown_system_fn=shutdown_system,
    handle_navigation_down_fn=handle_navigation_down,
)


__all__ = [
    'state', 'bridge', 'nav_manager', 'mission', 'robot_service',
    'handle_navigation_down',
    'robot_status', 'current_map', 'navigate_to', 'stop_motion', 'save_map',
    'shutdown_system',
    'GoalRejectedError', 'Nav2NotReadyError', 'ProcessError',
    'NavStatus', 'SlamStatus',
]
