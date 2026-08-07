"""``RobotService``：意圖層 facade，收斂 router 對底層單例
（``state`` / ``bridge`` / ``mission`` / ``nav_manager``）的直接存取。

依重構任務指示，只有 ``routers/robot.py`` 與 ``routers/mode.py``（對低階
狀態操作最直接的兩個 router）遷移為透過本 service 呼叫；其餘 router
（``points.py`` 只用 ``bridge.location()`` 取當前位置、``maps.py`` 是地圖
檔案管理而非機器人移動意圖）維持原樣直接使用 ``ros_bridge`` 匯出的名稱，
於重構回報中說明、不在此次擴大範圍。

錯誤轉譯（``errors.ApiError``）與 WebSocket 事件推播（``ws_server.hub``）
刻意留在 router 層——這兩者是 HTTP / 傳輸層關注點，不屬於「機器人該做
什麼」的意圖層，讓 ``RobotService`` 對 HTTP 無感知。

本類別不在模組層級建立單例；單例組裝（含它所依賴的四個底層單例與
facade 函式）統一在 composition root ``ros_facade.py`` 完成。
"""

from typing import Callable, Optional

from .bridge_node import RosBridge
from .mission import MissionTracker
from .models import Direction, Location, OpMode, RobotInfo, RobotStatus
from .navigator import NavigatorManager
from .process_manager import NavStatus, RobotStateManager, SlamStatus

#: 取不到定位時回報的 fallback（避免 /info 直接失敗），與原本
#: ``routers/robot.py`` 的 ``_UNKNOWN_LOCATION`` 逐字相同
_UNKNOWN_LOCATION = Location(x=0, y=0, orientation=0.0)


class RobotService:
    """意圖層方法集合：switch_mode / move / relocate / stop / save_map /
    status 等，包成一個物件供 router 呼叫，而非直接戳
    ``state`` / ``mission`` / ``nav_manager`` / ``bridge``。"""

    def __init__(
        self,
        state: RobotStateManager,
        bridge: RosBridge,
        mission: MissionTracker,
        nav_manager: NavigatorManager,
        *,
        navigate_to_fn: Callable[[Location, str], int],
        stop_motion_fn: Callable[[], None],
        save_map_fn: Callable[[str], None],
        robot_status_fn: Callable[[], RobotStatus],
        shutdown_system_fn: Callable[[], None],
        handle_navigation_down_fn: Callable[[], None],
    ):
        self._state = state
        self._bridge = bridge
        self._mission = mission
        self._nav_manager = nav_manager
        self._navigate_to_fn = navigate_to_fn
        self._stop_motion_fn = stop_motion_fn
        self._save_map_fn = save_map_fn
        self._robot_status_fn = robot_status_fn
        self._shutdown_system_fn = shutdown_system_fn
        self._handle_navigation_down_fn = handle_navigation_down_fn

    # --- 狀態查詢 ---
    @property
    def op_mode(self) -> OpMode:
        return self._state.op_mode()

    @property
    def nav_status(self) -> NavStatus:
        return self._state.nav_status

    @property
    def slam_status(self) -> SlamStatus:
        return self._state.slam_status

    @property
    def is_busy(self) -> bool:
        return self._state.is_busy

    @property
    def current_map(self) -> Optional[str]:
        return self._state.current_map

    def clear_current_map(self) -> None:
        self._state.current_map = None

    @property
    def mission_active(self) -> bool:
        return self._mission.active

    def is_navigation_mode_ready(self) -> bool:
        """導航模式且 nav 子程序正在跑（move / relocate 的前置條件）"""
        return self.op_mode == OpMode.NAVIGATE and self.nav_status == NavStatus.RUNNING

    def get_info(self) -> RobotInfo:
        """組出規格 §5 Robot Information / ``robot_info`` 事件的內容"""
        location = self._bridge.location() or _UNKNOWN_LOCATION
        return RobotInfo(
            op_mode=self.op_mode,
            status=self._robot_status_fn(),
            battery=self._bridge.battery(),
            voltage=self._bridge.voltage(),
            battery_state=self._bridge.battery_state(),
            battery_stop_latched=self._bridge.battery_stop_latched(),
            location=location,
        )

    # --- 移動 / 導航（阻塞方法，呼叫端須以 asyncio.to_thread 包裝）---
    def navigate_to(self, location: Location, kind: str = 'point') -> int:
        return self._navigate_to_fn(location, kind)

    def stop_motion(self) -> None:
        self._stop_motion_fn()

    def set_manual_direction(self, direction: Direction) -> None:
        self._bridge.set_manual_direction(direction)

    # --- 重定位（阻塞方法）---
    def publish_initial_pose(self, x_m: float, y_m: float, yaw_rad: float) -> bool:
        return self._bridge.publish_initial_pose(x_m, y_m, yaw_rad)

    # --- 模式切換（阻塞方法）---
    def start_slam(self) -> None:
        self._state.start_slam()

    def start_navigation(self, map_name: Optional[str]) -> str:
        return self._state.start_navigation(map_name)

    def handle_navigation_down(self) -> None:
        self._handle_navigation_down_fn()

    def wait_for_navigation_ready(self):
        return self._bridge.wait_for_navigation_ready()

    # --- 電源 / 地圖（阻塞方法）---
    def shutdown_system(self) -> None:
        self._shutdown_system_fn()

    def save_map(self, map_name: str) -> None:
        self._save_map_fn(map_name)


__all__ = ['RobotService']
