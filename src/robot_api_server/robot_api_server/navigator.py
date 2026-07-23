"""Nav2 ``BasicNavigator`` 的生命週期管理（執行緒安全）。

拆分自 ``ros_bridge.py``，逐字搬移。唯一的行為調整（依重構任務指示做的
低風險依賴注入）：``ensure_nav2_ready`` 原本直接呼叫模組全域單例
``bridge.is_localized()``；現在改為建構子注入的 ``is_localized_fn``
callable，由 composition root（``ros_facade.py``）在組裝單例時綁定
``bridge.is_localized``。呼叫時機、判準、逾時邏輯完全不變，只是「怎麼
拿到這個判準」從讀模組全域變成呼叫注入的 callable——避免本模組對
``bridge_node.py`` 產生 import 期的硬相依（兩者互不 import，組裝順序
只在 composition root 決定，無循環 import 風險）。
"""

import math
import threading
import time
from typing import Callable, Optional

from .config import NAV2_READY_TIMEOUT_SEC
from .conversions import yaw_to_quaternion
from .logging_config import get_logger
from .models import EventCode
from .ros_common import (
    NAV2_AVAILABLE, BasicNavigator, PoseStamped, TaskResult, ensure_rclpy_initialized,
)

logger = get_logger(__name__)


class Nav2NotReadyError(RuntimeError):
    """Nav2 未就緒（通常是 AMCL 尚未定位，或 navigator 於就緒核對後又被重置，
    例如導航行程於檢查與送出目標之間崩潰）。與其他導航失敗區分，讓端點能
    回傳語意正確的錯誤碼而非 ROBOT_BUSY。"""


class GoalRejectedError(RuntimeError):
    """導航目標被 Nav2 action server 拒絕（例如 bt_navigator 尚未進入 active
    狀態）。這與「機器人忙碌」是不同語意，端點應回傳語意正確的錯誤碼而非
    ROBOT_BUSY。"""


class NavigatorManager:
    """BasicNavigator 的生命週期管理（執行緒安全）"""

    def __init__(self, is_localized_fn: Optional[Callable[[], bool]] = None):
        self.navigator = None
        self._nav2_ready = False
        self._lock = threading.Lock()
        # 序列化就緒檢查：避免多個執行緒同時對同一個 navigator node spin
        self._ready_lock = threading.Lock()
        # 建構子注入：AMCL 是否已定位的判準，由 composition root 綁定
        # bridge.is_localized；未注入時保守回傳 False（維持原本「查不到就
        # 視為未定位」的安全預設）。
        self._is_localized: Callable[[], bool] = is_localized_fn or (lambda: False)

    @property
    def is_ready(self) -> bool:
        with self._lock:
            return self._nav2_ready and self.navigator is not None

    def ensure_nav2_ready(self) -> bool:
        if not NAV2_AVAILABLE:
            return False
        with self._lock:
            if self._nav2_ready and self.navigator is not None:
                return True

        # 慢路徑整段以 _ready_lock 序列化（併發 spin 同一個 node 會拋錯）
        with self._ready_lock:
            with self._lock:
                if self._nav2_ready and self.navigator is not None:
                    return True
                if self.navigator is None:
                    if not ensure_rclpy_initialized():
                        logger.error("Failed to initialize rclpy")
                        return False
                    logger.info("Creating BasicNavigator...")
                    try:
                        self.navigator = BasicNavigator()
                    except Exception as e:
                        logger.error(f"Failed to create BasicNavigator: {e}")
                        return False
                nav = self.navigator

            # 就緒判定：刻意「不」使用 BasicNavigator.waitUntilNav2Active()。
            #
            # 該函式的 _waitForInitialPose() 有兩個會直接毀掉定位的行為：
            #   1. 迴圈中不斷呼叫 _setInitialPose()，發布的是 BasicNavigator 自己
            #      的 initial_pose——未經 setInitialPose() 設定時是全零、frame_id
            #      為空字串的訊息，會把使用者手動指定的位姿覆蓋掉。
            #   2. 它等待 /amcl_pose，而 AMCL 只在濾波器更新時才發布，更新又需要
            #      機器人移動超過 update_min_d（0.25 m）。靜止的機器人永遠等不到，
            #      於是無限迴圈。
            #
            # 改為直接檢查真正的就緒條件：bt_navigator 已 active、且 map→odom
            # 存在（後者是 AMCL 完成定位的唯一可靠證據）。
            deadline = time.time() + NAV2_READY_TIMEOUT_SEC

            # 直接等待 NavigateToPose action server——這正是送目標需要的東西，
            # 比輪詢 lifecycle 狀態更直接，也不必反覆 spawn ros2 CLI。
            if not nav.nav_to_pose_client.wait_for_server(
                timeout_sec=max(1.0, deadline - time.time())
            ):
                logger.error(
                    "NavigateToPose action server 未就緒；"
                    "bt_navigator 可能未進入 active（常見原因是行為樹載入失敗）"
                )
                return False

            while time.time() < deadline:
                if self._is_localized():
                    break
                time.sleep(0.3)
            else:
                logger.error(
                    "Nav2 已啟動但 AMCL 未定位（map→odom 不存在）；"
                    "請先在前端指定機器人在地圖上的實際位置"
                )
                return False

            with self._lock:
                if self.navigator is not nav:
                    return False  # 等待期間被 reset
                self._nav2_ready = True
            logger.info("Nav2 已就緒")
            return True

    def reset(self):
        """導航進程停止或崩潰後呼叫。

        不使用 lifecycleShutdown()——它會對 nav2 lifecycle 服務發請求，
        在 nav2 已死亡時可能無限期阻塞；進程本身由 RobotStateManager 終止。
        """
        with self._lock:
            self._nav2_ready = False
            nav = self.navigator
            self.navigator = None
        if nav is not None:
            try:
                nav.destroy_node()
            except Exception as e:
                logger.warning(f"Error destroying navigator node: {e}")

    def send_goal(self, x_m: float, y_m: float, yaw_rad: float) -> None:
        with self._lock:
            if self.navigator is None:
                # 呼叫端已透過 ensure_nav2_ready() 核對過就緒；navigator 仍為
                # None 代表在核對之後、送出目標之前被併發重置（例如導航行程
                # 於此期間崩潰）——這是「Nav2 未就緒」而非「機器人忙碌」。
                raise Nav2NotReadyError(
                    "Navigator 尚未初始化（可能於就緒核對後被併發重置）"
                )
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = x_m
            goal_pose.pose.position.y = y_m
            qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)
            goal_pose.pose.orientation.z = qz
            goal_pose.pose.orientation.w = qw
            logger.info(f"Sending goal: x={x_m:.3f}, y={y_m:.3f}, yaw={math.degrees(yaw_rad):.1f}")
            # goToPose 在目標被 action server 拒絕時回傳 False（例如 bt_navigator
            # 尚未 activate）。忽略回傳值的話，isTaskComplete() 會因為沒有
            # result_future 而立刻回 True，任務被誤判成 STUCK——錯誤訊息會指向
            # 導航失敗，而真正的原因是目標從未被接受。
            if self.navigator.goToPose(goal_pose) is False:
                raise GoalRejectedError(
                    "導航目標被拒絕；bt_navigator 可能未進入 active 狀態"
                )

    def is_task_complete(self) -> bool:
        with self._lock:
            if self.navigator is None or not self._nav2_ready:
                return True
            try:
                return self.navigator.isTaskComplete()
            except Exception as e:
                # 無法判定時回 False，避免被誤判為「已完成」而錯誤推進狀態機
                logger.warning(f"Task complete check failed: {e}")
                return False

    def get_result(self) -> Optional["TaskResult"]:
        with self._lock:
            if self.navigator is None or not self._nav2_ready:
                return None
            try:
                return self.navigator.getResult()
            except Exception as e:
                logger.debug(f"Get result failed: {e}")
                return None

    def cancel_task(self):
        with self._lock:
            # 未就緒時沒有可取消的目標；且此時可能有執行緒正在
            # 就緒檢查與 cancel 會 spin 同一個 node，不可併發
            if self.navigator is not None and self._nav2_ready:
                try:
                    self.navigator.cancelTask()
                except Exception as e:
                    logger.warning(f"Cancel task failed: {e}")

    def result_to_event_code(self, result) -> EventCode:
        if not NAV2_AVAILABLE or result is None:
            return EventCode.ABORT
        if result == TaskResult.SUCCEEDED:
            return EventCode.COMPLETE
        if result == TaskResult.CANCELED:
            return EventCode.ABORT
        return EventCode.STUCK


__all__ = ['Nav2NotReadyError', 'GoalRejectedError', 'NavigatorManager']
