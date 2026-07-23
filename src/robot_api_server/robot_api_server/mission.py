"""導航任務的世代握手，並擁有完成偵測迴圈（監控 task）的完整生命週期。

拆分自 ``ros_bridge.py``，逐字搬移。純 asyncio 狀態機，不依賴任何 ROS
套件；``nav_manager`` 與 ``on_finished`` 皆以參數傳入，本模組原本就沒有
跨模組硬相依，維持原樣。
"""

import asyncio
import threading
from typing import Awaitable, Callable, Optional, Tuple

from .logging_config import get_logger
from .models import EventCode, WsEvent

logger = get_logger(__name__)


class MissionTracker:
    """導航任務的世代握手，並擁有完成偵測迴圈（監控 task）的完整生命週期。

    狀態推進（begin）到 goal 實際送出（dispatched）之間，
    ``isTaskComplete()`` 反映的是上一段航程的結果，不得據此判定完成。
    """

    #: 完成偵測迴圈的輪詢間隔
    MONITOR_INTERVAL_SEC = 0.5

    def __init__(self):
        self._lock = threading.Lock()
        self._kind: Optional[str] = None       # "point" | "charging" | None
        self._generation = 0
        self._awaiting_goal = False
        self._monitor_task: Optional["asyncio.Task"] = None

    def begin(self, kind: str) -> int:
        with self._lock:
            self._generation += 1
            self._kind = kind
            self._awaiting_goal = True
            return self._generation

    def dispatched(self, generation: int) -> None:
        with self._lock:
            if generation == self._generation:
                self._awaiting_goal = False

    def abort(self, generation: Optional[int] = None) -> None:
        with self._lock:
            if generation is not None and generation != self._generation:
                return
            self._kind = None
            self._awaiting_goal = False

    def snapshot(self) -> Tuple[Optional[str], int, bool]:
        with self._lock:
            return self._kind, self._generation, self._awaiting_goal

    def finish(self, generation: int) -> Optional[str]:
        """把任務標記為完成，回傳其 kind（若世代已過期則回 None）"""
        with self._lock:
            if generation != self._generation or self._kind is None:
                return None
            kind = self._kind
            self._kind = None
            self._awaiting_goal = False
            return kind

    @property
    def active(self) -> bool:
        with self._lock:
            return self._kind is not None

    # --- 完成偵測 driver（監控迴圈；生命週期由本物件自行擁有）---
    async def _monitor_loop(
        self,
        nav_manager: "NavigatorManager",
        on_finished: Callable[["WsEvent", "EventCode"], Awaitable[None]],
    ) -> None:
        """監控導航任務並在完成時呼叫 ``on_finished(event, code)``。

        goal 世代握手：``awaiting_goal`` 期間 ``isTaskComplete()`` 反映的是
        上一段航程的結果，不得據此判定完成。
        """
        while True:
            try:
                await asyncio.sleep(self.MONITOR_INTERVAL_SEC)
                kind, generation, awaiting = self.snapshot()
                if kind is None or awaiting:
                    continue
                if not nav_manager.is_ready:
                    continue
                if not await asyncio.to_thread(nav_manager.is_task_complete):
                    continue

                result = await asyncio.to_thread(nav_manager.get_result)
                code = nav_manager.result_to_event_code(result)
                finished = self.finish(generation)
                if finished is None:
                    continue  # 世代已過期（例如被新的目標取代）
                event = WsEvent.GO_CHARGING if finished == 'charging' else WsEvent.GO_POINT
                logger.info(f"Mission finished: {finished} → {code.value}")
                await on_finished(event, code)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                logger.error(f"Mission monitor error: {e}")

    def start_monitor(
        self,
        nav_manager: "NavigatorManager",
        on_finished: Callable[["WsEvent", "EventCode"], Awaitable[None]],
    ) -> None:
        """啟動完成偵測迴圈（背景 task，強引用由本物件持有）。"""
        if self._monitor_task is not None:
            return
        self._monitor_task = asyncio.create_task(self._monitor_loop(nav_manager, on_finished))

    async def stop_monitor(self) -> None:
        """停止完成偵測迴圈並等待其結束。"""
        task = self._monitor_task
        self._monitor_task = None
        if task is None:
            return
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass


__all__ = ['MissionTracker']
