"""WebSocket server（規格 §5 Event Reference，獨立 port 5001）。

注意：這是**獨立的 WebSocket server**，不是 FastAPI 的 ``/ws`` 端點。

事件：

- ``robot_info`` — 每 1 秒推播 ``{event, op_mode, status, battery, voltage, location}``
  （``voltage`` 為 🟡 本專案擴充欄位）
- ``go_point`` / ``go_charging`` / ``switch_mode`` / ``relocate`` / ``power``
  — 事件式 ``{event, code}``，code 取自 §7.6（一律大寫）
"""

import asyncio
import json
from typing import Callable, Optional, Set

from .config import BIND_HOST, WS_PORT
from .logging_config import get_logger
from .models import EventCode, Location, RobotInfo, WsEvent

logger = get_logger(__name__)

try:
    from websockets.asyncio.server import serve as ws_serve
    WEBSOCKETS_AVAILABLE = True
except Exception:  # pragma: no cover - 舊版 websockets fallback
    try:
        from websockets.server import serve as ws_serve
        WEBSOCKETS_AVAILABLE = True
    except Exception as e:
        WEBSOCKETS_AVAILABLE = False
        logger.error(f"websockets library unavailable, WS server disabled: {e}")


class EventHub:
    """管理 WebSocket 連線並推播事件。"""

    def __init__(self):
        self._connections: Set = set()
        self._lock = asyncio.Lock()
        self._server = None
        self._tasks: Set[asyncio.Task] = set()
        self._info_provider: Optional[Callable[[], Optional[RobotInfo]]] = None

    def set_info_provider(self, provider: Callable[[], Optional[RobotInfo]]) -> None:
        """provider() -> Optional[RobotInfo]，由 main 注入以避免循環相依"""
        self._info_provider = provider

    # --- 生命週期 ---
    async def start(self, host: str = BIND_HOST, port: int = WS_PORT) -> bool:
        if not WEBSOCKETS_AVAILABLE:
            return False
        try:
            self._server = await ws_serve(self._handler, host, port)
        except OSError as e:
            logger.error(f"Failed to bind WebSocket server on {host}:{port}: {e}")
            return False
        logger.info(f"WebSocket server listening on ws://{host}:{port}")
        self._spawn(self._robot_info_loop())
        return True

    async def stop(self) -> None:
        for task in list(self._tasks):
            task.cancel()
        for task in list(self._tasks):
            try:
                await task
            except (asyncio.CancelledError, Exception):
                pass
        self._tasks.clear()
        if self._server is not None:
            self._server.close()
            try:
                await self._server.wait_closed()
            except Exception:
                pass
            self._server = None

    def _spawn(self, coro) -> asyncio.Task:
        """建立背景任務並保留強引用（event loop 對 task 只持弱引用）"""
        task = asyncio.create_task(coro)
        self._tasks.add(task)
        task.add_done_callback(self._tasks.discard)
        return task

    # --- 連線處理 ---
    async def _handler(self, websocket) -> None:
        async with self._lock:
            self._connections.add(websocket)
        logger.info(f"WebSocket client connected ({len(self._connections)} total)")
        try:
            # 連線後立即送一筆 robot_info，client 不必等滿一秒
            info = self._current_info()
            if info is not None:
                await websocket.send(json.dumps(self._info_message(info)))
            async for _ in websocket:
                pass  # 目前不處理 client 送來的訊息
        except Exception as e:
            logger.debug(f"WebSocket connection closed: {e}")
        finally:
            async with self._lock:
                self._connections.discard(websocket)

    # --- 推播 ---
    async def broadcast(self, message: dict) -> None:
        """鎖內只複製連線集合，實際送出在鎖外並行，避免單一慢速 client 拖垮廣播"""
        async with self._lock:
            connections = list(self._connections)
        if not connections:
            return
        payload = json.dumps(message)
        results = await asyncio.gather(
            *(conn.send(payload) for conn in connections), return_exceptions=True
        )
        dead = {c for c, r in zip(connections, results) if isinstance(r, Exception)}
        if dead:
            async with self._lock:
                self._connections -= dead

    def emit_event(self, event: WsEvent, code: EventCode) -> None:
        """從同步/非同步情境送出事件式訊息（fire-and-forget）"""
        message = {"event": event.value, "code": code.value}
        try:
            asyncio.get_running_loop()
        except RuntimeError:
            logger.warning(f"emit_event({event.value}) called outside event loop; dropped")
            return
        self._spawn(self.broadcast(message))

    async def emit_event_async(self, event: WsEvent, code: EventCode) -> None:
        await self.broadcast({"event": event.value, "code": code.value})

    # --- robot_info 週期推播 ---
    @staticmethod
    def _info_message(info: RobotInfo) -> dict:
        return {
            "event": WsEvent.ROBOT_INFO.value,
            "op_mode": info.op_mode.value,
            "status": info.status.value,
            "battery": info.battery,
            "voltage": info.voltage,
            "location": info.location.model_dump(),
        }

    def _current_info(self) -> Optional[RobotInfo]:
        if self._info_provider is None:
            return None
        try:
            return self._info_provider()
        except Exception as e:
            logger.error(f"robot_info provider failed: {e}")
            return None

    async def _robot_info_loop(self) -> None:
        while True:
            try:
                await asyncio.sleep(1.0)
                if not self._connections:
                    continue
                info = await asyncio.to_thread(self._current_info)
                if info is not None:
                    await self.broadcast(self._info_message(info))
            except asyncio.CancelledError:
                raise
            except Exception as e:
                logger.error(f"robot_info loop error: {e}")

    @property
    def connection_count(self) -> int:
        return len(self._connections)


#: 全域事件中樞
hub = EventHub()


__all__ = ['EventHub', 'hub', 'EventCode', 'WsEvent', 'Location']
