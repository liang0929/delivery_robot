"""Winstec Robot API v1.1 — app 組裝與啟動。

- REST server：port 5000（本檔）
- WebSocket server：port 5001（``ws_server.py``，獨立 port，非 FastAPI 的 /ws）

所有 REST 路徑前綴 ``/v1/robot``。
"""

import asyncio
from contextlib import asynccontextmanager
from typing import Set

import uvicorn
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from . import errors, ros_bridge
from .config import (
    ALLOW_CREDENTIALS,
    ALLOWED_ORIGINS,
    BIND_HOST,
    HTTP_PORT,
    WS_PORT,
)
from .logging_config import get_logger, setup_logging
from .models import EventCode, WsEvent
from .ros_bridge import bridge, mission, nav_manager, state
from .routers import edits, groups, maps, mode, points, robot, virtual_walls
from .store import store
from .ws_server import hub

setup_logging()
logger = get_logger(__name__)

API_PREFIX = "/v1/robot"

# event loop 對 task 只持弱引用，fire-and-forget 的 task 必須保留強引用
_background_tasks: Set[asyncio.Task] = set()


def spawn_background_task(coro) -> asyncio.Task:
    task = asyncio.create_task(coro)
    _background_tasks.add(task)
    task.add_done_callback(_background_tasks.discard)
    return task


async def _mission_monitor_loop() -> None:
    """監控導航任務並推播 ``go_point`` / ``go_charging`` 事件。

    goal 世代握手：``awaiting_goal`` 期間 ``isTaskComplete()`` 反映的是
    上一段航程的結果，不得據此判定完成。
    """
    while True:
        try:
            await asyncio.sleep(0.5)
            kind, generation, awaiting = mission.snapshot()
            if kind is None or awaiting:
                continue
            if not nav_manager.is_ready:
                continue
            if not await asyncio.to_thread(nav_manager.is_task_complete):
                continue

            result = await asyncio.to_thread(nav_manager.get_result)
            code = nav_manager.result_to_event_code(result)
            finished = mission.finish(generation)
            if finished is None:
                continue  # 世代已過期（例如被新的目標取代）
            event = WsEvent.GO_CHARGING if finished == 'charging' else WsEvent.GO_POINT
            logger.info(f"Mission finished: {finished} → {code.value}")
            await hub.emit_event_async(event, code)
        except asyncio.CancelledError:
            raise
        except Exception as e:
            logger.error(f"Mission monitor error: {e}")


@asynccontextmanager
async def lifespan(app: FastAPI):
    # store 需要知道「目前載入的地圖」才能解析省略 map 的請求
    store.set_current_map_provider(ros_bridge.current_map)

    state.start_health_monitor()
    bridge.start()

    hub.set_info_provider(robot.build_robot_info)
    ws_started = await hub.start(BIND_HOST, WS_PORT)
    if not ws_started:
        logger.error(f"WebSocket server failed to start on port {WS_PORT}")

    monitor_task = spawn_background_task(_mission_monitor_loop())

    try:
        yield
    finally:
        monitor_task.cancel()
        try:
            await monitor_task
        except asyncio.CancelledError:
            pass
        await hub.stop()
        bridge.stop()
        state.cleanup()


def create_app() -> FastAPI:
    app = FastAPI(
        title="Winstec Robot API",
        version="1.1",
        description=(
            "Winstec Robot API v1.1。標記 x-extension 的端點為本專案擴充，非規格內容。"
        ),
        lifespan=lifespan,
    )

    logger.info(f"CORS allowed origins: {ALLOWED_ORIGINS}, credentials: {ALLOW_CREDENTIALS}")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=ALLOWED_ORIGINS,
        allow_credentials=ALLOW_CREDENTIALS,
        allow_methods=["GET", "POST", "PATCH", "DELETE", "OPTIONS"],
        allow_headers=["*"],
    )

    errors.register_exception_handlers(app)

    # 🟢 規格端點
    app.include_router(robot.router, prefix=API_PREFIX)
    app.include_router(points.router, prefix=API_PREFIX)
    app.include_router(virtual_walls.router, prefix=API_PREFIX)
    app.include_router(groups.router, prefix=API_PREFIX)
    app.include_router(edits.router, prefix=API_PREFIX)
    # 🟡 本專案擴充
    app.include_router(mode.router, prefix=API_PREFIX)
    app.include_router(maps.router, prefix=API_PREFIX)

    return app


app = create_app()


def main():
    uvicorn.run(app, host=BIND_HOST, port=HTTP_PORT)


if __name__ == '__main__':
    main()


__all__ = ['app', 'create_app', 'main', 'EventCode', 'WsEvent']
