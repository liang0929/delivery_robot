"""Winstec Robot API v1.1 — app 組裝與啟動。

- REST server：port 5000（本檔）
- WebSocket server：port 5001（``ws_server.py``，獨立 port，非 FastAPI 的 /ws）

所有 REST 路徑前綴 ``/v1/robot``。
"""

from contextlib import asynccontextmanager

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
    settings,
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


@asynccontextmanager
async def lifespan(app: FastAPI):
    # 啟動期才碰觸檔案系統，而非 import 期的副作用
    settings.ensure_map_path()

    # store 需要知道「目前載入的地圖」才能解析省略 map 的請求
    store.set_current_map_provider(ros_bridge.current_map)

    state.start_health_monitor()
    bridge.start()

    hub.set_info_provider(robot.build_robot_info)
    ws_started = await hub.start(BIND_HOST, WS_PORT)
    if not ws_started:
        logger.error(f"WebSocket server failed to start on port {WS_PORT}")

    # 導航任務的完成偵測迴圈由 MissionTracker 自己擁有生命週期，
    # 這裡只負責啟動/停止（推播交給 hub.emit_event_async 這個既有 callback）
    mission.start_monitor(nav_manager, hub.emit_event_async)

    try:
        yield
    finally:
        await mission.stop_monitor()
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
