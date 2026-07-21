"""模式切換端點 🟡（本專案擴充，見文件 §7）。

規格定義了 ``explore`` / ``navigate`` 兩種 op_mode 與 ``switch_mode`` 事件，
卻沒有切換模式的端點，因此在此補上。
"""

import asyncio

from fastapi import APIRouter

from .. import errors
from ..logging_config import get_logger
from ..models import EventCode, ModeRequest, OpMode, WsEvent
from ..ros_bridge import NavStatus, ProcessError, SlamStatus, handle_navigation_down, state
from ..store import map_exists, sanitize_map_name
from ..ws_server import hub

logger = get_logger(__name__)

router = APIRouter(tags=["mode"])

EXTENSION = {"x-extension": True}


def _switch(mode: OpMode, map_name) -> None:
    """在工作執行緒中執行的同步切換（子程序生命週期操作）"""
    if mode == OpMode.EXPLORE:
        if state.slam_status == SlamStatus.MAPPING:
            return
        state.start_slam()
        state.current_map = None
    else:
        if state.nav_status == NavStatus.RUNNING:
            return
        state.start_navigation(map_name)


@router.post("/mode", openapi_extra=EXTENSION)
async def switch_mode(request: ModeRequest) -> dict:
    """POST /v1/robot/mode 🟡 — 切換模式，完成後推播 ``switch_mode`` 事件"""
    if state.is_busy:
        raise errors.ApiError(errors.ROBOT_BUSY)

    map_name = None
    if request.mode == OpMode.NAVIGATE:
        map_name = sanitize_map_name(request.map) if request.map else state.current_map
        if not map_name or not map_exists(map_name):
            raise errors.ApiError(errors.MAP_NOT_FOUND)

    try:
        await asyncio.to_thread(_switch, request.mode, map_name)
    except ProcessError as e:
        logger.error(f"Mode switch failed: {e}")
        await hub.emit_event_async(WsEvent.SWITCH_MODE, EventCode.ABORT)
        raise errors.ApiError(errors.ROBOT_BUSY)

    if request.mode == OpMode.EXPLORE:
        # 導航已被停止，重置 navigator
        await asyncio.to_thread(handle_navigation_down)

    await hub.emit_event_async(WsEvent.SWITCH_MODE, EventCode.COMPLETE)
    return {"mode": request.mode.value, "map": map_name}
