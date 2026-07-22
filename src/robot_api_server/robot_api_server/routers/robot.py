"""機器人資訊與移動端點 🟢（規格 §5）。"""

import asyncio

from fastapi import APIRouter, Response

from .. import errors, ros_bridge
from ..logging_config import get_logger
from ..models import (
    Direction,
    EventCode,
    Location,
    LocationResponse,
    ManualMoveRequest,
    MoveRequest,
    Point,
    PointType,
    RelocateLocationRequest,
    RobotInfo,
    WsEvent,
)
from ..ros_bridge import NavStatus, bridge, mission, nav_manager, state
from ..store import store
from ..ws_server import hub

logger = get_logger(__name__)

router = APIRouter(tags=["robot"])

#: 取不到定位時回報的 fallback（避免 /info 直接失敗）
_UNKNOWN_LOCATION = Location(x=0, y=0, orientation=0.0)


def build_robot_info() -> RobotInfo:
    """組出規格 §5 Robot Information / §5 robot_info 事件的內容（同步、可在執行緒中呼叫）"""
    location = bridge.location() or _UNKNOWN_LOCATION
    return RobotInfo(
        op_mode=state.op_mode(),
        status=ros_bridge.robot_status(),
        battery=bridge.battery(),
        voltage=bridge.voltage(),
        location=location,
    )


def _require_navigation_mode() -> None:
    if state.op_mode() != ros_bridge.OpMode.NAVIGATE or state.nav_status != NavStatus.RUNNING:
        raise errors.ApiError(errors.NOT_IN_NAVIGATION_MODE)


def _require_not_busy() -> None:
    if state.is_busy:
        raise errors.ApiError(errors.ROBOT_BUSY)


@router.get("/info", response_model=RobotInfo)
async def get_info() -> RobotInfo:
    """GET /v1/robot/info — 200"""
    return await asyncio.to_thread(build_robot_info)


@router.post("/move", response_model=MoveRequest)
async def move_to_location(request: MoveRequest) -> MoveRequest:
    """POST /v1/robot/move — 200，回傳同 body"""
    _require_not_busy()
    _require_navigation_mode()
    kind = 'charging' if request.type == PointType.CHARGE else 'point'
    try:
        await asyncio.to_thread(ros_bridge.navigate_to, request.location, kind)
    except ros_bridge.Nav2NotReadyError as e:
        # 未定位不是「忙碌」——回報語意正確的碼，讓前端能提示使用者先重定位
        logger.error(f"Navigation not ready: {e}")
        raise errors.ApiError(errors.NOT_IN_NAVIGATION_MODE)
    except RuntimeError as e:
        logger.error(f"Failed to send goal: {e}")
        raise errors.ApiError(errors.ROBOT_BUSY)
    return request


@router.post("/move/{point_id}", response_model=Point)
async def move_to_point(point_id: str) -> Point:
    """POST /v1/robot/move/{pointId} — 200，回傳完整 Point 物件"""
    _require_not_busy()
    _require_navigation_mode()
    owner, point = store.find_point(point_id)
    current = state.current_map
    if current and owner != current:
        raise errors.ApiError(errors.POINT_NOT_IN_MAP)
    kind = 'charging' if point.type == PointType.CHARGE else 'point'
    try:
        await asyncio.to_thread(ros_bridge.navigate_to, point.location, kind)
    except ros_bridge.Nav2NotReadyError as e:
        # 未定位不是「忙碌」——回報語意正確的碼，讓前端能提示使用者先重定位
        logger.error(f"Navigation not ready: {e}")
        raise errors.ApiError(errors.NOT_IN_NAVIGATION_MODE)
    except RuntimeError as e:
        logger.error(f"Failed to send goal: {e}")
        raise errors.ApiError(errors.ROBOT_BUSY)
    return point


@router.post("/manual/move")
async def manual_move(request: ManualMoveRequest) -> Response:
    """POST /v1/robot/manual/move — 200，回傳空"""
    if request.direction != Direction.STOP:
        _require_not_busy()
        # 手動控制與導航互斥：先取消目前導航目標
        if mission.active:
            await asyncio.to_thread(ros_bridge.stop_motion)
    await asyncio.to_thread(bridge.set_manual_direction, request.direction)
    return Response(status_code=200)


@router.post("/stop")
async def stop() -> Response:
    """POST /v1/robot/stop — 200，軟停止並取消導航"""
    await asyncio.to_thread(ros_bridge.stop_motion)
    return Response(status_code=200)


@router.post("/relocate/location", response_model=LocationResponse)
async def relocate_by_location(request: RelocateLocationRequest) -> LocationResponse:
    """POST /v1/robot/relocate/location — 200，回傳 {location}"""
    _require_navigation_mode()
    location = request.location
    ok = await asyncio.to_thread(
        bridge.publish_initial_pose,
        ros_bridge.cm_to_m(location.x),
        ros_bridge.cm_to_m(location.y),
        ros_bridge.deg_to_yaw(location.orientation),
    )
    await hub.emit_event_async(
        WsEvent.RELOCATE, EventCode.COMPLETE if ok else EventCode.ABORT
    )
    if not ok:
        raise errors.ApiError(errors.GET_LOCATION_FAILED)
    return LocationResponse(location=location)


@router.post("/relocate/{point_id}", response_model=Point)
async def relocate_by_point(point_id: str) -> Point:
    """POST /v1/robot/relocate/{pointId} — 200，回傳完整 Point 物件"""
    _require_navigation_mode()
    owner, point = store.find_point(point_id)
    current = state.current_map
    if current and owner != current:
        raise errors.ApiError(errors.POINT_NOT_IN_MAP)
    ok = await asyncio.to_thread(
        bridge.publish_initial_pose,
        ros_bridge.cm_to_m(point.location.x),
        ros_bridge.cm_to_m(point.location.y),
        ros_bridge.deg_to_yaw(point.location.orientation),
    )
    await hub.emit_event_async(
        WsEvent.RELOCATE, EventCode.COMPLETE if ok else EventCode.ABORT
    )
    if not ok:
        raise errors.ApiError(errors.GET_LOCATION_FAILED)
    return point


@router.post("/shutdown")
async def shutdown() -> Response:
    """POST /v1/robot/shutdown — 200。先送 power 事件（SHUTDOWN）再關機（文件 §9）。"""
    await hub.emit_event_async(WsEvent.POWER, EventCode.SHUTDOWN)
    await asyncio.to_thread(ros_bridge.stop_motion)
    try:
        await asyncio.to_thread(ros_bridge.shutdown_system)
    except ros_bridge.ProcessError as e:
        logger.error(f"Shutdown failed: {e}")
        raise errors.ApiError(errors.INTERNAL_ERROR)
    return Response(status_code=200)


# 讓 nav_manager 在本模組可見（供 main 的監控迴圈引用）
__all__ = ['router', 'build_robot_info', 'nav_manager']
