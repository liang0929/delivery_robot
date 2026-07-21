"""Points 端點 🟢（規格 §5）。"""

import asyncio
from typing import Optional

from fastapi import APIRouter, Query, Response

from .. import errors
from ..logging_config import get_logger
from ..models import Point, PointCreate, PointList, PointUpdate
from ..ros_bridge import bridge
from ..store import store

logger = get_logger(__name__)

router = APIRouter(prefix="/points", tags=["points"])


@router.post("", response_model=Point, status_code=201)
@router.post("/", response_model=Point, status_code=201, include_in_schema=False)
async def create_point(request: PointCreate) -> Point:
    """POST /v1/robot/points — **201**

    ``location`` 省略時使用機器人當前位置。
    """
    location = request.location
    if location is None:
        location = await asyncio.to_thread(bridge.location)
        if location is None:
            raise errors.ApiError(errors.GET_LOCATION_FAILED)
    return store.create_point(request.map, request.name, request.type, location)


@router.get("", response_model=PointList)
@router.get("/", response_model=PointList, include_in_schema=False)
async def list_points(map: Optional[str] = Query(default=None)) -> PointList:
    """GET /v1/robot/points?map=name — 200"""
    return PointList(points=store.list_points(map))


@router.get("/{point_id}", response_model=Point)
async def get_point(point_id: str) -> Point:
    """GET /v1/robot/points/{pointId} — 200"""
    _, point = store.find_point(point_id)
    return point


@router.patch("/{point_id}", response_model=Point)
async def update_point(point_id: str, request: PointUpdate) -> Point:
    """PATCH /v1/robot/points/{pointId} — 200（所有欄位皆選填）"""
    return store.update_point(
        point_id,
        name=request.name,
        point_type=request.type,
        location=request.location,
        map_name=request.map,
    )


@router.delete("/{point_id}", status_code=204)
async def delete_point(point_id: str, map: Optional[str] = Query(default=None)) -> Response:
    """DELETE /v1/robot/points/{pointId}?map=name — **204**"""
    store.delete_points(map, point_id)
    return Response(status_code=204)


@router.delete("", status_code=204)
@router.delete("/", status_code=204, include_in_schema=False)
async def delete_all_points(map: Optional[str] = Query(default=None)) -> Response:
    """DELETE /v1/robot/points?map=name — **204**

    ``pointId`` 為選填，省略時刪除整個 map 的點位（規格明定）。
    """
    store.delete_points(map, None)
    return Response(status_code=204)
