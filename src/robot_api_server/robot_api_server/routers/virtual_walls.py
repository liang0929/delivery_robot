"""Virtual Walls 端點 🟢（規格 §5）。虛擬牆是線段，非多邊形。"""

from typing import Optional

from fastapi import APIRouter, Query, Response

from ..logging_config import get_logger
from ..models import VirtualWall, VirtualWallCreate, VirtualWallList, VirtualWallUpdate
from ..store import store

logger = get_logger(__name__)

router = APIRouter(prefix="/virtual-walls", tags=["virtual-walls"])


@router.post("", response_model=VirtualWall, status_code=201)
@router.post("/", response_model=VirtualWall, status_code=201, include_in_schema=False)
async def create_virtual_wall(request: VirtualWallCreate) -> VirtualWall:
    """POST /v1/robot/virtual-walls — **201**"""
    return store.create_wall(
        request.map, request.name, request.start_position, request.end_position
    )


@router.get("", response_model=VirtualWallList)
@router.get("/", response_model=VirtualWallList, include_in_schema=False)
async def list_virtual_walls(map: Optional[str] = Query(default=None)) -> VirtualWallList:
    """GET /v1/robot/virtual-walls?map=name — 200"""
    return VirtualWallList(virtual_walls=store.list_walls(map))


@router.patch("/{wall_id}", response_model=VirtualWall)
async def update_virtual_wall(wall_id: str, request: VirtualWallUpdate) -> VirtualWall:
    """PATCH /v1/robot/virtual-walls/{virtualWallId} — 200"""
    return store.update_wall(
        wall_id,
        name=request.name,
        start_position=request.start_position,
        end_position=request.end_position,
        map_name=request.map,
    )


@router.delete("/{wall_id}", status_code=204)
async def delete_virtual_wall(wall_id: str, map: Optional[str] = Query(default=None)) -> Response:
    """DELETE /v1/robot/virtual-walls/{virtualWallId}?map=name — **204**"""
    store.delete_walls(map, wall_id)
    return Response(status_code=204)


@router.delete("", status_code=204)
@router.delete("/", status_code=204, include_in_schema=False)
async def delete_all_virtual_walls(map: Optional[str] = Query(default=None)) -> Response:
    """DELETE /v1/robot/virtual-walls?map=name — **204**

    ``virtualWallId`` 為選填，省略時刪除整個 map 的虛擬牆（規格明定）。
    """
    store.delete_walls(map, None)
    return Response(status_code=204)
