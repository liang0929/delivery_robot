"""Groups 端點 🟢（規格 §5 / §4.2）。

要點：

- Group 預設 ``is_enable = false``，只有 enabled 的 group 會套用到機器人
- 一面虛擬牆可屬於多個 group
- ``PATCH`` 只接受 ``name`` / ``is_enable``，變更要等 apply 才生效
- ``GET /groups/{groupId}/virtual-walls`` 只回 ``{id}``；
  ``GET /groups/{groupId}`` 才回完整虛擬牆物件
"""

import asyncio
from typing import Optional

from fastapi import APIRouter, Query, Response

from ..logging_config import get_logger
from ..models import (
    GroupCreate,
    GroupCreated,
    GroupDetail,
    GroupList,
    GroupSummary,
    GroupUpdate,
    GroupWallAdd,
    GroupWallLink,
    VirtualWallRef,
    VirtualWallRefList,
)
from ..store import store

logger = get_logger(__name__)

router = APIRouter(prefix="/groups", tags=["groups"])


@router.post("", response_model=GroupCreated, status_code=201)
@router.post("/", response_model=GroupCreated, status_code=201, include_in_schema=False)
async def create_group(request: GroupCreate) -> GroupCreated:
    """POST /v1/robot/groups — **201** → {id, map, name}"""
    group = store.create_group(request.map, request.name)
    return GroupCreated(id=group.id, map=group.map, name=group.name)


@router.get("", response_model=GroupList)
@router.get("/", response_model=GroupList, include_in_schema=False)
async def list_groups(map: Optional[str] = Query(default=None)) -> GroupList:
    """GET /v1/robot/groups?map=name — 200 → {groups: [{id, map, name, is_enable}]}"""
    groups = store.list_groups(map)
    return GroupList(
        groups=[
            GroupSummary(id=g.id, map=g.map, name=g.name, is_enable=g.is_enable) for g in groups
        ]
    )


@router.post("/actions/apply")
async def apply_groups(map: Optional[str] = Query(default=None)) -> Response:
    """POST /v1/robot/groups/actions/apply — 200

    只把目前的 group 狀態套用到機器人（重新產生 keepout mask），**不寫入磁碟**。
    """
    await asyncio.to_thread(store.apply, map)
    return Response(status_code=200)


@router.get("/{group_id}", response_model=GroupDetail)
async def get_group(group_id: str) -> GroupDetail:
    """GET /v1/robot/groups/{groupId} — 200，含完整 virtual_walls 物件陣列"""
    group, walls = store.group_detail(group_id)
    return GroupDetail(
        id=group.id,
        map=group.map,
        name=group.name,
        is_enable=group.is_enable,
        virtual_walls=walls,
    )


@router.patch("/{group_id}", response_model=GroupSummary)
async def update_group(group_id: str, request: GroupUpdate) -> GroupSummary:
    """PATCH /v1/robot/groups/{groupId} — 200 → {id, map, name, is_enable}"""
    group = store.update_group(group_id, name=request.name, is_enable=request.is_enable)
    return GroupSummary(id=group.id, map=group.map, name=group.name, is_enable=group.is_enable)


@router.delete("/{group_id}", status_code=204)
async def delete_group(group_id: str) -> Response:
    """DELETE /v1/robot/groups/{groupId} — **204**"""
    store.delete_group(group_id)
    return Response(status_code=204)


@router.post("/{group_id}/virtual-walls", response_model=GroupWallLink, status_code=201)
async def add_wall_to_group(group_id: str, request: GroupWallAdd) -> GroupWallLink:
    """POST /v1/robot/groups/{groupId}/virtual-walls — **201** → {group_id, virtual_wall_id}"""
    store.group_add_wall(group_id, request.id)
    return GroupWallLink(group_id=group_id, virtual_wall_id=request.id)


@router.get("/{group_id}/virtual-walls", response_model=VirtualWallRefList)
async def list_group_walls(group_id: str) -> VirtualWallRefList:
    """GET /v1/robot/groups/{groupId}/virtual-walls — 200 → {virtual_walls: [{id}]}

    注意：**只回 id**，與 Get Group Details 不同。
    """
    wall_ids = store.group_wall_ids(group_id)
    return VirtualWallRefList(virtual_walls=[VirtualWallRef(id=wid) for wid in wall_ids])


@router.delete("/{group_id}/virtual-walls/{wall_id}", status_code=204)
async def remove_wall_from_group(group_id: str, wall_id: str) -> Response:
    """DELETE /v1/robot/groups/{groupId}/virtual-walls/{virtualWallId} — **204**"""
    store.group_remove_wall(group_id, wall_id)
    return Response(status_code=204)
