"""points / virtual walls / groups 的 staging + 磁碟持久化（文件 §8）。

儲存契約（全部位於 ``$ROBOT_MAP_PATH``）::

    <map>.yaml / <map>.pgm             # 地圖本體
    <map>.points.json                  # [{id, map, name, type, location}]
    <map>.virtual_walls.json           # [{id, map, name, start_position, end_position}]
    <map>.groups.json                  # [{id, map, name, is_enable, virtual_wall_ids: []}]

**JSON 內一律存 API 單位（cm 整數 + 度）**。

staging 模型：

- 未提交的變更只存在記憶體，磁碟永遠是已提交狀態
- ``discard()`` → 丟棄記憶體暫存
- ``commit()`` → 寫入磁碟 → 重新產生 keepout mask
- ``apply()``（groups/actions/apply）→ 只重新產生 mask，不寫入磁碟
"""

import json
import os
import secrets
import threading
from typing import Callable, Dict, List, Optional, Tuple, TypeVar

from pydantic import BaseModel, Field, ValidationError

from . import errors
from .config import MAP_PATH
from .keepout import regenerate_keepout
from .logging_config import get_logger
from .models import (
    GroupSummary,
    Location,
    Point,
    PointType,
    Position,
    VirtualWall,
)

logger = get_logger(__name__)

_T = TypeVar('_T')


def _update_in_list(
    collection: List[_T],
    item_id: str,
    mutator: Callable[[_T], None],
    not_found_code: str,
) -> _T:
    """在 ``collection`` 中找到 ``id == item_id`` 的項目，深拷貝後交給 ``mutator``
    就地修改欄位，寫回原位並回傳更新後的項目。

    找不到、或 ``mutator`` 內丟出例外（例如欄位驗證失敗）時都不會寫回，
    語意與逐一手寫的 for 迴圈完全一致。
    """
    for idx, item in enumerate(collection):
        if item.id == item_id:
            updated = item.model_copy(deep=True)
            mutator(updated)
            collection[idx] = updated
            return updated
    raise errors.ApiError(not_found_code)


# --- ID 產生（文件 §9）---
def new_point_id() -> str:
    return f"pt_{secrets.token_urlsafe(16)}"


def new_wall_id() -> str:
    return f"vw_{secrets.token_urlsafe(16)}"


def new_group_id() -> str:
    return f"gp_{secrets.token_urlsafe(16)}"


class GroupRecord(GroupSummary):
    """磁碟上的 group 表示，比 API 的 GroupSummary 多了成員清單"""
    virtual_wall_ids: List[str] = Field(default_factory=list)


class MapData(BaseModel):
    """單一地圖的全部可編輯資料"""
    points: List[Point] = Field(default_factory=list)
    walls: List[VirtualWall] = Field(default_factory=list)
    groups: List[GroupRecord] = Field(default_factory=list)


def sanitize_map_name(map_name: str) -> str:
    """驗證並回傳安全的地圖名稱，防止路徑注入"""
    if not isinstance(map_name, str):
        raise errors.ApiError(errors.INVALID_INPUT)
    safe = "".join(c for c in map_name if c.isalnum() or c in ('-', '_'))
    if not safe or len(safe) > 64:
        raise errors.ApiError(errors.INVALID_INPUT)
    full = os.path.normpath(os.path.join(MAP_PATH, safe))
    base = os.path.normpath(MAP_PATH)
    if not full.startswith(base + os.sep) and full != base:
        raise errors.ApiError(errors.INVALID_INPUT)
    return safe


def map_yaml_path(map_name: str) -> str:
    return os.path.join(MAP_PATH, f"{map_name}.yaml")


def map_exists(map_name: str) -> bool:
    return os.path.exists(map_yaml_path(map_name))


def list_map_names() -> List[str]:
    """列出磁碟上所有地圖（同時有 .yaml 與 .pgm 才算）"""
    names = []
    try:
        for filename in sorted(os.listdir(MAP_PATH)):
            if filename.endswith('.yaml') and not filename.endswith('.keepout.yaml'):
                name = filename[:-5]
                if os.path.exists(os.path.join(MAP_PATH, f"{name}.pgm")):
                    names.append(name)
    except OSError as e:
        logger.error(f"Failed to list maps: {e}")
    return names


class EditStore:
    """points / virtual walls / groups 的暫存與持久化。

    執行緒安全。慢速磁碟 I/O 只在 :meth:`commit` 發生，且已在鎖內序列化——
    commit 的呼叫端應以 ``asyncio.to_thread`` 包裝，避免阻塞事件迴圈。
    """

    def __init__(self, current_map_provider: Optional[Callable[[], Optional[str]]] = None):
        self._lock = threading.RLock()
        self._staged: Dict[str, MapData] = {}
        self._current_map_provider = current_map_provider

    def set_current_map_provider(self, provider: Callable[[], Optional[str]]) -> None:
        self._current_map_provider = provider

    # ---------- 地圖解析 ----------
    def resolve_map(self, map_name: Optional[str]) -> str:
        """map 省略時代表「目前載入的地圖」；解析不到就是 MAP_NOT_FOUND"""
        if map_name:
            name = sanitize_map_name(map_name)
        else:
            current = self._current_map_provider() if self._current_map_provider else None
            if not current:
                raise errors.ApiError(errors.MAP_NOT_FOUND)
            name = sanitize_map_name(current)
        if not map_exists(name):
            raise errors.ApiError(errors.MAP_NOT_FOUND)
        return name

    # ---------- 磁碟存取 ----------
    @staticmethod
    def _read_json(path: str) -> list:
        if not os.path.exists(path):
            return []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                data = json.load(f)
            return data if isinstance(data, list) else []
        except (OSError, json.JSONDecodeError) as e:
            logger.error(f"Failed to read {path}: {e}")
            return []

    @staticmethod
    def _write_json(path: str, data: list) -> None:
        tmp = f"{path}.tmp"
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        os.replace(tmp, path)

    def _load_from_disk(self, map_name: str) -> MapData:
        base = os.path.join(MAP_PATH, map_name)
        data = MapData()
        for raw in self._read_json(f"{base}.points.json"):
            try:
                data.points.append(Point(**raw))
            except ValidationError as e:
                logger.warning(f"Skipping invalid point in {map_name}: {e}")
        for raw in self._read_json(f"{base}.virtual_walls.json"):
            try:
                data.walls.append(VirtualWall(**raw))
            except ValidationError as e:
                logger.warning(f"Skipping invalid virtual wall in {map_name}: {e}")
        for raw in self._read_json(f"{base}.groups.json"):
            try:
                data.groups.append(GroupRecord(**raw))
            except ValidationError as e:
                logger.warning(f"Skipping invalid group in {map_name}: {e}")
        return data

    def _save_to_disk(self, map_name: str, data: MapData) -> None:
        base = os.path.join(MAP_PATH, map_name)
        self._write_json(f"{base}.points.json", [p.model_dump(mode='json') for p in data.points])
        self._write_json(
            f"{base}.virtual_walls.json", [w.model_dump(mode='json') for w in data.walls]
        )
        self._write_json(f"{base}.groups.json", [g.model_dump(mode='json') for g in data.groups])

    # ---------- 讀 / 寫視圖 ----------
    def _view(self, map_name: str) -> MapData:
        """唯讀視圖：有暫存用暫存，否則讀磁碟（不會建立暫存）"""
        with self._lock:
            staged = self._staged.get(map_name)
            if staged is not None:
                return staged.model_copy(deep=True)
        return self._load_from_disk(map_name)

    def _mutable(self, map_name: str) -> MapData:
        """可寫視圖：確保該 map 已進入 staging（需持有 self._lock）"""
        staged = self._staged.get(map_name)
        if staged is None:
            staged = self._load_from_disk(map_name)
            self._staged[map_name] = staged
        return staged

    def _known_maps(self) -> List[str]:
        with self._lock:
            staged_names = list(self._staged.keys())
        names = list(dict.fromkeys(list_map_names() + staged_names))
        return names

    # ---------- Points ----------
    def list_points(self, map_name: Optional[str]) -> List[Point]:
        name = self.resolve_map(map_name)
        return self._view(name).points

    def find_point(self, point_id: str) -> Tuple[str, Point]:
        """跨地圖依 id 找點位（GET/PATCH /points/{id} 沒有 map 參數）"""
        for name in self._known_maps():
            for p in self._view(name).points:
                if p.id == point_id:
                    return name, p
        raise errors.ApiError(errors.POINT_NOT_FOUND)

    def create_point(
        self, map_name: Optional[str], name: str, point_type: PointType, location: Location
    ) -> Point:
        if not name or not name.strip():
            raise errors.ApiError(errors.MISSING_POINT_NAME)
        resolved = self.resolve_map(map_name)
        point = Point(
            id=new_point_id(), map=resolved, name=name, type=point_type, location=location
        )
        with self._lock:
            self._mutable(resolved).points.append(point)
        return point

    def update_point(
        self,
        point_id: str,
        name: Optional[str] = None,
        point_type: Optional[PointType] = None,
        location: Optional[Location] = None,
        map_name: Optional[str] = None,
    ) -> Point:
        owner, _ = self.find_point(point_id)
        if map_name:
            requested = sanitize_map_name(map_name)
            if requested != owner:
                raise errors.ApiError(errors.MAP_MISMATCH)

        def mutate(updated: Point) -> None:
            if name is not None:
                if not name.strip():
                    raise errors.ApiError(errors.MISSING_POINT_NAME)
                updated.name = name
            if point_type is not None:
                updated.type = point_type
            if location is not None:
                updated.location = location

        with self._lock:
            data = self._mutable(owner)
            return _update_in_list(data.points, point_id, mutate, errors.POINT_NOT_FOUND)

    def delete_points(self, map_name: Optional[str], point_id: Optional[str] = None) -> None:
        """point_id 省略時刪除該 map 全部點位（規格明定 pointId 為選填）"""
        resolved = self.resolve_map(map_name)
        with self._lock:
            data = self._mutable(resolved)
            if point_id is None:
                data.points.clear()
                return
            remaining = [p for p in data.points if p.id != point_id]
            if len(remaining) == len(data.points):
                raise errors.ApiError(errors.POINT_NOT_FOUND)
            data.points = remaining

    # ---------- Virtual Walls ----------
    def list_walls(self, map_name: Optional[str]) -> List[VirtualWall]:
        name = self.resolve_map(map_name)
        return self._view(name).walls

    def find_wall(self, wall_id: str) -> Tuple[str, VirtualWall]:
        for name in self._known_maps():
            for w in self._view(name).walls:
                if w.id == wall_id:
                    return name, w
        raise errors.ApiError(errors.MISSING_VIRTUAL_WALL_NAME)

    def create_wall(
        self,
        map_name: Optional[str],
        name: str,
        start_position: Position,
        end_position: Position,
    ) -> VirtualWall:
        if not name or not name.strip():
            raise errors.ApiError(errors.MISSING_VIRTUAL_WALL_NAME)
        resolved = self.resolve_map(map_name)
        wall = VirtualWall(
            id=new_wall_id(),
            map=resolved,
            name=name,
            start_position=start_position,
            end_position=end_position,
        )
        with self._lock:
            self._mutable(resolved).walls.append(wall)
        return wall

    def update_wall(
        self,
        wall_id: str,
        name: Optional[str] = None,
        start_position: Optional[Position] = None,
        end_position: Optional[Position] = None,
        map_name: Optional[str] = None,
    ) -> VirtualWall:
        owner, _ = self.find_wall(wall_id)
        if map_name:
            requested = sanitize_map_name(map_name)
            if requested != owner:
                raise errors.ApiError(errors.MAP_MISMATCH)

        def mutate(updated: VirtualWall) -> None:
            if name is not None:
                if not name.strip():
                    raise errors.ApiError(errors.MISSING_VIRTUAL_WALL_NAME)
                updated.name = name
            if start_position is not None:
                updated.start_position = start_position
            if end_position is not None:
                updated.end_position = end_position

        with self._lock:
            data = self._mutable(owner)
            return _update_in_list(
                data.walls, wall_id, mutate, errors.MISSING_VIRTUAL_WALL_NAME
            )

    def delete_walls(self, map_name: Optional[str], wall_id: Optional[str] = None) -> None:
        """wall_id 省略時刪除該 map 全部虛擬牆（規格明定 virtualWallId 為選填）"""
        resolved = self.resolve_map(map_name)
        with self._lock:
            data = self._mutable(resolved)
            if wall_id is None:
                data.walls.clear()
                for g in data.groups:
                    g.virtual_wall_ids.clear()
                return
            remaining = [w for w in data.walls if w.id != wall_id]
            if len(remaining) == len(data.walls):
                raise errors.ApiError(errors.MISSING_VIRTUAL_WALL_NAME)
            data.walls = remaining
            # 一面牆可屬於多個 group，刪牆時一併移除所有參照
            for g in data.groups:
                if wall_id in g.virtual_wall_ids:
                    g.virtual_wall_ids.remove(wall_id)

    # ---------- Groups ----------
    def list_groups(self, map_name: Optional[str]) -> List[GroupRecord]:
        name = self.resolve_map(map_name)
        return self._view(name).groups

    def find_group(self, group_id: str) -> Tuple[str, GroupRecord]:
        for name in self._known_maps():
            for g in self._view(name).groups:
                if g.id == group_id:
                    return name, g
        raise errors.ApiError(errors.GROUP_NOT_FOUND)

    def create_group(self, map_name: Optional[str], name: str) -> GroupRecord:
        if not name or not name.strip():
            raise errors.ApiError(errors.MISSING_GROUP_NAME)
        resolved = self.resolve_map(map_name)
        # Group 預設 is_enable = false（規格 §4.2）
        group = GroupRecord(id=new_group_id(), map=resolved, name=name, is_enable=False)
        with self._lock:
            self._mutable(resolved).groups.append(group)
        return group

    def update_group(
        self, group_id: str, name: Optional[str] = None, is_enable: Optional[bool] = None
    ) -> GroupRecord:
        owner, _ = self.find_group(group_id)

        def mutate(updated: GroupRecord) -> None:
            if name is not None:
                if not name.strip():
                    raise errors.ApiError(errors.MISSING_GROUP_NAME)
                updated.name = name
            if is_enable is not None:
                updated.is_enable = is_enable

        with self._lock:
            data = self._mutable(owner)
            return _update_in_list(data.groups, group_id, mutate, errors.GROUP_NOT_FOUND)

    def delete_group(self, group_id: str) -> None:
        owner, _ = self.find_group(group_id)
        with self._lock:
            data = self._mutable(owner)
            data.groups = [g for g in data.groups if g.id != group_id]

    def group_detail(self, group_id: str) -> Tuple[GroupRecord, List[VirtualWall]]:
        """Get Group Details 需要完整的 virtual wall 物件陣列"""
        owner, group = self.find_group(group_id)
        walls_by_id = {w.id: w for w in self._view(owner).walls}
        walls = [walls_by_id[wid] for wid in group.virtual_wall_ids if wid in walls_by_id]
        return group, walls

    def group_add_wall(self, group_id: str, wall_id: str) -> None:
        owner, _ = self.find_group(group_id)
        wall_owner, _ = self.find_wall(wall_id)
        if wall_owner != owner:
            raise errors.ApiError(errors.MAP_MISMATCH)
        with self._lock:
            data = self._mutable(owner)
            for g in data.groups:
                if g.id == group_id:
                    if wall_id not in g.virtual_wall_ids:
                        g.virtual_wall_ids.append(wall_id)
                    return
        raise errors.ApiError(errors.GROUP_NOT_FOUND)

    def group_wall_ids(self, group_id: str) -> List[str]:
        _, group = self.find_group(group_id)
        return list(group.virtual_wall_ids)

    def group_remove_wall(self, group_id: str, wall_id: str) -> None:
        owner, _ = self.find_group(group_id)
        with self._lock:
            data = self._mutable(owner)
            for g in data.groups:
                if g.id == group_id:
                    if wall_id not in g.virtual_wall_ids:
                        raise errors.ApiError(errors.NOT_FOUND_IN_GROUP)
                    g.virtual_wall_ids.remove(wall_id)
                    return
        raise errors.ApiError(errors.GROUP_NOT_FOUND)

    # ---------- 編輯交易 ----------
    def commit(self) -> List[str]:
        """把全部暫存寫入磁碟並重新產生 keepout mask。回傳受影響的地圖名稱。"""
        with self._lock:
            staged = self._staged
            self._staged = {}

        written: List[str] = []
        for map_name, data in staged.items():
            try:
                self._save_to_disk(map_name, data)
                written.append(map_name)
            except OSError as e:
                logger.error(f"Failed to persist map data for '{map_name}': {e}")
                raise errors.ApiError(errors.INTERNAL_ERROR)

        for map_name in written:
            self._safe_regenerate(map_name)
        return written

    def discard(self) -> None:
        """丟棄全部未提交的變更"""
        with self._lock:
            self._staged.clear()

    def apply(self, map_name: Optional[str] = None) -> None:
        """只重新產生 mask 並通知重載，**不寫入磁碟**（規格 §4.2 apply）"""
        if map_name:
            targets = [self.resolve_map(map_name)]
        else:
            current = self._current_map_provider() if self._current_map_provider else None
            targets = [current] if current and map_exists(current) else self._known_maps()
        for name in targets:
            self._safe_regenerate(name)

    @staticmethod
    def _safe_regenerate(map_name: str) -> None:
        try:
            regenerate_keepout(map_name)
        except Exception as e:  # keepout 失敗不應讓資料變更的請求失敗
            logger.error(f"regenerate_keepout('{map_name}') failed: {e}")

    def has_pending(self) -> bool:
        with self._lock:
            return bool(self._staged)

    def delete_map_data(self, map_name: str) -> None:
        """刪除地圖時一併移除 points/walls/groups（🟡 擴充端點用）"""
        with self._lock:
            self._staged.pop(map_name, None)
        base = os.path.join(MAP_PATH, map_name)
        for suffix in ('.points.json', '.virtual_walls.json', '.groups.json'):
            try:
                os.remove(f"{base}{suffix}")
            except FileNotFoundError:
                pass
            except OSError as e:
                logger.warning(f"Failed to remove {base}{suffix}: {e}")


#: 全域 store 實例（current_map_provider 由 main 注入）
store = EditStore()
