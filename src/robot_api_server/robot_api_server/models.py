"""Pydantic 模型（規格 §7 共用資料模型 + 各端點請求體）。

所有座標欄位皆為 **API 單位**：x/y 為公分整數，orientation 為度（float）。
"""

from enum import Enum
from typing import List, Optional

from pydantic import BaseModel, Field


# --- §7.1 Operating Mode ---
class OpMode(str, Enum):
    EXPLORE = "explore"     # Mapping mode（slam_toolbox）
    NAVIGATE = "navigate"   # Navigation mode（Nav2）


# --- §7.2 Robot Status ---
class RobotStatus(str, Enum):
    INIT = "init"
    IDLE = "idle"
    RELOCATING = "relocating"
    MOVING = "moving"
    GO_CHARGING = "go_charging"
    SWITCHING_MODE = "switching_mode"


# --- §7.3 Point Type ---
class PointType(str, Enum):
    POINT = "point"
    CHARGE = "charge"


# --- §7.6 Event Codes（一律大寫，見文件 §3）---
class EventCode(str, Enum):
    COMPLETE = "COMPLETE"
    STUCK = "STUCK"
    ABORT = "ABORT"
    CHG_STA_NOT_FOUND = "CHG_STA_NOT_FOUND"
    SHUTDOWN = "SHUTDOWN"


class WsEvent(str, Enum):
    ROBOT_INFO = "robot_info"
    GO_POINT = "go_point"
    GO_CHARGING = "go_charging"
    SWITCH_MODE = "switch_mode"
    RELOCATE = "relocate"
    POWER = "power"


#: 🟡 本專案擴充：battery_guard（`/battery/state`）的低電壓保護狀態。
#: 值刻意用小寫，與 ``op_mode`` / ``status`` 等既有 API 欄位的慣例一致；
#: 上游 DiagnosticStatus 的 ``state`` KeyValue 是大寫（OK/WARNING/SHUTDOWN/
#: UNKNOWN），轉換在 ``bridge_node._on_battery_state`` 完成。
class BatteryState(str, Enum):
    OK = "ok"               # 電壓正常
    WARNING = "warning"     # 低於警告門檻，尚可行走
    SHUTDOWN = "shutdown"   # 低電壓停機（鎖存，需充電後重啟解除）
    UNKNOWN = "unknown"     # 沒有 /battery/state 資料，或來源全部逾時


class Direction(str, Enum):
    STOP = "stop"
    FORWARD = "forward"
    BACKWARD = "backward"
    RIGHT = "right"   # 順時針
    LEFT = "left"     # 逆時針


# --- §7.4 Location Object ---
class Location(BaseModel):
    x: int
    y: int
    orientation: float


# --- §7.5 Position Object（虛擬牆端點，無 orientation）---
class Position(BaseModel):
    x: int
    y: int


# --- 資源物件 ---
class Point(BaseModel):
    id: str
    map: str
    name: str
    type: PointType
    location: Location


class VirtualWall(BaseModel):
    id: str
    map: str
    name: str
    start_position: Position
    end_position: Position


class VirtualWallRef(BaseModel):
    """GET /groups/{id}/virtual-walls 只回 id（規格明定，與 Get Group Details 不同）"""
    id: str


class GroupSummary(BaseModel):
    id: str
    map: str
    name: str
    is_enable: bool = False


class GroupDetail(GroupSummary):
    virtual_walls: List[VirtualWall] = Field(default_factory=list)


class GroupCreated(BaseModel):
    """POST /groups 的回應：只有 id/map/name"""
    id: str
    map: str
    name: str


# --- 請求體 ---
class MoveRequest(BaseModel):
    type: PointType
    location: Location


class ManualMoveRequest(BaseModel):
    direction: Direction


class RelocateLocationRequest(BaseModel):
    location: Location


class PointCreate(BaseModel):
    map: Optional[str] = None
    name: str
    type: PointType
    location: Optional[Location] = None


class PointUpdate(BaseModel):
    map: Optional[str] = None
    name: Optional[str] = None
    type: Optional[PointType] = None
    location: Optional[Location] = None


class VirtualWallCreate(BaseModel):
    map: Optional[str] = None
    name: str
    start_position: Position
    end_position: Position


class VirtualWallUpdate(BaseModel):
    map: Optional[str] = None
    name: Optional[str] = None
    start_position: Optional[Position] = None
    end_position: Optional[Position] = None


class GroupCreate(BaseModel):
    map: Optional[str] = None
    name: str


class GroupUpdate(BaseModel):
    """PATCH /groups/{id} 只接受 name / is_enable，且要等 apply 才生效"""
    name: Optional[str] = None
    is_enable: Optional[bool] = None


class GroupWallAdd(BaseModel):
    id: str


class GroupWallLink(BaseModel):
    group_id: str
    virtual_wall_id: str


# --- 回應包裝 ---
class RobotInfo(BaseModel):
    op_mode: OpMode
    status: RobotStatus
    battery: int
    #: 🟡 本專案擴充：電池母線電壓（V）。取不到 /motor/voltage 時為 None，
    #: 前端以此區分「真的沒電」與「還沒有資料」。
    voltage: Optional[float] = None
    #: 🟡 本專案擴充：低電壓保護狀態，來自 battery_guard 的 ``/battery/state``。
    #: 沒有 battery_guard（或還沒收到訊息）時為 ``unknown``。
    battery_state: BatteryState = BatteryState.UNKNOWN
    #: 🟡 本專案擴充：停機鎖存旗標。True 代表 battery_guard 已發出
    #: ``/safety/stop``，機器人不會動；解除方式只有「充電後重啟」。
    battery_stop_latched: bool = False
    location: Location


class PointList(BaseModel):
    points: List[Point] = Field(default_factory=list)


class VirtualWallList(BaseModel):
    virtual_walls: List[VirtualWall] = Field(default_factory=list)


class VirtualWallRefList(BaseModel):
    virtual_walls: List[VirtualWallRef] = Field(default_factory=list)


class GroupList(BaseModel):
    groups: List[GroupSummary] = Field(default_factory=list)


class LocationResponse(BaseModel):
    location: Location


# --- 🟡 擴充端點的模型 ---
class ModeRequest(BaseModel):
    mode: OpMode
    map: Optional[str] = None


class MapSaveRequest(BaseModel):
    name: str


class MapInfo(BaseModel):
    name: str


class MapList(BaseModel):
    maps: List[MapInfo] = Field(default_factory=list)


class MapMetadata(BaseModel):
    resolution: float
    origin: List[float]
    width: int
    height: int
    negate: int = 0
    occupied_thresh: float = 0.65
    free_thresh: float = 0.196
