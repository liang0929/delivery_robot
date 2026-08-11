"""充電座位置記錄端點 🟡（本專案擴充）。

部署現場的流程是：把車推／開到與充電座**完全對接**的位置 → 在手機網頁按
一顆「記錄充電座位置」→ 當下的 ``map→base_link`` 換算成 dock pose 寫進
``dock_database.yaml``。opennav_docking 會再由 dock pose 自行推算 staging
pose（沿 dock 朝向退 ``staging_x_offset``），所以現場只需要記這一個點。

換算式（``base_link yaw + 180°``）與其原始碼出處寫在 ``..dock_recorder``
的模組說明，這裡只做 HTTP 層的事：取位姿、轉錯誤碼、回傳。

## 🔴 map frame 不存在時的降級

Nav2 尚未啟動，``map`` frame 目前不存在。這種情況**明確失敗**
（``DOCK_POSE_UNAVAILABLE`` + 人話 detail），不會靜默記成別的 frame。
``?frame=odom`` 是給流程測試用的逃生口：odom 在每次重開機歸零，記到的值
下次開機就對不上實體 dock，所以回應帶 ``test_only: true``、寫進 yaml 的
條目也會標成測試值，UI 必須照樣標示。
"""

import asyncio

from fastapi import APIRouter, Query

from .. import errors
from ..config import DOCK_DATABASE_PATH
from ..dock_recorder import (
    DEFAULT_DOCK_ID,
    DockDatabaseError,
    PRODUCTION_FRAME,
    record_dock_pose,
)
from ..logging_config import get_logger
from ..models import DockPoseRecord
from ..ros_facade import robot_service as service
from .common import EXTENSION

logger = get_logger(__name__)

router = APIRouter(tags=["dock"])

#: TF 查詢逾時。TF 是串流資料，剛切到導航模式時 buffer 可能還在沉澱；
#: 但這是一顆前景按鈕，等太久使用者會重按，2 秒是兩者的折衷。
_TF_TIMEOUT_SEC = 2.0


@router.post("/dock/record_pose", response_model=DockPoseRecord, openapi_extra=EXTENSION)
async def record_dock_pose_endpoint(
    frame: str = Query(
        PRODUCTION_FRAME,
        description="記錄所在的 frame。map 以外一律標記為測試值（test_only）",
    ),
    dock_id: str = Query(DEFAULT_DOCK_ID, description="dock_database.yaml 的條目名"),
    contact_offset: float = Query(
        0.0,
        description=(
            "沿 dock 朝向（+x，指向充電座內部）平移的距離，公尺。"
            "預設 0＝直接記 base_link 原點；量到車體中心→刷塊接觸面的距離後再帶入"
        ),
    ),
) -> DockPoseRecord:
    """POST /v1/robot/dock/record_pose 🟡 — 記錄「車已對接」時的充電座位姿"""
    pose, detail = await asyncio.to_thread(
        service.lookup_pose, frame, 'base_link', _TF_TIMEOUT_SEC
    )
    if pose is None:
        logger.error(f"記錄充電座位置失敗：{detail}")
        hint = detail
        if frame == PRODUCTION_FRAME:
            hint = (
                f"{detail}。map frame 需要 Nav2／AMCL 在跑並且已完成定位；"
                "目前無法取得正式座標。僅測試流程可改用 ?frame=odom（記到的值"
                "重開機後失效，不可當正式值）。"
            )
        raise errors.ApiError(errors.DOCK_POSE_UNAVAILABLE, detail=hint)

    x_m, y_m, yaw_rad = pose
    try:
        result = await asyncio.to_thread(
            record_dock_pose,
            DOCK_DATABASE_PATH, x_m, y_m, yaw_rad, frame, dock_id, contact_offset,
        )
    except DockDatabaseError as e:
        logger.error(f"寫入 dock database 失敗：{e}")
        raise errors.ApiError(errors.DOCK_DB_WRITE_FAILED, detail=str(e))

    return DockPoseRecord(**result)


__all__ = ['router']
