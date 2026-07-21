"""地圖管理端點 🟡（本專案擴充，見文件 §7）。

``/maps/live/*`` 讓前端在建圖時以 1–2 Hz 輪詢顯示地圖成長，取代 rosbridge。
"""

import asyncio
import io
import os

import yaml
from fastapi import APIRouter, Response

from .. import errors
from ..config import MAP_PATH
from ..logging_config import get_logger
from ..models import MapInfo, MapList, MapMetadata, MapSaveRequest
from ..ros_bridge import ProcessError, bridge, save_map, state
from ..store import list_map_names, map_exists, sanitize_map_name, store

logger = get_logger(__name__)

try:
    from PIL import Image
    PIL_AVAILABLE = True
except Exception:  # pragma: no cover
    PIL_AVAILABLE = False

router = APIRouter(prefix="/maps", tags=["maps"])

EXTENSION = {"x-extension": True}


@router.get("", response_model=MapList, openapi_extra=EXTENSION)
@router.get("/", response_model=MapList, include_in_schema=False)
async def list_maps() -> MapList:
    """GET /v1/robot/maps 🟡"""
    return MapList(maps=[MapInfo(name=n) for n in list_map_names()])


@router.post("", status_code=201, openapi_extra=EXTENSION)
@router.post("/", status_code=201, include_in_schema=False)
async def create_map(request: MapSaveRequest) -> MapInfo:
    """POST /v1/robot/maps 🟡 — 儲存目前建圖結果"""
    name = sanitize_map_name(request.name)
    try:
        await asyncio.to_thread(save_map, name)
    except ProcessError as e:
        logger.error(f"Map save failed: {e}")
        raise errors.ApiError(errors.INTERNAL_ERROR)
    return MapInfo(name=name)


# --- /maps/live/* 必須排在 /maps/{name}/* 之前，否則會被路徑參數吃掉 ---
@router.get("/live/image", openapi_extra=EXTENSION)
async def get_live_map_image() -> Response:
    """GET /v1/robot/maps/live/image 🟡 — 建圖中的即時地圖 PNG（訂閱 /map 轉檔）"""
    png = await asyncio.to_thread(bridge.live_map_png)
    if png is None:
        raise errors.ApiError(errors.MAP_NOT_FOUND)
    return Response(content=png, media_type="image/png",
                    headers={"Cache-Control": "no-store"})


@router.get("/live/metadata", response_model=MapMetadata, openapi_extra=EXTENSION)
async def get_live_map_metadata() -> MapMetadata:
    """GET /v1/robot/maps/live/metadata 🟡"""
    meta = bridge.live_map_metadata()
    if meta is None:
        raise errors.ApiError(errors.MAP_NOT_FOUND)
    return MapMetadata(**meta)


@router.get("/{map_name}/image", openapi_extra=EXTENSION)
async def get_map_image(map_name: str) -> Response:
    """GET /v1/robot/maps/{name}/image 🟡 — PGM 轉 PNG"""
    safe = sanitize_map_name(map_name)
    pgm_path = os.path.join(MAP_PATH, f"{safe}.pgm")
    if not os.path.exists(pgm_path) or not PIL_AVAILABLE:
        raise errors.ApiError(errors.MAP_NOT_FOUND)

    def _convert() -> bytes:
        with Image.open(pgm_path) as img:
            buf = io.BytesIO()
            img.save(buf, format="PNG")
            return buf.getvalue()

    try:
        png = await asyncio.to_thread(_convert)
    except Exception as e:
        logger.error(f"Failed to convert map image '{safe}': {e}")
        raise errors.ApiError(errors.INTERNAL_ERROR)
    return Response(content=png, media_type="image/png")


@router.get("/{map_name}/metadata", response_model=MapMetadata, openapi_extra=EXTENSION)
async def get_map_metadata(map_name: str) -> MapMetadata:
    """GET /v1/robot/maps/{name}/metadata 🟡"""
    safe = sanitize_map_name(map_name)
    yaml_path = os.path.join(MAP_PATH, f"{safe}.yaml")
    pgm_path = os.path.join(MAP_PATH, f"{safe}.pgm")
    if not os.path.exists(yaml_path):
        raise errors.ApiError(errors.MAP_NOT_FOUND)

    def _read() -> dict:
        with open(yaml_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        width = height = 0
        if PIL_AVAILABLE and os.path.exists(pgm_path):
            try:
                with Image.open(pgm_path) as img:
                    width, height = img.size
            except Exception as e:
                logger.error(f"Failed to read PGM size for '{safe}': {e}")
        return {
            "resolution": float(data.get("resolution", 0.05)),
            "origin": [float(v) for v in data.get("origin", [0.0, 0.0, 0.0])],
            "width": width,
            "height": height,
            "negate": int(data.get("negate", 0)),
            "occupied_thresh": float(data.get("occupied_thresh", 0.65)),
            "free_thresh": float(data.get("free_thresh", 0.196)),
        }

    try:
        meta = await asyncio.to_thread(_read)
    except Exception as e:
        # 細節只留在 log，不回傳內部錯誤資訊
        logger.error(f"Failed to read map metadata for '{safe}': {e}")
        raise errors.ApiError(errors.INTERNAL_ERROR)
    return MapMetadata(**meta)


@router.delete("/{map_name}", status_code=204, openapi_extra=EXTENSION)
async def delete_map(map_name: str) -> Response:
    """DELETE /v1/robot/maps/{name} 🟡 — 刪除地圖及其 points/walls/groups"""
    safe = sanitize_map_name(map_name)
    if not map_exists(safe):
        raise errors.ApiError(errors.MAP_NOT_FOUND)
    if state.current_map == safe:
        raise errors.ApiError(errors.ROBOT_BUSY)

    def _delete() -> None:
        store.delete_map_data(safe)
        base = os.path.join(MAP_PATH, safe)
        for suffix in ('.yaml', '.pgm', '.keepout.yaml', '.keepout.pgm'):
            try:
                os.remove(f"{base}{suffix}")
            except FileNotFoundError:
                pass
            except OSError as e:
                logger.warning(f"Failed to remove {base}{suffix}: {e}")

    await asyncio.to_thread(_delete)
    return Response(status_code=204)
