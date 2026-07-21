"""Nav2 keepout mask 產生（文件 §8）——轉呼叫 ``nav2.keepout``。

實際的 mask 產生與 map_server 重載邏輯在 ``src/nav2/nav2/keepout.py``::

    regenerate_keepout(map_name, map_dir) -> str | None      # 回傳 keepout.yaml 路徑
    reload_keepout_mask(mask_yaml_path=..., ...) -> bool
    regenerate_and_reload(map_name, map_dir, ...) -> str | None

本模組只負責三件事：

1. 固定帶入本套件的 ``MAP_PATH`` 作為 ``map_dir``
2. ``nav2`` 套件不可用時降級（開發機可能沒 source ROS 環境）
3. 失敗只記 log、不讓 commit / apply 的 API 請求失敗

呼叫時機（見 ``store.py``）：

- ``POST /v1/robot/edits/commit`` → 寫入磁碟後
- ``POST /v1/robot/groups/actions/apply`` → 不寫磁碟，只重產 mask 並重載
"""

from typing import Optional

from .config import MAP_PATH
from .logging_config import get_logger

logger = get_logger(__name__)

NAV2_KEEPOUT_AVAILABLE = True
_IMPORT_ERROR: Optional[str] = None
try:
    from nav2.keepout import regenerate_and_reload as _regenerate_and_reload
except Exception as e:  # pragma: no cover - 取決於是否 source 了 ROS 環境
    NAV2_KEEPOUT_AVAILABLE = False
    _IMPORT_ERROR = str(e)
    _regenerate_and_reload = None
    logger.warning(f"nav2.keepout unavailable, keepout mask will not be regenerated: {e}")


def regenerate_keepout(map_name: str) -> bool:
    """重新產生 ``map_name`` 的 keepout mask 並通知 Nav2 重載。

    Args:
        map_name: 地圖名稱（不含副檔名），對應 ``$ROBOT_MAP_PATH/<map_name>.yaml``

    Returns:
        True 表示 mask 已成功產生，False 表示未產生（套件不可用或產生失敗）。
        呼叫端不應因為 False 而讓 API 請求失敗——commit/apply 的資料變更
        已經生效，mask 產生失敗只記錄警告。
    """
    if not NAV2_KEEPOUT_AVAILABLE:
        logger.warning(
            f"Skipping keepout regeneration for '{map_name}': "
            f"nav2.keepout unavailable ({_IMPORT_ERROR})"
        )
        return False

    try:
        path = _regenerate_and_reload(map_name, MAP_PATH)
    except Exception as e:
        logger.error(f"regenerate_keepout('{map_name}') failed: {e}")
        return False

    if not path:
        logger.warning(f"keepout mask not generated for '{map_name}'")
        return False

    logger.info(f"keepout mask regenerated for '{map_name}': {path}")
    return True
