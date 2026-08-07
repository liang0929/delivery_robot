"""集中管理環境變數與路徑設定（Winstec Robot API v1.1）。

以 pydantic ``BaseModel`` 集中定義所有設定值（型別驗證），環境變數在
:meth:`Settings.from_env` 讀入後一次性建構。專案已內建 pydantic 2.x，但
執行環境未安裝 ``pydantic-settings``（Jetson 上不便額外安裝依賴），因此
不用 ``pydantic.BaseSettings`` / ``pydantic_settings.BaseSettings`` 自動讀
環境變數，改為手動組值後傳入建構子——效果等價，且不新增依賴。

為向後相容既有 ``from .config import X`` 的散落用法，模組層級仍保留同名
常數作為 ``settings`` 的別名匯出；新程式碼建議直接使用 ``settings.x``。

副作用（``os.makedirs``）已移除：目錄建立改由啟動流程呼叫
``settings.ensure_map_path()``（見 ``main.py`` 的 lifespan），import 這個
模組不再有檔案系統副作用。
"""

import os
from typing import List

from pydantic import BaseModel

from .logging_config import get_logger

logger = get_logger(__name__)


def _get_workspace_root() -> str:
    """取得 workspace 根目錄路徑"""
    env_workspace = os.environ.get('ROBOT_WORKSPACE')
    if env_workspace and os.path.isdir(env_workspace):
        return env_workspace

    # .../src/robot_api_server/robot_api_server/config.py → 往上 4 層
    current_file = os.path.abspath(__file__)
    workspace = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(current_file))))
    if os.path.isdir(os.path.join(workspace, 'src')):
        return workspace

    return '/home/robot0/base_dev'


class Settings(BaseModel):
    """集中設定值（型別已由 pydantic 驗證）。"""

    workspace_root: str
    map_path: str

    # --- 服務埠（規格 §3）---
    http_port: int = 5000
    ws_port: int = 5001
    bind_host: str = "0.0.0.0"

    # --- 電池換算（文件 §9）---
    # 24V 30Ah 18650 電池組為 7S：滿充 7 × 4.2V = 29.4V，放空 7 × 3.0V = 21.0V。
    # 電壓來源是 /motor/voltage（馬達驅動器回報的母線電壓），負載時會下垂，
    # 因此百分比只能當粗略指示，不是庫倫計。
    battery_min_v: float = 21.0
    battery_max_v: float = 29.4

    # --- 手動移動速度 ---
    manual_linear_speed: float = 0.16875
    manual_angular_speed: float = 0.5
    manual_publish_hz: float = 10.0

    # --- keepout 產生參數（文件 §8）---
    robot_radius_m: float = 0.25

    # --- CORS ---
    allowed_origins: List[str] = ["*"]
    allow_credentials: bool = False

    # --- Nav2 就緒逾時 ---
    # 等待 Nav2 就緒的上限。冷啟動時全部節點 active 約需 10-20 秒，留足餘裕。
    nav2_ready_timeout_sec: float = 40.0

    # --- /battery/state 過期判定 ---
    # battery_guard 以 publish_rate_hz=2.0 週期發布（見
    # motor_control/config/battery_guard.yaml），不是只在狀態變化時發，
    # 所以「超過這段時間沒收到新訊息」是上游死掉的可靠訊號。5 秒 = 10 個
    # 發布週期，容忍排程抖動與 DDS 重傳，又能在操作者反應時間內把 UI 退回
    # unknown——保留凍結的 ok 會讓人以為低電壓保護還在線。
    # 設為 0 或負值＝停用過期判定（保留最後一次狀態，即舊行為）。
    battery_state_timeout_sec: float = 5.0

    @classmethod
    def from_env(cls) -> "Settings":
        workspace_root = _get_workspace_root()
        default_map_path = os.path.join(workspace_root, 'map')
        map_path = os.environ.get('ROBOT_MAP_PATH', default_map_path)

        origins_env = os.environ.get('CORS_ORIGINS', '')
        allowed_origins = [o.strip() for o in origins_env.split(',') if o.strip()] or ['*']
        allow_all_origins = '*' in allowed_origins

        return cls(
            workspace_root=workspace_root,
            map_path=map_path,
            http_port=int(os.environ.get('ROBOT_API_HTTP_PORT', '5000')),
            ws_port=int(os.environ.get('ROBOT_API_WS_PORT', '5001')),
            bind_host=os.environ.get('ROBOT_API_HOST', '0.0.0.0'),
            battery_min_v=float(os.environ.get('ROBOT_BATTERY_MIN_V', '21.0')),
            battery_max_v=float(os.environ.get('ROBOT_BATTERY_MAX_V', '29.4')),
            manual_linear_speed=float(os.environ.get('ROBOT_MANUAL_LINEAR', '0.16875')),
            manual_angular_speed=float(os.environ.get('ROBOT_MANUAL_ANGULAR', '0.5')),
            manual_publish_hz=float(os.environ.get('ROBOT_MANUAL_HZ', '10.0')),
            robot_radius_m=float(os.environ.get('ROBOT_RADIUS_M', '0.25')),
            allowed_origins=allowed_origins,
            allow_credentials=not allow_all_origins,
            nav2_ready_timeout_sec=float(os.environ.get('ROBOT_NAV2_READY_TIMEOUT', '40')),
            battery_state_timeout_sec=float(
                os.environ.get('ROBOT_BATTERY_STATE_TIMEOUT', '5.0')),
        )

    def ensure_map_path(self) -> None:
        """建立地圖目錄。刻意延後到啟動流程呼叫，避免 import 期的檔案系統副作用。"""
        os.makedirs(self.map_path, exist_ok=True)


settings = Settings.from_env()

# --- 向後相容別名：既有 `from .config import X` 用法維持可用 ---
WORKSPACE_ROOT = settings.workspace_root
DEFAULT_MAP_PATH = os.path.join(settings.workspace_root, 'map')
MAP_PATH = settings.map_path

HTTP_PORT = settings.http_port
WS_PORT = settings.ws_port
BIND_HOST = settings.bind_host

BATTERY_MIN_V = settings.battery_min_v
BATTERY_MAX_V = settings.battery_max_v

MANUAL_LINEAR_SPEED = settings.manual_linear_speed
MANUAL_ANGULAR_SPEED = settings.manual_angular_speed
MANUAL_PUBLISH_HZ = settings.manual_publish_hz

ROBOT_RADIUS_M = settings.robot_radius_m

ALLOWED_ORIGINS = settings.allowed_origins
ALLOW_ALL_ORIGINS = '*' in ALLOWED_ORIGINS
ALLOW_CREDENTIALS = settings.allow_credentials

NAV2_READY_TIMEOUT_SEC = settings.nav2_ready_timeout_sec
BATTERY_STATE_TIMEOUT_SEC = settings.battery_state_timeout_sec

logger.info(f"Map path: {MAP_PATH}; HTTP {BIND_HOST}:{HTTP_PORT}; WS {BIND_HOST}:{WS_PORT}")
