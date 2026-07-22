"""集中管理環境變數與路徑設定（Winstec Robot API v1.1）。"""

import os

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


WORKSPACE_ROOT = _get_workspace_root()
DEFAULT_MAP_PATH = os.path.join(WORKSPACE_ROOT, 'map')
MAP_PATH = os.environ.get('ROBOT_MAP_PATH', DEFAULT_MAP_PATH)

os.makedirs(MAP_PATH, exist_ok=True)

# --- 服務埠（規格 §3）---
HTTP_PORT = int(os.environ.get('ROBOT_API_HTTP_PORT', '5000'))
WS_PORT = int(os.environ.get('ROBOT_API_WS_PORT', '5001'))
BIND_HOST = os.environ.get('ROBOT_API_HOST', '0.0.0.0')

# --- 電池換算（文件 §9）---
# 24V 30Ah 18650 電池組為 7S：滿充 7 × 4.2V = 29.4V，放空 7 × 3.0V = 21.0V。
# 電壓來源是 /motor/voltage（馬達驅動器回報的母線電壓），負載時會下垂，
# 因此百分比只能當粗略指示，不是庫倫計。
BATTERY_MIN_V = float(os.environ.get('ROBOT_BATTERY_MIN_V', '21.0'))
BATTERY_MAX_V = float(os.environ.get('ROBOT_BATTERY_MAX_V', '29.4'))

# --- 手動移動速度 ---
MANUAL_LINEAR_SPEED = float(os.environ.get('ROBOT_MANUAL_LINEAR', '0.15'))
MANUAL_ANGULAR_SPEED = float(os.environ.get('ROBOT_MANUAL_ANGULAR', '0.5'))
MANUAL_PUBLISH_HZ = float(os.environ.get('ROBOT_MANUAL_HZ', '10.0'))

# --- keepout 產生參數（文件 §8）---
ROBOT_RADIUS_M = float(os.environ.get('ROBOT_RADIUS_M', '0.25'))

# --- CORS ---
_origins_env = os.environ.get('CORS_ORIGINS', '')
ALLOWED_ORIGINS = [o.strip() for o in _origins_env.split(',') if o.strip()] or ['*']
ALLOW_ALL_ORIGINS = '*' in ALLOWED_ORIGINS
ALLOW_CREDENTIALS = not ALLOW_ALL_ORIGINS

logger.info(f"Map path: {MAP_PATH}; HTTP {BIND_HOST}:{HTTP_PORT}; WS {BIND_HOST}:{WS_PORT}")
