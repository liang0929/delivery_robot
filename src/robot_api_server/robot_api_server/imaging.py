"""集中處理 PIL 的可選相依（Jetson 環境有時未安裝 Pillow）。

``ros_bridge.py``（即時地圖 PNG）與 ``routers/maps.py``（PGM 轉 PNG）都需要
同一套「有就用、沒有就降級」邏輯，統一從這裡匯入避免重複的 try/except。
"""

try:
    from PIL import Image
    PIL_AVAILABLE = True
except Exception:  # pragma: no cover - 取決於執行環境
    Image = None
    PIL_AVAILABLE = False

__all__ = ['Image', 'PIL_AVAILABLE']
