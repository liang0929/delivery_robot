# Robot Web Frontend

AMR 操作介面。只依賴 **Winstec Robot API v1.1**（`docs/winstec_api_v1.1.md`）：

| 服務 | Port |
| --- | --- |
| HTTP REST `/v1/robot/*` | 5000 |
| WebSocket（`robot_info` 與事件） | 5001 |

**不使用 rosbridge / roslib。** 地圖影像與 metadata 全部走 HTTP，機器人即時位姿由
WebSocket 的 `robot_info` 事件（每秒一次）提供。

## 功能

單頁應用，三個分頁：

1. **建圖** — 切 `explore` 模式、方向鍵遙控跑圖、約 1.25 Hz 輪詢 `/maps/live/image`
   顯示地圖成長、命名存檔。
2. **設定點位** — 在已存檔地圖上點擊建立點位（名稱 / `point`｜`charge` / 朝向）、
   拖出線段建立虛擬牆、列出與編輯刪除；因後端是暫存交易模型，畫面上會明示
   「有未提交的變更」並提供 commit / discard。
3. **自動導航** — 切 `navigate` 模式、地圖點擊或選點位重定位、選點位或點地圖導航、停止。

## 開發

```bash
cd ~/base_dev/src/robot_web_frontend
npm install
npm run dev     # http://<robot-ip>:3000
npm run build   # tsc + vite build
```

## 連線設定

機器人位址依序取用：`VITE_ROBOT_IP` → 瀏覽器 `window.location.hostname` → `localhost`。

```bash
VITE_ROBOT_IP=192.168.1.100 npm run build
```

其餘設定（port、輪詢頻率、WebSocket 重連 backoff）集中在 `src/config/robot.config.ts`。

## 原始碼結構

```
src/
  api/           REST 型別、fetch 包裝與端點封裝
  ws/            WebSocket client（3 秒 backoff 自動重連）
  store/         zustand：連線狀態、robot_info、toast
  lib/coords.ts  座標換算唯一來源（API 公分/度 ↔ ROS 公尺 ↔ 影像像素 ↔ canvas）
  hooks/         非同步動作、地圖來源輪詢、手動遙控、地圖清單
  components/    MapCanvas（雙層 canvas + 離屏點陣快取）、DPad、StatusBar、Toasts
  pages/         MappingPage / PointsPage / NavigationPage
```

### 兩個關鍵不變量

- **座標換算只寫在 `src/lib/coords.ts`。** 其他檔案不得出現 `resolution`、`origin`
  或 `* 100` 的手算。
- **遙控一定會停。** `hooks/useManualDrive.ts` 在 pointerup / pointercancel /
  touchcancel / blur / visibilitychange / pagehide / 元件卸載時都會送 `stop`，
  頁面卸載期改用 `navigator.sendBeacon`。
