# robot_api_server

Winstec Robot API v1.1 的實作（實作參考見 `docs/winstec_api_v1.1.md`）。

| 服務 | Port |
|---|---|
| HTTP（REST） | **5000** |
| WebSocket（事件） | **5001** |

REST 路徑前綴一律為 `/v1/robot`。錯誤回應格式為 `{"event": {"code": "..."}}`
（外層鍵名是 `event`，非 `error`——規格如此）。

## 模組結構

| 檔案 | 職責 |
|---|---|
| `main.py` | app 組裝、lifespan、uvicorn 啟動 |
| `ws_server.py` | 獨立 WebSocket server（5001），`robot_info` 每秒推播 + 事件式訊息 |
| `models.py` | Pydantic 模型（Location / Position / Point / VirtualWall / Group + 請求體） |
| `store.py` | points / walls / groups 的 staging 與磁碟持久化（文件 §8） |
| `ros_bridge.py` | rclpy 節點、Nav2 導航、cmd_vel 手動移動、電壓訂閱、模式切換 |
| `conversions.py` | 座標／角度換算（cm 整數 ↔ 公尺、度 ↔ 弧度） |
| `errors.py` | 規格錯誤碼與 `{"event": {...}}` 回應格式 |
| `keepout.py` | keepout mask 產生介面（目前是 stub，由另一個 agent 實作） |
| `config.py` | 環境變數與路徑設定 |
| `routers/` | 依資源分檔的端點 |

## 單位

對外一律 **公分整數 + 角度（度，0–360）**，只在與 ROS 互動的邊界換算為公尺／弧度。

## 端點

🟢 規格內：`/info`、`/move`、`/move/{pointId}`、`/manual/move`、`/stop`、
`/relocate/location`、`/relocate/{pointId}`、`/shutdown`、
`/points`、`/virtual-walls`、`/groups`（含 `/groups/actions/apply`）、
`/edits/commit`、`/edits/discard`

🟡 本專案擴充（OpenAPI 標記 `x-extension: true`）：
`/mode`、`/maps`、`/maps/{name}`、`/maps/{name}/image`、`/maps/{name}/metadata`、
`/maps/live/image`、`/maps/live/metadata`

## WebSocket 事件（port 5001）

| event | 觸發 | 訊息體 |
|---|---|---|
| `robot_info` | 每 1 秒 | `{event, op_mode, status, battery, location}` |
| `go_point` / `go_charging` / `switch_mode` / `relocate` / `power` | 事件 | `{event, code}` |

`code` 一律大寫：`COMPLETE` / `STUCK` / `ABORT` / `CHG_STA_NOT_FOUND` / `SHUTDOWN`。

## 環境變數

| 變數 | 預設 | 說明 |
|---|---|---|
| `ROBOT_MAP_PATH` | `<workspace>/map` | 地圖與 points/walls/groups JSON 的位置 |
| `ROBOT_API_HTTP_PORT` | `5000` | REST port |
| `ROBOT_API_WS_PORT` | `5001` | WebSocket port |
| `ROBOT_API_HOST` | `0.0.0.0` | 綁定位址 |
| `CORS_ORIGINS` | `*` | 逗號分隔的允許來源 |
| `ROBOT_BATTERY_MIN_V` / `ROBOT_BATTERY_MAX_V` | `21.0` / `25.2` | 電量換算（文件 §9） |
| `ROBOT_MANUAL_LINEAR` / `ROBOT_MANUAL_ANGULAR` | `0.15` / `0.5` | 手動移動速度 |

## 啟動

```bash
source install/setup.bash
ros2 launch robot_api_server api_server.launch.py
# 或
ros2 run robot_api_server api_server
```

互動式 API 文件（Swagger UI）：`http://<機器人IP>:5000/docs`

## 範例

```bash
# 取得機器人資訊
curl http://<機器人IP>:5000/v1/robot/info

# 建立點位（location 省略時使用機器人當前位置）
curl -X POST http://<機器人IP>:5000/v1/robot/points \
     -H "Content-Type: application/json" \
     -d '{"map": "test", "name": "p1", "type": "point"}'

# 導航到該點位
curl -X POST http://<機器人IP>:5000/v1/robot/move/pt_xxxxxxxx
```
