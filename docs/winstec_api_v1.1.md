# Winstec Robot API v1.1 — 實作參考

本文件是 `Winstec_RobotAPI_V1.1.pdf`（Rev 1.1, 2026-07-14）的實作用轉錄，
外加本專案的對應決策。**規格與本文件衝突時以 PDF 為準。**

標記說明：
- 🟢 **規格內** — PDF 明確定義
- 🟡 **本專案擴充** — PDF 沒有，但系統運作必需（見「擴充端點」章節）

---

## 1. 連線設定

| 服務 | 協定 | Port |
|---|---|---|
| HTTP Server | HTTP | **5000** |
| WebSocket Server | WebSocket | **5001** |

規格中 IP 寫 `192.168.125.88`，實作時綁 `0.0.0.0`。

---

## 2. 單位換算（關鍵）

規格的 Location/Position Object 用**整數座標**，ROS 內部用公尺浮點。

**本專案決定：單位為公分（cm）。**

```python
COORD_SCALE = 100          # ros 公尺 → api 整數

api_x = round(ros_x * COORD_SCALE)
ros_x = api_x / COORD_SCALE
```

**orientation 為浮點角度（degrees）**，範圍 0–360（規格範例出現 `356`）：

```python
api_deg = math.degrees(ros_yaw) % 360.0           # ros rad → api deg
ros_yaw = math.atan2(math.sin(r), math.cos(r))    # api deg → ros rad, 正規化到 -pi..pi
                                                   # 其中 r = math.radians(api_deg)
```

---

## 3. 共用資料模型（§7）

### 7.1 Operating Mode
| 值 | 說明 | 本專案對應 |
|---|---|---|
| `explore` | Mapping mode | slam_toolbox 執行中 |
| `navigate` | Navigation mode | Nav2 執行中 |

### 7.2 Robot Status
| 值 | 說明 |
|---|---|
| `init` | 系統初始化中 |
| `idle` | 閒置 |
| `relocating` | 重定位中 |
| `moving` | 導航中 |
| `go_charging` | 前往充電座中 |
| `switching_mode` | 模式切換中 |

### 7.3 Point Type
| 值 | 說明 |
|---|---|
| `point` | 一般點位 |
| `charge` | 充電座 |

### 7.4 Location Object
| 欄位 | 型別 | 必填 |
|---|---|---|
| `x` | integer | Yes |
| `y` | integer | Yes |
| `orientation` | float（度） | Yes |

### 7.5 Position Object
| 欄位 | 型別 | 必填 |
|---|---|---|
| `x` | integer | Yes |
| `y` | integer | Yes |

（無 orientation，用於虛擬牆端點）

### 7.6 Event Codes
`COMPLETE` / `STUCK` / `ABORT` / `CHG_STA_NOT_FOUND` / `SHUTDOWN`

> 注意：PDF 事件範例中寫成小寫 `"code": "complete"`，但 §7.6 表列為大寫。
> **本專案一律送大寫**，與 §7.6 一致。

---

## 4. 錯誤回應（§6）

回應體結構（注意外層鍵名是 `event`，非 `error`）：

```json
{ "event": { "code": "POINT_NOT_FOUND" } }
```

| HTTP | Error Code |
|---|---|
| 400 | `INVALID_INPUT`, `GET_LOCATION_FAILED`, `MAP_MISMATCH` |
| 404 | `POINT_NOT_FOUND`, `MAP_NOT_FOUND`, `GROUP_NOT_FOUND`, `MISSING_POINT_NAME`, `MISSING_VIRTUAL_WALL_NAME`, `MISSING_GROUP_NAME`, `NOT_FOUND_IN_GROUP` |
| 409 | `ROBOT_BUSY`, `NOT_IN_NAVIGATION_MODE`, `POINT_NOT_IN_MAP` |

---

## 5. REST 端點 🟢

所有路徑前綴 `/v1/robot`。

### 機器人資訊與移動

| Method | Path | 成功碼 | 說明 |
|---|---|---|---|
| GET | `/info` | 200 | 回 `{op_mode, status, battery, location}` |
| POST | `/move` | 200 | body `{type, location}`；回傳同 body |
| POST | `/move/{pointId}` | 200 | 回傳完整 Point 物件 |
| POST | `/manual/move` | 200 | body `{direction}`；回傳空 |
| POST | `/stop` | 200 | 軟停止並取消導航；回傳空 |
| POST | `/relocate/location` | 200 | body `{location}`；回傳 `{location}` |
| POST | `/relocate/{pointId}` | 200 | 回傳完整 Point 物件 |
| POST | `/shutdown` | 200 | 關機；回傳空 |

`manual/move` 的 `direction` 值：`stop` / `forward` / `backward` / `right`（順時針）/ `left`（逆時針）。

### Points

| Method | Path | 成功碼 |
|---|---|---|
| POST | `/points` | **201** |
| GET | `/points?map=name` | 200 → `{points: [...]}` |
| GET | `/points/{pointId}` | 200 |
| PATCH | `/points/{pointId}` | 200 |
| DELETE | `/points/{pointId}?map=name` | **204** |

- Create body：`map`(選)、`name`(必)、`type`(必)、`location`(選 — 省略時用機器人當前位置)
- `map` 省略時一律代表「目前載入的地圖」
- PATCH 所有欄位皆選填
- DELETE 的 `pointId` **選填** — 省略時刪除該 map 全部點位

Point 物件：`{id, map, name, type, location}`

### Virtual Walls

| Method | Path | 成功碼 |
|---|---|---|
| POST | `/virtual-walls` | **201** |
| GET | `/virtual-walls?map=name` | 200 → `{virtual_walls: [...]}` |
| PATCH | `/virtual-walls/{virtualWallId}` | 200 |
| DELETE | `/virtual-walls/{virtualWallId}?map=name` | **204** |

- Create body：`map`(選)、`name`(必)、`start_position`(必)、`end_position`(必)
- DELETE 的 `virtualWallId` **選填** — 省略時刪除該 map 全部虛擬牆
- 虛擬牆是**線段**，非多邊形

Virtual Wall 物件：`{id, map, name, start_position, end_position}`

### Groups

| Method | Path | 成功碼 |
|---|---|---|
| POST | `/groups` | **201** → `{id, map, name}` |
| GET | `/groups?map=name` | 200 → `{groups: [{id, map, name, is_enable}]}` |
| GET | `/groups/{groupId}` | 200 → 含 `virtual_walls` 完整物件陣列 |
| PATCH | `/groups/{groupId}` | 200 → `{id, map, name, is_enable}` |
| DELETE | `/groups/{groupId}` | **204** |
| POST | `/groups/{groupId}/virtual-walls` | **201** → `{group_id, virtual_wall_id}` |
| GET | `/groups/{groupId}/virtual-walls` | 200 → `{virtual_walls: [{id}]}`（**只有 id**） |
| DELETE | `/groups/{groupId}/virtual-walls/{virtualWallId}` | **204** |
| POST | `/groups/actions/apply` | 200 |

- Group 預設 `is_enable = false`
- **只有 enabled 的 group 會套用到機器人**
- 一面虛擬牆可屬於多個 group
- PATCH 只接受 `name`、`is_enable`，且**變更要等 apply 才生效**
- `GET /groups/{groupId}/virtual-walls` 回傳的物件**只含 `id`**（與 Get Group Details 不同）

### 編輯交易

| Method | Path | 成功碼 | 說明 |
|---|---|---|---|
| POST | `/edits/commit` | 200 | 提交 points / virtual walls / groups 全部變更並套用 |
| POST | `/edits/discard` | 200 | 捨棄全部未提交變更 |

**Apply vs Commit**（§4.2）：
- `apply` — 只把目前的 group 狀態套用到機器人
- `commit` — 提交所有虛擬牆與 group 變更，然後套用

---

## 6. WebSocket 事件 🟢

Port 5001。訊息皆為 JSON。

| event | 觸發 | 訊息體 |
|---|---|---|
| `robot_info` | **每 1 秒** | `{event, op_mode, status, battery, location}` |
| `go_point` | 事件 | `{event, code}` |
| `go_charging` | 事件 | `{event, code}` |
| `switch_mode` | 事件 | `{event, code}` |
| `relocate` | 事件 | `{event, code}` |
| `power` | 事件 | `{event, code}` |

`code` 取自 §7.6 Event Codes。

---

## 7. 擴充端點 🟡

規格沒有建圖、地圖管理與模式切換端點，但：
- Points 全都要指定 `map`，表示地圖必須先存在
- §7.1 定義了 `explore`/`navigate` 兩種 op_mode、§5 也有 `switch_mode` 事件，
  **卻沒有任何切換模式的端點**

因此本專案在 `/v1/robot` 下新增以下擴充，**全部以 `x-extension: true` 註記於 OpenAPI**：

| Method | Path | 說明 |
|---|---|---|
| POST | `/mode` | body `{mode: "explore"\|"navigate", map?}`；切換模式，觸發 `switch_mode` 事件 |
| GET | `/maps` | 列出所有地圖 |
| POST | `/maps` | body `{name}`；儲存目前建圖結果 |
| DELETE | `/maps/{name}` | 刪除地圖及其 points/walls/groups |
| GET | `/maps/{name}/image` | 地圖 PNG（前端顯示用） |
| GET | `/maps/{name}/metadata` | 解析度、origin、尺寸 |
| GET | `/maps/live/image` | **建圖中**的即時地圖 PNG（訂閱 `/map` 轉檔） |
| GET | `/maps/live/metadata` | 建圖中的即時地圖 resolution / origin / 尺寸 |

`/maps/live/*` 讓前端在建圖時以 1–2 Hz 輪詢顯示地圖成長，
取代 rosbridge —— 前端因此**只依賴 `/v1`，不需要 roslib**。
機器人即時位姿由 WebSocket 的 `robot_info` 事件提供（每秒一次）。

---

## 8. 儲存契約（agent 間共用）

全部位於 `$ROBOT_MAP_PATH`（預設 `~/base_dev/map`）：

```
<map>.yaml / <map>.pgm            # 地圖本體（既有格式）
<map>.points.json                 # [{id, map, name, type, location}]
<map>.virtual_walls.json          # [{id, map, name, start_position, end_position}]
<map>.groups.json                 # [{id, map, name, is_enable, virtual_wall_ids: []}]
<map>.keepout.pgm / .keepout.yaml # 由虛擬牆產生的 Nav2 keepout mask
```

**JSON 內一律存 API 單位（cm 整數 + 度）**，只在與 ROS 互動的邊界換算。

### staging 模型

- 未提交的變更**只存在記憶體**，磁碟永遠是已提交狀態
- `discard` → 丟棄記憶體暫存
- `commit` → 寫入磁碟 → 重新產生 keepout mask → 通知 Nav2 重載
- `groups/actions/apply` → 只重新產生 mask 並通知重載，**不寫入磁碟**

### keepout mask 產生規則

1. 讀 `<map>.yaml` 取 `resolution` 與 `origin`
2. 建立與地圖同尺寸、全 0（自由）的灰階影像
3. 對**每個 enabled group 內的每面虛擬牆**，把線段以 Bresenham 畫成值 254（禁止）
4. 線寬取 `max(1, ceil(robot_radius / resolution))`，`robot_radius` 用 0.25 m（footprint 內切）
5. 輸出 `<map>.keepout.pgm` + `.keepout.yaml`（`mode: scale`）

---

## 9. 其他實作決策

**ID 格式** — 比照規格範例（22 字元 base64url）：

```python
import secrets
point_id = f"pt_{secrets.token_urlsafe(16)}"
wall_id  = f"vw_{secrets.token_urlsafe(16)}"
group_id = f"gp_{secrets.token_urlsafe(16)}"
```

**電池百分比** — 24V 鋰電 6S，由 `/motor/voltage` 換算：

```python
BATTERY_MIN_V = 21.0   # 0%
BATTERY_MAX_V = 25.2   # 100%
pct = int(round(max(0.0, min(1.0, (v - BATTERY_MIN_V) / (BATTERY_MAX_V - BATTERY_MIN_V))) * 100))
```

**充電座** — 無充電硬體。`type: "charge"` 的點位可建立並導航前往；
`go_charging` 事件在找不到 charge 點位時送 `CHG_STA_NOT_FOUND`，
抵達時送 `COMPLETE`，但**不會有實際充電行為**。

**Shutdown** — `POST /shutdown` 先送 `power` 事件（code `SHUTDOWN`），
再執行系統關機。
