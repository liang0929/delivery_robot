# 專案架構指南

本文件幫助新加入的開發者快速了解專案結構與設計理念。

---

## 1. 專案概述

這是一個基於 **ROS2 Humble** 的差速驅動移動機器人控制系統，運行在 **NVIDIA Jetson** 平台上。

### 核心功能
- **遙控移動** - 透過網頁搖桿或鍵盤控制
- **SLAM 建圖** - 使用 LiDAR 掃描環境並建立 2D 地圖
- **自主導航** - 在已知地圖中自動規劃路徑並移動到目標點

### 硬體配置
```
┌─────────────────────────────────────────────────────────┐
│                    NVIDIA Jetson                        │
├─────────────────────────────────────────────────────────┤
│  USB ──► 馬達驅動器 (AGV-BLD-2S) ──► 左右輪馬達        │
│  USB ──► LiDAR (SLAMTEC A2M12)                         │
│  I2C ──► IMU (BNO055)                                  │
└─────────────────────────────────────────────────────────┘
```

---

## 2. 系統架構

### 2.1 整體架構圖

```
┌─────────────────────────────────────────────────────────────────────┐
│                         使用者介面層                                 │
├─────────────────────────────────────────────────────────────────────┤
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐          │
│  │  Web 前端    │    │    RViz2     │    │   鍵盤控制   │          │
│  │  (React)     │    │  (視覺化)    │    │              │          │
│  └──────┬───────┘    └──────────────┘    └──────┬───────┘          │
│         │                                        │                  │
│         ▼                                        ▼                  │
│  ┌──────────────────────────────┐        /cmd_vel                  │
│  │  Robot API Server (FastAPI)  │◄───────────────┘                 │
│  │  REST 5000  /  WebSocket 5001│                                   │
│  │  Winstec Robot API v1.1      │                                   │
│  └──────────────────────────────┘                                   │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         ROS2 節點層                                  │
├─────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐               │
│  │ HS Motor    │   │  LiDAR      │   │    IMU      │               │
│  │ Controller  │   │  (sllidar)  │   │  (bno055)   │               │
│  └──────┬──────┘   └──────┬──────┘   └──────┬──────┘               │
│         │                 │                 │                       │
│    /cmd_vel          /scan            /imu/data                    │
│    /odom_raw                                                        │
│    /motor/voltage                                                   │
│         │                 │                 │                       │
│         ▼                 ▼                 ▼                       │
│  ┌─────────────────────────────────────────────────────────┐       │
│  │                    EKF (robot_localization)              │       │
│  │              融合 odom + IMU → /odometry/filtered        │       │
│  └─────────────────────────────────────────────────────────┘       │
│                              │                                      │
│                              ▼                                      │
│  ┌─────────────────────────────────────────────────────────┐       │
│  │                      Nav2 Stack                          │       │
│  │  ┌───────────┐  ┌───────────┐  ┌───────────┐           │       │
│  │  │ AMCL      │  │ Planner   │  │Controller │           │       │
│  │  │ (定位)    │  │ (路徑規劃)│  │ (軌跡追蹤)│           │       │
│  │  └───────────┘  └───────────┘  └───────────┘           │       │
│  └─────────────────────────────────────────────────────────┘       │
│                                                                      │
└─────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         硬體抽象層                                   │
├─────────────────────────────────────────────────────────────────────┤
│  /dev/motor (USB-RS232)    /dev/lidar (USB)    /dev/i2c-7          │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 TF 座標樹

```
map
 └── odom (由 AMCL 或 slam_toolbox 發布)
      └── base_footprint
           └── base_link
                ├── laser (LiDAR)
                └── imu_link (IMU)
```

### 2.3 資料流

```
使用者操作 (搖桿/鍵盤/導航目標)
         │
         ▼
    /cmd_vel (Twist)
         │
         ▼
┌─────────────────┐
│  HS Motor       │──► 馬達驅動器 ──► 輪子轉動
│  Controller     │
└────────┬────────┘
         │
    /odom_raw (輪子里程計)
         │
         ▼
┌─────────────────┐
│      EKF        │◄── /imu/data
└────────┬────────┘
         │
    /odometry/filtered
         │
         ▼
┌─────────────────┐
│  Nav2 / SLAM    │◄── /scan (LiDAR)
└─────────────────┘
```

---

## 3. 目錄結構

```
base_dev/
├── src/
│   ├── motor_control/           # 馬達控制套件
│   │   ├── config/
│   │   │   └── hs_motor_config.yaml    # 馬達參數、EKF 設定
│   │   ├── launch/
│   │   │   ├── bringup.launch.py       # 統一系統啟動（推薦）
│   │   │   ├── hs_motor_controller.launch.py
│   │   │   └── robot_state_publisher.launch.py
│   │   └── motor_control/
│   │       └── hs_motor_controller.py  # HS 協議馬達控制器
│   │
│   ├── nav2/                    # 導航套件
│   │   ├── config/
│   │   │   └── nav2_params.yaml        # Nav2 參數
│   │   └── launch/
│   │       ├── mapping.launch.py       # SLAM 建圖
│   │       └── autonomous_navigation.launch.py  # 自主導航
│   │
│   ├── robot_api_server/        # REST API 伺服器
│   │   └── robot_api_server/
│   │       └── main.py                 # FastAPI 應用
│   │
│   ├── robot_web_frontend/      # React 網頁前端
│   │   ├── src/
│   │   │   ├── components/             # React 元件
│   │   │   ├── services/               # API/WebSocket 服務
│   │   │   └── pages/                  # 頁面
│   │   └── package.json
│   │
│   ├── ros-imu-bno055/          # IMU 驅動 (submodule)
│   ├── sllidar_ros2/            # LiDAR 驅動 (submodule)
│   └── map/                     # 儲存的地圖檔案
│
├── install/                     # colcon build 輸出
├── build/                       # 建置暫存
├── log/                         # 日誌
├── README.md                    # 使用說明
├── ARCHITECTURE.md              # 本文件
└── 99-robot-usb.rules          # udev 規則
```

---

## 4. 核心模組說明

### 4.1 HS Motor Controller (`hs_motor_controller.py`)

**功能：** 與馬達驅動器通訊，實現速度控制和里程計回傳

**HS 協議封包格式：**
```
詢問封包 (16 bytes):
AA + 地址 + 回傳類型 + 故障清除 + 保留 + A控制 + B控制 + A方向 + B方向 + A轉速(2B) + B轉速(2B) + 55 + CRC16

應答封包 (16 bytes):
55 + 地址 + A電流(2B) + B電流(2B) + A轉速(2B) + B轉速(2B) + 電壓(2B) + 故障 + AA + CRC16
```
（欄位順序以實測硬體行為為準，詳見 `hs_motor_controller.py` docstring）

**關鍵參數（`hs_motor_config.yaml`）：**
- `wheel_separation`: 0.27m (左右輪間距)
- `wheel_radius`: 0.065m (輪子半徑)
- `gear_ratio`: 20.0 (減速比，馬達 RPM = 輪 RPM × 20)
- `min_rpm` / `max_rpm`: 100 / 3000 (馬達轉速範圍，非零命令低於 min_rpm 時 clamp 到 min_rpm)

**訂閱/發布：**
| Topic | 類型 | 方向 | 說明 |
|-------|------|------|------|
| `/cmd_vel` | Twist | 訂閱 | 速度命令 |
| `/odom_raw` | Odometry | 發布 | 輪子里程計 |
| `/motor/voltage` | Float32 | 發布 | 電壓 |

### 4.2 EKF (robot_localization)

**功能：** 融合輪子里程計和 IMU 數據，產生更準確的位置估計

**設定檔：** `hs_motor_config.yaml` 中的 `ekf_filter_node` 區段

**輸入：**
- `/odom_raw` - 輪子里程計 (x, y, yaw)
- `/imu/data` - IMU 角速度和方向

**輸出：**
- `/odometry/filtered` - 融合後的里程計
- `odom → base_footprint` TF

### 4.3 Nav2 Stack

**元件：**
| 元件 | 功能 |
|------|------|
| AMCL | 粒子濾波定位 |
| Planner Server | 全域路徑規劃 (NavFn) |
| Controller Server | 局部軌跡追蹤 (Regulated Pure Pursuit) |
| BT Navigator | 行為樹導航管理 |
| Costmap 2D | 障礙物地圖 |

**關鍵參數 (`nav2_params.yaml`)：**
- `desired_linear_vel`: 0.05 m/s
- `max_angular_vel`: 0.4 rad/s
- footprint: 0.5m × 0.5m 方形（`inflation_radius`: 0.35m）

### 4.4 Web Frontend

**技術棧：**
- React 18 + TypeScript
- Vite (建置工具)
- 原生 fetch + WebSocket（**不使用 roslibjs**，只依賴 Robot API）

**架構：**
```
┌─────────────────────────────────────────────┐
│              React App                       │
├─────────────────────────────────────────────┤
│  Pages: 建圖 / 點位設定 / 自動導航           │
├─────────────────────────────────────────────┤
│  Components: MapCanvas / DPad / StatusBar   │
├─────────────────────────────────────────────┤
│  api/robot.api.ts    ws/RobotSocket.ts      │
├─────────────────────────────────────────────┤
│    REST (5000)          WebSocket (5001)    │
└─────────────────────────────────────────────┘
```

---

## 5. 通訊協議

對外介面遵循 **Winstec Robot API v1.1**，完整規格與實作決策見
`docs/winstec_api_v1.1.md`，原始文件為 `Winstec_RobotAPI_V1.1.pdf`。

### 5.1 REST API (Port 5000)

路徑前綴一律 `/v1/robot`。錯誤回應格式為 `{"event": {"code": "..."}}`。

| 端點 | 方法 | 用途 |
|------|------|------|
| `/info` | GET | 取得 op_mode / status / battery / location |
| `/move`、`/move/{pointId}` | POST | 導航至座標或既有點位 |
| `/manual/move` | POST | 手動移動（forward/backward/left/right/stop）|
| `/stop` | POST | 軟停止並取消導航 |
| `/relocate/location`、`/relocate/{pointId}` | POST | 設定機器人位姿 |
| `/points`、`/virtual-walls`、`/groups` | CRUD | 點位、虛擬牆與群組管理 |
| `/edits/commit`、`/edits/discard` | POST | 編輯交易的提交與捨棄 |
| `/mode`、`/maps/*` | — | 🟡 本專案擴充：模式切換與地圖管理 |

**座標單位**：對外為**公分整數** + 角度（度，0–360），僅在與 ROS 互動的
邊界換算為公尺／弧度。

### 5.2 WebSocket 事件 (Port 5001)

- `robot_info` — 每秒推播 op_mode / status / battery / location
- `go_point`、`go_charging`、`switch_mode`、`relocate`、`power` — 事件式，
  帶事件碼 `COMPLETE` / `STUCK` / `ABORT` / `CHG_STA_NOT_FOUND` / `SHUTDOWN`

### 5.3 虛擬牆

Virtual Wall 為**線段**，歸屬於 Group；只有 `is_enable: true` 的 Group 會生效。
`commit` 或 `groups/actions/apply` 時由 `nav2.keepout` 產生 keepout mask
（`<map>.keepout.pgm`／`.yaml`），經 Nav2 的 `KeepoutFilter` 套用到 costmap。

> mask 的 `negate: 1` 不可省略 —— mask 語意是「254 = 禁行」，與一般地圖相反，
> 少了這個旗標虛擬牆會靜默失效。

---

## 6. 開發指南

### 6.1 建置專案

```bash
# 完整建置
colcon build

# 建置單一套件
colcon build --packages-select motor_control

# 建置前端
cd src/robot_web_frontend && npm install && npm run build
```

### 6.2 測試流程

```bash
# 1. 啟動核心
ros2 launch motor_control bringup.launch.py

# 2. 啟動 LiDAR
ros2 service call /start_motor std_srvs/srv/Empty

# 3. 檢查 topics
ros2 topic list
ros2 topic echo /motor/voltage

# 4. 測試移動
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 6.3 除錯技巧

**TF 問題：**
```bash
ros2 run tf2_tools view_frames  # 產生 TF 樹 PDF
ros2 run tf2_ros tf2_echo map base_link
```

**Topic 問題：**
```bash
ros2 topic hz /scan  # 檢查發布頻率
ros2 topic info /cmd_vel  # 檢查發布者/訂閱者
```

**Nav2 問題：**
```bash
ros2 lifecycle list /map_server  # 檢查生命週期狀態
ros2 lifecycle set /map_server activate
```

---

## 7. 常見問題

### Q: LiDAR 沒有數據？
1. 檢查 `/dev/lidar` 是否存在
2. 調用 `/start_motor` 服務啟動馬達
3. 檢查 `ros2 topic hz /scan`

### Q: 機器人不移動？
1. 檢查 `/cmd_vel` 是否有數據
2. 檢查馬達驅動器電源
3. 檢查串口連接 `/dev/motor`

### Q: 導航時地圖不顯示？
1. 檢查 map_server 生命週期狀態
2. 確認地圖檔案路徑正確
3. 檢查 AMCL 是否發布 `map → odom` TF

### Q: 前端顯示 Disconnected？
1. 確認 API server 正在運行：`curl http://<IP>:5000/v1/robot/info`
2. 確認 WebSocket port 5001 也有監聽：`ss -tlnp | grep 5001`
3. 檢查 `robot.config.ts` 中的 IP 設定（或 build 時的 `VITE_ROBOT_IP`）
4. 確認防火牆沒有阻擋 5000 / 5001

---

## 8. 未來規劃

- [ ] 多機器人協作
- [ ] 3D SLAM (使用深度相機)
- [ ] 語音控制整合
- [ ] 自動充電對接
- [ ] 手機 App 控制
