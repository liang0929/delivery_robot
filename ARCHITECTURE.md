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
│  ┌──────────────┐                        /cmd_vel                  │
│  │  rosbridge   │◄──── WebSocket ────────────────┘                 │
│  │  (9090)      │                                                   │
│  └──────┬───────┘                                                   │
│         │                                                           │
│  ┌──────┴───────┐                                                   │
│  │  API Server  │◄──── REST API (8000)                             │
│  │  (FastAPI)   │                                                   │
│  └──────────────┘                                                   │
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
AA + 地址 + 數據類型 + 故障清除 + 保留 + A控制 + A方向 + A轉速(2B) + B控制 + B方向 + B轉速(2B) + 55 + CRC16

應答封包 (16 bytes):
55 + 地址 + A電流(2B) + B電流(2B) + A轉速(2B) + B轉速(2B) + 電壓(2B) + 故障 + AA + CRC16
```

**關鍵參數：**
- `wheel_separation`: 0.37m (左右輪間距)
- `wheel_radius`: 0.0775m (輪子半徑)
- `max_rpm`: 200 (最大轉速)

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
- `robot_radius`: 0.22m

### 4.4 Web Frontend

**技術棧：**
- React 18 + TypeScript
- Vite (建置工具)
- roslibjs (ROS WebSocket 通訊)
- Axios (REST API)

**架構：**
```
┌─────────────────────────────────────────────┐
│              React App                       │
├─────────────────────────────────────────────┤
│  Pages: RemoteControl / SlamMapping / Nav   │
├─────────────────────────────────────────────┤
│  Components: Joystick / MapView / Panels    │
├─────────────────────────────────────────────┤
│  Services: rosbridge.service / api.service  │
├─────────────────────────────────────────────┤
│       rosbridge (9090)    API (8000)        │
└─────────────────────────────────────────────┘
```

---

## 5. 通訊協議

### 5.1 rosbridge WebSocket (Port 9090)

用於即時 ROS2 topic 訂閱/發布：
- 發布 `/cmd_vel` (搖桿控制)
- 訂閱 `/map` (地圖顯示)
- 訂閱 `/odometry/filtered` (機器人位置)
- 訂閱 `/motor/voltage` (電壓監控)

### 5.2 REST API (Port 8000)

用於控制命令和狀態查詢：

| 端點 | 方法 | 用途 |
|------|------|------|
| `/robot/start` | POST | 啟動 bringup.launch.py (僅核心節點) |
| `/robot/stop` | POST | 停止機器人核心 |
| `/slam/start` | POST | 啟動 SLAM |
| `/slam/save_map` | POST | 儲存地圖 |
| `/navigate_to_goal` | POST | 發送導航目標 |

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
1. 確認 rosbridge 正在運行 (port 9090)
2. 檢查 `robot.config.ts` 中的 IP 設定
3. 確認防火牆沒有阻擋

---

## 8. 未來規劃

- [ ] 多機器人協作
- [ ] 3D SLAM (使用深度相機)
- [ ] 語音控制整合
- [ ] 自動充電對接
- [ ] 手機 App 控制
