# ROS2 差速驅動機器人控制專案

這是一個為差速驅動移動機器人設計的完整 ROS2 專案，整合了感測器、馬達控制、導航功能以及網頁控制介面。

---

## 目錄
1. [硬體設置](#1-硬體設置)
2. [環境依賴](#2-環境依賴)
3. [安裝與建置](#3-安裝與建置)
4. [使用說明](#4-使用說明)
   - [一鍵啟動（推薦）](#41-一鍵啟動推薦)
   - [網頁前端控制](#42-網頁前端控制)
   - [手動鍵盤控制](#43-手動鍵盤控制)
   - [SLAM 建圖](#44-slam-建圖)
   - [自主導航](#45-自主導航)
5. [模擬模式](#5-模擬模式)
6. [開發者與除錯](#6-開發者與除錯)

---

## 1. 硬體設置

在啟動系統前，請確保硬體已正確連接：

| 設備 | 連接介面 | 說明 |
|------|----------|------|
| 馬達驅動器 (AGV-BLD-2S) | `/dev/motor` | USB-RS232 轉接，HS 協議，地址 127 |
| LiDAR (SLAMTEC A2M12) | `/dev/lidar` | USB 連接，256000 baud |
| IMU (BNO055) | `/dev/i2c-7` | I2C 連接，地址 0x28 |

### USB 設備固定路徑

為避免 USB 設備編號變動，系統已安裝 udev 規則（`/etc/udev/rules.d/99-robot-devices.rules`）建立固定符號連結：

```bash
# 確認符號連結存在
ls -l /dev/motor /dev/lidar

# 修改規則後重新載入
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 馬達驅動器設定

本專案使用泰映科技 (TROY) AGV-BLD-2S 雙軸無刷馬達驅動器，通過 USB-RS232 直接連接 Jetson。

**驅動器參數：**
- 通訊協議：HS 協議 (RS-232 模式)
- 波特率：115200
- 設備地址：127
- RPM 範圍：100-3000

**馬達方向設定：**
- `invert_motor_a: true` (A 馬達反轉)
- `invert_motor_b: false`

## 2. 環境依賴

- **ROS2 Humble Hawksbill**
- **Python 套件**:
  ```bash
  pip install pyserial tornado netifaces pymongo Pillow cbor2
  ```
- **ROS2 套件**:
  ```bash
  sudo apt-get update
  sudo apt-get install ros-humble-robot-localization \
                       ros-humble-teleop-twist-keyboard \
                       ros-humble-rosbridge-suite \
                       ros-humble-nav2-bringup \
                       ros-humble-slam-toolbox
  ```
- **Node.js** (網頁前端):
  ```bash
  curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
  sudo apt-get install -y nodejs
  ```

## 3. 安裝與建置

```bash
# 建置所有套件
colcon build

# 或使用自動化腳本
./build_ros2.sh

# 安裝前端依賴
cd src/robot_web_frontend
npm install
```

## 4. 使用說明

### 快速指令 (Make)

本專案提供 Makefile 快速指令，輸入 `make help` 查看所有可用命令：

```bash
# 服務管理 (使用 systemd)
make start      # 啟動所有服務
make stop       # 停止所有服務
make restart    # 重啟所有服務 (修改程式碼後使用)
make status     # 查看服務狀態
make logs       # 查看即時日誌

# 開發模式 (前景執行)
make dev        # 開發模式 (核心 + 前端)
make dev-core   # 只啟動核心
make dev-web    # 只啟動前端

# 模擬模式 (不需要硬體)
make sim        # 模擬模式 + 前端
make sim-core   # 只啟動模擬核心
make sim-room   # 模擬方形房間
make sim-corridor  # 模擬走廊

# 建置
make build      # 建置 ROS2 + 前端
make build-ros  # 只建置 ROS2
make build-web  # 只建置前端

# 服務安裝
make install    # 安裝為 systemd 服務 (開機自啟)
make uninstall  # 移除服務
```

### 4.1. 一鍵啟動（推薦）

**使用 Make 指令（最簡單）：**

```bash
make dev        # 開發模式
# 或
make start      # 使用 systemd 服務
```

**使用 ROS2 Launch（手動）：**

每次開啟新的終端機時，請記得先 source 工作區環境：

```bash
source install/setup.bash
```

使用統一 launch 檔案啟動所有核心節點與 Web 服務：

```bash
ros2 launch motor_control bringup.launch.py
```

此指令會啟動：
- 馬達控制器、LiDAR、IMU、EKF 狀態估算
- rosbridge WebSocket 通訊
- API Server（REST 5000 / WebSocket 事件 5001）
- robot_state_publisher（URDF TF；未偵測到 URDF 時退回靜態 TF）

啟動後，開啟前端開發伺服器：
```bash
cd src/robot_web_frontend
npm run dev
```

**開啟瀏覽器：** `http://<機器人IP>:3000`

> 網頁前端可動態切換模式（遙控、建圖、導航），無需重新啟動 launch。

### 4.2. 網頁前端控制

網頁前端提供三大功能：遙控機器人、SLAM 建圖、自主導航。

**開啟瀏覽器：** `http://<機器人IP>:3000`

**網頁功能：**
| 功能 | 說明 |
|------|------|
| Remote Control | 虛擬搖桿遙控、啟動/停止機器人核心、電壓電流監控 |
| SLAM Mapping | 啟動/停止建圖、儲存地圖、即時地圖顯示 |
| Navigation | 點擊地圖選擇目標、自動導航、取消導航 |

### 4.3. 手動鍵盤控制

**方法一：使用 launch 檔案 + 另開終端**

終端 1 - 啟動馬達控制器：
```bash
ros2 launch motor_control keyboard_control.launch.py
```

終端 2 - 啟動鍵盤控制：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
> 提示：啟動後按 `z` 鍵降低速度（每次 -10%），按 `q` 增加速度

**鍵盤操作說明：**
| 按鍵 | 動作 |
|------|------|
| `i` | 前進 |
| `,` | 後退 |
| `j` | 左轉 |
| `l` | 右轉 |
| `k` | 停止 |
| `q`/`z` | 增加/減少速度 |

### 4.4. SLAM 建圖

**方法一：使用網頁前端（推薦）**
1. 使用統一 launch 啟動系統 (參考 4.1)
2. 開啟網頁前端，點擊 "SLAM Mapping" 頁面
3. 點擊 "Start Mapping" 開始建圖
4. 使用虛擬搖桿控制機器人移動
5. 輸入地圖名稱，點擊 "Save Map" 儲存

**方法二：使用命令列**

1. **啟動建圖模式** (終端 1) - 包含馬達、LiDAR、IMU、SLAM:
   ```bash
   ros2 launch nav2 mapping.launch.py
   ```

2. **啟動鍵盤控制** (終端 2):
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   > 啟動後按 `z` 降低速度至約 0.05 m/s（建議建圖時用低速）

3. **啟動 RViz2 視覺化** (終端 3，可選):
   ```bash
   rviz2
   ```
   在 RViz2 中加入顯示：Add → By topic → `/map` → Map

4. **儲存地圖**:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f map/<地圖名稱>
   ```
   > 地圖統一存放於專案根目錄的 `map/`（API server 的 `ROBOT_MAP_PATH` 預設也指向這裡）。

**SLAM 參數調整：**
SLAM Toolbox 參數配置檔位於 `src/nav2/config/slam_toolbox_params.yaml`，可調整：
- `resolution`: 地圖解析度（預設 0.05m）
- `max_laser_range`: LiDAR 最大有效範圍
- `do_loop_closing`: 迴環檢測開關

### 4.5. 自主導航

**方法一：使用網頁前端（推薦）**
1. 確保已有儲存的地圖
2. 使用統一 launch 啟動系統 (參考 4.1)
3. 開啟網頁前端，點擊 "Navigation" 頁面
4. 點擊 "Start Navigation" 開始導航模式
5. 點擊地圖選擇目標位置
6. 機器人自動導航至目標

**方法二：使用 RViz2**

1. **啟動系統** (終端 1):
   ```bash
   ros2 launch motor_control bringup.launch.py enable_web:=false
   ```

2. **啟動導航模式** (終端 2):
   ```bash
   ros2 launch nav2 autonomous_navigation.launch.py
   ```

3. **啟動 RViz2** (終端 3):
   ```bash
   rviz2
   ```

4. **在 RViz2 中操作**:
   - 使用 **"2D Pose Estimate"** 設定機器人初始位置
   - 使用 **"Nav2 Goal"** 設定目標點

## 5. 模擬模式

在沒有實體硬體的環境下，可使用模擬模式進行開發與測試。

### 5.1. 啟動模擬模式

**使用 Make 指令（推薦）：**

```bash
make sim          # 模擬模式 + 前端服務
make sim-core     # 只啟動模擬核心
make sim-room     # 方形房間場景
make sim-corridor # 走廊場景
```

**使用 ROS2 Launch：**

```bash
ros2 launch motor_control bringup.launch.py simulation:=true
```

模擬模式會自動啟動以下 Mock 節點取代實體硬體：

| Mock 節點 | 取代硬體 | 功能說明 |
|-----------|----------|----------|
| `mock_motor_controller` | 馬達驅動器 | 差速驅動運動學模擬、里程計發布 |
| `mock_lidar` | LiDAR | 模擬房間環境的 LaserScan 數據 |
| `mock_imu` | IMU | 模擬 IMU 數據（含角速度整合） |

### 5.2. 模擬環境配置

Mock LiDAR 預設模擬 5m × 5m 的方形房間，可透過 `sim_scene` 參數切換場景：

```bash
ros2 launch motor_control bringup.launch.py simulation:=true sim_scene:=corridor
```

| 參數 | 預設值 | 說明 |
|------|--------|------|
| `sim_scene` | `room` | 模擬場景：`room`（方形房間）/ `corridor`（走廊）/ `empty`（空曠） |

> 房間尺寸由 mock_lidar 節點的 `room_width` / `room_height` 參數決定（預設各 5.0 m），
> 目前無法經由 bringup launch 覆寫，需直接以節點參數指定。

### 5.3. 模擬模式下的功能測試

模擬模式支援完整功能測試：

1. **SLAM 建圖**：使用網頁前端或 RViz2 進行建圖測試
2. **地圖儲存**：儲存模擬環境建立的地圖
3. **自主導航**：載入地圖並測試導航功能

> 注意：模擬模式下的 LiDAR 數據為簡化的幾何計算，與實際感測器特性有差異。

## 6. 開發者與除錯

### 常用監控指令

```bash
# 監控里程計數據
ros2 topic echo /odom_raw

# 監控融合後里程計
ros2 topic echo /odometry/filtered

# 監控速度指令
ros2 topic echo /cmd_vel

# 監控 LiDAR 掃描數據
ros2 topic echo /scan

# 監控馬達電壓
ros2 topic echo /motor/voltage

# 監控馬達電流
ros2 topic echo /motor/current_a
ros2 topic echo /motor/current_b

# 查看 TF 樹
ros2 run tf2_tools view_frames

# 查看所有 topics
ros2 topic list

# 查看 topic 發布頻率
ros2 topic hz /odom_raw
```

### 單獨測試各感測器

```bash
# 測試馬達控制器
ros2 run motor_control hs_motor_controller --ros-args -p device_id:=127

# 測試 LiDAR
ros2 launch sllidar_ros2 sllidar_a2m12_launch.py serial_port:=/dev/lidar

# 測試 IMU
ros2 run imu_bno055 bno055_i2c_node --ros-args -p device:=/dev/i2c-7 -p address:=40
```

### ROS2 Topics 一覽

| Topic | 類型 | 說明 |
|-------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 速度命令輸入 |
| `/odom_raw` | nav_msgs/Odometry | 馬達里程計輸出 |
| `/odometry/filtered` | nav_msgs/Odometry | EKF 融合後里程計 |
| `/scan` | sensor_msgs/LaserScan | LiDAR 掃描數據 |
| `/imu/data` | sensor_msgs/Imu | IMU 數據 |
| `/motor/voltage` | std_msgs/Float32 | 馬達控制器電壓 |
| `/motor/current_a` | std_msgs/Float32 | A 馬達電流 |
| `/motor/current_b` | std_msgs/Float32 | B 馬達電流 |
| `/map` | nav_msgs/OccupancyGrid | SLAM 地圖 |
| `/tf` | tf2_msgs/TFMessage | 座標轉換 |

### API Server 端點

Robot API 遵循 **Winstec Robot API v1.1**（完整規格見 `docs/winstec_api_v1.1.md`）：

- REST：port **5000**，路徑前綴一律 `/v1/robot`
- WebSocket 事件推播：port **5001**（`robot_info` 每秒推播 + 事件式訊息）

| 端點 | 方法 | 說明 |
|------|------|------|
| `/v1/robot/info` | GET | 取得 op_mode / status / battery / location |
| `/v1/robot/move`、`/v1/robot/move/{pointId}` | POST | 導航至座標或既有點位 |
| `/v1/robot/manual/move` | POST | 手動移動（forward/backward/left/right/stop） |
| `/v1/robot/stop` | POST | 軟停止並取消導航 |
| `/v1/robot/relocate/location`、`/v1/robot/relocate/{pointId}` | POST | 設定機器人位姿 |
| `/v1/robot/shutdown` | POST | 關機 |
| `/v1/robot/points`、`/v1/robot/virtual-walls`、`/v1/robot/groups` | CRUD | 點位、虛擬牆與群組管理 |
| `/v1/robot/edits/commit`、`/v1/robot/edits/discard` | POST | 編輯交易的提交與捨棄 |
| `/v1/robot/mode`、`/v1/robot/maps/*` | — | 本專案擴充：模式切換與地圖管理 |

互動式 API 文件（Swagger UI）：`http://<機器人IP>:5000/docs`
