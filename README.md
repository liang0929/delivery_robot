# ROS2 差速驅動機器人控制專案

這是一個為差速驅動移動機器人設計的完整 ROS2 專案，整合了感測器、馬達控制、導航功能以及網頁控制介面。

---

## 目錄
1. [硬體設置](#1-硬體設置)
2. [環境依賴](#2-環境依賴)
3. [安裝與建置](#3-安裝與建置)
4. [使用說明](#4-使用說明)
   - [啟動機器人核心](#41-啟動機器人核心)
   - [網頁前端控制](#42-網頁前端控制)
   - [手動鍵盤控制](#43-手動鍵盤控制)
   - [SLAM 建圖](#44-slam-建圖)
   - [自主導航](#45-自主導航)
5. [開發者與除錯](#5-開發者與除錯)

---

## 1. 硬體設置

在啟動系統前，請確保硬體已正確連接：

| 設備 | 連接介面 | 說明 |
|------|----------|------|
| 馬達驅動器 (AGV-BLD-2S) | `/dev/motor` | USB-RS232 轉接，HS 協議，地址 127 |
| LiDAR (SLAMTEC A2M12) | `/dev/lidar` | USB 連接，256000 baud |
| IMU (BNO055) | `/dev/i2c-7` | I2C 連接，地址 0x28 |

### USB 設備固定路徑

為避免 USB 設備編號變動，已設定 udev 規則建立固定符號連結：

```bash
# 安裝 udev 規則
sudo cp 99-robot-usb.rules /etc/udev/rules.d/
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

每次開啟新的終端機時，請記得先 source 工作區環境：

```bash
source install/setup.bash
```

### 4.1. 啟動機器人核心

此指令會啟動所有基礎節點，包括馬達控制器、LiDAR、IMU 以及 EKF 狀態估算。

```bash
ros2 launch motor_control full_system.launch.py
```

啟動後需要啟動 LiDAR 馬達：
```bash
ros2 service call /start_motor std_srvs/srv/Empty
```

### 4.2. 網頁前端控制

網頁前端提供三大功能：遙控機器人、SLAM 建圖、自主導航。

**啟動步驟：**

終端 1 - rosbridge (WebSocket 通訊)：
```bash
source install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

終端 2 - API Server (REST API)：
```bash
source install/setup.bash
ros2 run robot_api_server api_server
```

終端 3 - 前端開發伺服器：
```bash
cd src/robot_web_frontend
npm run dev
```

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

**方法一：使用網頁前端**
1. 開啟網頁前端 (參考 4.2)
2. 點擊 "SLAM Mapping" 頁面
3. 點擊 "Start Mapping" 開始建圖
4. 使用虛擬搖桿控制機器人移動
5. 輸入地圖名稱，點擊 "Save Map" 儲存

**方法二：使用命令列**

1. **啟動建圖模式** (終端 1) - 包含馬達、LiDAR、IMU、SLAM:
   ```bash
   ros2 launch nav2 mapping.launch.py
   ```

2. **啟動 LiDAR 馬達**:
   ```bash
   ros2 service call /start_motor std_srvs/srv/Empty
   ```

3. **啟動鍵盤控制** (終端 2):
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```
   > 啟動後按 `z` 降低速度至約 0.05 m/s（建議建圖時用低速）

4. **啟動 RViz2 視覺化** (終端 3):
   ```bash
   rviz2
   ```
   在 RViz2 中加入顯示：Add → By topic → `/map` → Map

5. **儲存地圖**:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f src/map/map
   ```

### 4.5. 自主導航

**方法一：使用網頁前端**
1. 確保已有儲存的地圖
2. 啟動自主導航 (終端):
   ```bash
   ros2 launch nav2 autonomous_navigation.launch.py
   ```
3. 開啟網頁前端，點擊 "Navigation" 頁面
4. 點擊地圖選擇目標位置
5. 點擊 "Navigate to Goal" 開始導航

**方法二：使用 RViz2**

1. **啟動機器人核心** (終端 1):
   ```bash
   ros2 launch motor_control full_system.launch.py
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

## 5. 開發者與除錯

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

| 端點 | 方法 | 說明 |
|------|------|------|
| `/robot/start` | POST | 啟動機器人核心 |
| `/robot/stop` | POST | 停止機器人核心 |
| `/robot/start_lidar` | POST | 啟動 LiDAR 馬達 |
| `/robot/status` | GET | 取得機器人狀態 |
| `/slam/start` | POST | 開始 SLAM 建圖 |
| `/slam/stop` | POST | 停止 SLAM 建圖 |
| `/slam/save_map` | POST | 儲存地圖 |
| `/slam/status` | GET | 取得 SLAM 狀態 |
| `/navigate_to_goal` | POST | 發送導航目標 |
| `/navigation/cancel` | POST | 取消導航 |
| `/navigation/status` | GET | 取得導航狀態 |
