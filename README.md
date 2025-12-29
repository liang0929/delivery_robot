# ROS2 差速驅動機器人控制專案

這是一個為差速驅動移動機器人設計的完整 ROS2 專案，整合了感測器、馬達控制以及導航功能。

---

## 目錄
1. [硬體設置](#1-硬體設置)
2. [環境依賴](#2-環境依賴)
3. [安裝與建置](#3-安裝與建置)
4. [使用說明](#4-使用說明)
   - [啟動機器人核心](#41-啟動機器人核心)
   - [手動鍵盤控制](#42-手動鍵盤控制)
   - [SLAM 建圖](#43-slam-建圖)
   - [自主導航](#44-自主導航)
5. [開發者與除錯](#5-開發者與除錯)

---

## 1. 硬體設置

在啟動系統前，請確保硬體已正確連接：

| 設備 | 連接介面 | 說明 |
|------|----------|------|
| 馬達驅動器 (AGV-BLD-2S) | `/dev/ttyUSB0` | USB-RS232 轉接，HS 協議，地址 127 |
| LiDAR (SLAMTEC A2M12) | `/dev/ttyUSB1` | USB 連接，256000 baud |
| IMU (BNO055) | `/dev/i2c-7` | I2C 連接，地址 0x28 |

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
  pip install pyserial
  ```
- **robot_localization**: ROS2 的標準 EKF 狀態估算套件
  ```bash
  sudo apt-get update
  sudo apt-get install ros-humble-robot-localization
  ```
- **teleop_twist_keyboard**: 鍵盤控制
  ```bash
  sudo apt-get install ros-humble-teleop-twist-keyboard
  ```

## 3. 安裝與建置

```bash
# 建置所有套件
colcon build

# 或使用自動化腳本
./build_ros2.sh
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

### 4.2. 手動鍵盤控制

**方法一：使用 launch 檔案 + 另開終端**

終端 1 - 啟動馬達控制器：
```bash
ros2 launch motor_control keyboard_control.launch.py
```

終端 2 - 啟動鍵盤控制：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**方法二：分別啟動**

終端 1：
```bash
ros2 run motor_control hs_motor_controller --ros-args -p device_id:=127
```

終端 2：
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**鍵盤操作說明：**
| 按鍵 | 動作 |
|------|------|
| `i` | 前進 |
| `,` | 後退 |
| `j` | 左轉 |
| `l` | 右轉 |
| `k` | 停止 |
| `q`/`z` | 增加/減少速度 |

### 4.3. SLAM 建圖

1. **啟動建圖模式**:
   ```bash
   ros2 launch nav2 mapping.launch.py
   ```

2. **啟動鍵盤控制** (另一個終端):
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

3. **啟動 RViz2 視覺化** (另一個終端):
   ```bash
   rviz2
   ```

4. **儲存地圖**:
   ```bash
   ros2 run nav2_map_server map_saver_cli -f ./map
   ```

### 4.4. 自主導航

1. **啟動導航模式**:
   ```bash
   ros2 launch nav2 autonomous_navigation.launch.py
   ```

2. **啟動 RViz2** (另一個終端):
   ```bash
   rviz2
   ```

3. **在 RViz2 中操作**:
   - 使用 **"2D Pose Estimate"** 設定機器人初始位置
   - 使用 **"Nav2 Goal"** 設定目標點

## 5. 開發者與除錯

### 常用監控指令

```bash
# 監控里程計數據
ros2 topic echo /odom_raw

# 監控速度指令
ros2 topic echo /cmd_vel

# 監控 LiDAR 掃描數據
ros2 topic echo /scan

# 監控 IMU 數據
ros2 topic echo /data

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
ros2 launch sllidar_ros2 sllidar_a2m12_launch.py serial_port:=/dev/ttyUSB1

# 測試 IMU
ros2 run imu_bno055 bno055_i2c_node --ros-args -p device:=/dev/i2c-7 -p address:=40
```

### ROS2 Topics 一覽

| Topic | 類型 | 說明 |
|-------|------|------|
| `/cmd_vel` | geometry_msgs/Twist | 速度命令輸入 |
| `/odom_raw` | nav_msgs/Odometry | 馬達里程計輸出 |
| `/scan` | sensor_msgs/LaserScan | LiDAR 掃描數據 |
| `/data` | sensor_msgs/Imu | IMU 數據 |
| `/tf` | tf2_msgs/TFMessage | 座標轉換 |
