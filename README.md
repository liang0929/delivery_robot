
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

- **LIDAR**: 連接到 `/dev/ttyUSB0`
- **ESP32 馬達控制器**: 連接到 `/dev/ttyTHS1` (Jetson 預設序列埠)

**注意**: 上述序列埠為目前程式中的預設值。如果您的設備連接到不同的序列埠，請修改對應的啟動或設定檔。

### ESP32 韌體

本專案的馬達控制核心運行在 ESP32 上。您需要將 `src/motorControl/motor_control.ino` 的程式碼燒錄到您的 ESP32 開發板。

**韌體依賴的函式庫**:
- `ArduinoJson`
- `ModbusMaster`

請在上傳前，透過 Arduino IDE 的程式庫管理員安裝以上兩個函式庫。

## 2. 環境依賴

- **ROS2 Humble Hawksbill**
- **robot_localization**: ROS2 的標準 EKF 狀態估算套件。
  ```bash
  sudo apt-get update
  sudo apt-get install ros-humble-robot-localization
  ```

## 3. 安裝與建置

專案根目錄下提供了一個自動化腳本，會自動清理舊的建置緩存並使用 `colcon` 進行編譯。

```bash
./build_ros2.sh
```

## 4. 使用說明

每次開啟新的終端機時，請記得先 source 工作區環境：

```bash
source install/setup.bash
```

### 4.1. 啟動機器人核心

此指令會啟動所有基礎節點，包括馬達控制器、LIDAR、IMU 以及 EKF 狀態估算。這是執行任何操作前的基礎。

```bash
ros2 launch motor_control full_system.launch.py
```

### 4.2. 手動鍵盤控制

在**另一個**終端機中，啟動鍵盤控制節點。您將可以在此終端機中透過鍵盤控制機器人移動。

```bash
ros2 launch motor_control keyboard_control.launch.py
```

### 4.3. SLAM 建圖

1.  **啟動建圖模式**:
    此模式會啟動 SLAM 相關節點。
    ```bash
    ros2 launch nav2 mapping.launch.py
    ```

2.  **啟動鍵盤控制**:
    在另一個終端機中啟動鍵盤控制，手動遙控機器人探索環境。
    ```bash
    ros2 launch motor_control keyboard_control.launch.py
    ```

3.  **啟動 RViz2 視覺化**:
    在另一個終端機中啟動 RViz2，觀察即時的建圖過程。
    ```bash
    rviz2
    ```

4.  **儲存地圖**:
    當您對地圖感到滿意時，執行以下指令儲存地圖。地圖將被儲存為 `map.pgm` 和 `map.yaml`。
    ```bash
    ros2 run nav2_map_server map_saver_cli -f ~/map
    ```
    建議將生成的地圖檔案複製到 `src/map/` 目錄下，以供導航使用。

### 4.4. 自主導航

1.  **啟動導航模式**:
    此指令會載入已儲存的地圖，並啟動 Nav2 導航堆疊。
    ```bash
    ros2 launch nav2 autonomous_navigation.launch.py
    ```

2.  **啟動 RViz2**:
    在另一個終端機中啟動 RViz2。
    ```bash
    rviz2
    ```

3.  **在 RViz2 中操作**:
    - 使用工具列上的 **"2D Pose Estimate"** 按鈕，在地圖上標示出機器人的初始位置與方向。
    - 使用工具列上的 **"Nav2 Goal"** 按鈕，在地圖上設定一個目標點。
    - 機器人將會自動規劃路徑並駛向目標。

## 5. 開發者與除錯

以下是一些在開發與除錯時常用的監控指令：

```bash
# 監控里程計數據 (EKF融合後)
ros2 topic echo /odom

# 監控來自 ESP32 的原始里程計數據
ros2 topic echo /odom_raw

# 監控發送給馬達的速度指令
ros2 topic echo /cmd_vel

# 監控 LIDAR 掃描數據
ros2 topic echo /scan

# 監控 IMU 數據
ros2 topic echo /imu/data

# 產生 TF 樹的 PDF 檔案
ros2 run tf2_tools view_frames
```
