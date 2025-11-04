# ROS2 差速驅動機器人控制專案

這是一個為差速驅動移動機器人設計的完整 ROS2 專案，整合了感測器、馬達控制以及導航功能。

---

## 目錄
1. [硬體設置](#1-硬體設置)
2. [環境依賴](#2-環境依賴)
3. [安裝與建置](#3-安裝與建置)
4. [Docker 使用說明](#4-docker-使用說明)
5. [使用說明](#5-使用說明)
   - [啟動機器人核心](#51-啟動機器人核心)
   - [手動鍵盤控制](#52-手動鍵盤控制)
   - [SLAM 建圖](#53-slam-建圖)
   - [自主導航](#54-自主導航)
6. [開發者與除錯](#6-開發者與除錯)

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

## 4. Docker 使用說明

本專案支援使用 Docker 進行部署，簡化環境設定的複雜性。

### 4.1. 建置 Docker 映像

在專案根目錄下，執行以下指令來建置 Docker 映像：

```bash
docker build -t ros2_robot .
```

### 4.2. 啟動 Docker 容器

#### 標準模式（僅掛載序列埠）

為了讓容器內的 ROS2 節點能與硬體（LIDAR、ESP32）通訊，您需要在啟動容器時將序列埠設備掛載進去。

```bash
docker run -it --rm \
  --device=/dev/ttyUSB0:/dev/ttyUSB0 \
  --device=/dev/ttyTHS1:/dev/ttyTHS1 \
  ros2_robot
```

**注意**:
- `--rm`: 容器停止後自動刪除。
- `--device`: 將主機的硬體設備掛載到容器中。請確保您的 LIDAR 和 ESP32 的確是連接到 `/dev/ttyUSB0` 和 `/dev/ttyTHS1`。

成功啟動後，您將會進入容器的 shell 環境，並且 ROS2 的環境已經為您設定完成。接下來的操作（如啟動 launch 檔案）都在此容器內執行。

#### 高權限模式（完全硬體訪問）

如果您需要容器擁有與主機完全相同的硬體訪問權限（包括所有 I/O 埠、GPIO、網路等），可以使用 `--privileged` 和 `--net=host` 旗標。

```bash
docker run -it --rm --privileged --net=host ros2_robot
```

**警告：**
- `--privileged`: 此旗標會給予容器內 root 用戶真正的 root 權限，移除了大部分的 Docker 安全隔離機制。請僅在您完全信任映像內容時使用。
- `--net=host`: 容器將共享主機的網路堆疊，效能較好，但可能會與主機上的服務產生埠號衝突。

## 5. 使用說明

每次開啟新的終端機時，請記得先 source 工作區環境：

```bash
source install/setup.bash
```
**Docker 注意**: 如果您是透過 Docker 容器執行，則無需執行上述 `source` 指令，因為環境已經設定好。

### 5.1. 啟動機器人核心

此指令會啟動所有基礎節點，包括馬達控制器、LIDAR、IMU 以及 EKF 狀態估算。這是執行任何操作前的基礎。

```bash
ros2 launch motor_control full_system.launch.py
```

### 5.2. 手動鍵盤控制

在**另一個**終端機中（或透過 `docker exec` 開啟一個新的 shell），啟動鍵盤控制節點。您將可以在此終端機中透過鍵盤控制機器人移動。

```bash
ros2 launch motor_control keyboard_control.launch.py
```

### 5.3. SLAM 建圖

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
    ros2 run nav2_map_server map_saver_cli -f /ros2_ws/src/map/map
    ```
    建議將生成的地圖檔案儲存到 `src/map/` 目錄下，以供導航使用。

### 5.4. 自主導航

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

## 6. 開發者與除錯

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