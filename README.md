### 編譯自動化腳本
此指令會自動刪除過去編譯的緩存並編譯
```bash
./build_ros2.sh
```
———————————————————————————————————————————
### 基礎啟動
```bash
# Launch complete system with all sensors
ros2 launch motor_control full_system.launch.py

# Launch keyboard control for manual driving
ros2 launch motor_control keyboard_control.launch.py

# Launch ESP32 motor controller only
ros2 launch motor_control esp32_motor_controller.launch.py
```
———————————————————————————————————————————
### 啟動建圖模式
```bash
ros2 launch nav2 mapping.launch.py
```
使用鍵盤控制機器人移動建圖
鍵盤控制會在 xterm 窗口中顯示操作說明
### 在另一個終端啟動 RViz2 觀察建圖過程
```bash
rviz2
```

### 建圖完成後儲存地圖
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```
將生成的 map.pgm 和 map.yaml 複製到 src/map/ 目錄

———————————————————————————————————————————

### 啟動導航模式
```bash
ros2 launch nav2 autonomous_navigation.launch.py
```
### 啟動 RViz2
```bash
rviz2
```
在 RViz2 中：
1. 使用 "2D Pose Estimate" 設定機器人初始位置
2. 使用 "Nav2 Goal" 設定目標位置
3. 機器人會自主規劃路徑並導航到目標

———————————————————————————————————————————
### 監控指令
```bash
# Monitor odometry data
ros2 topic echo /odom
ros2 topic echo /odom_raw

# Monitor cmd_vel commands
ros2 topic echo /cmd_vel

# Check TF tree
ros2 run tf2_tools view_frames

# Monitor LIDAR data
ros2 topic echo /scan

# Test ESP32 connection
python3 src/motor_control/test/test_connect.py
```