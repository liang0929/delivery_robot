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
ros2 launch nav2 simple_navigation.launch.py
```
### 啟動 RViz2
```bash
rviz2
```
在 RViz2 中：
1. 使用 "2D Pose Estimate" 設定機器人初始位置
2. 使用 "Nav2 Goal" 設定目標位置
3. 機器人會自主規劃路徑並導航到目標
