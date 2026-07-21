#!/bin/bash
# 清理所有 ROS2 導航/建圖相關進程
# 用於切換模式前確保乾淨的狀態

echo "正在清理 ROS2 進程..."

# 停止所有 Nav2 相關進程
# pattern 錨定完整路徑/指令，避免誤殺（如編輯器開著 nav2_params.yaml、
# 使用者手動執行的 ros2 CLI 等）
pkill -f "ros2 launch nav2 autonomous_navigation.launch.py" 2>/dev/null
pkill -f "ros2 launch nav2 mapping.launch.py" 2>/dev/null
pkill -f "/lib/nav2_" 2>/dev/null          # nav2 節點二進位 (/opt/ros/*/lib/nav2_*/...)
pkill -f "/lib/slam_toolbox/" 2>/dev/null  # slam_toolbox 節點二進位

# 停止 API server（bringup 的 ExecuteProcess 有 respawn=True 會自動重啟）
pkill -f "robot_api_server/api_server" 2>/dev/null

# 停止 map_relay
# [警告] 導航/建圖模式下 map_relay 由對應 launch 啟動且無 respawn，
# 在導航運行中執行本腳本會使前端地圖與存圖功能失效，需重啟該模式
pkill -f "/lib/motor_control/map_relay" 2>/dev/null

# 等待進程結束
sleep 2

echo "清理完成"

# 顯示剩餘的 ROS2 節點
echo ""
echo "剩餘節點:"
source /opt/ros/humble/setup.bash
ros2 node list 2>/dev/null
