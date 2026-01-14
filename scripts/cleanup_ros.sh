#!/bin/bash
# 清理所有 ROS2 導航/建圖相關進程
# 用於切換模式前確保乾淨的狀態

echo "正在清理 ROS2 進程..."

# 停止所有 Nav2 相關進程
pkill -f "nav2_" 2>/dev/null
pkill -f "slam_toolbox" 2>/dev/null
pkill -f "autonomous_navigation.launch" 2>/dev/null
pkill -f "mapping.launch" 2>/dev/null

# 停止 API server (會自動重啟)
pkill -f "robot_api_server/api_server" 2>/dev/null

# 停止 map_relay
pkill -f "map_relay" 2>/dev/null

# 停止可能的重複 basic_navigator
pkill -f "basic_navigator" 2>/dev/null

# 等待進程結束
sleep 2

# 清理殘留的 ros2 topic/service 命令
pkill -f "ros2 topic" 2>/dev/null
pkill -f "ros2 service" 2>/dev/null

echo "清理完成"

# 顯示剩餘的 ROS2 節點
echo ""
echo "剩餘節點:"
source /opt/ros/humble/setup.bash
ros2 node list 2>/dev/null
