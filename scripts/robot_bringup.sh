#!/bin/bash
# 機器人核心啟動腳本

# 等待系統完全啟動
sleep 10

# Source ROS2 環境
source /opt/ros/humble/setup.bash
source /home/jetson/base_dev/install/setup.bash

# 設定環境變數
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 啟動機器人核心
exec ros2 launch motor_control bringup.launch.py
