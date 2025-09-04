#!/bin/bash

# ROS2 自動建置腳本
# 自動執行清理、建置和環境設置

echo "開始 ROS2 workspace 建置流程..."

# 切換到 base_dev 目錄
echo "切換到 ~/base_dev 目錄..."
cd ~/base_dev || {
    echo "錯誤: 無法切換到 ~/base_dev 目錄"
    exit 1
}

# 清理舊的建置文件和 Python 快取
echo "清理舊的建置文件..."
rm -rf build install log

echo "清理 Python 快取文件..."
find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find . -name "*.pyc" -delete 2>/dev/null || true

# 使用 colcon 建置
echo "開始建置..."
colcon build --symlink-install

# 檢查建置是否成功
if [ $? -eq 0 ]; then
    echo "建置成功!"
    
    # 設置環境變量
    echo "設置環境變量..."
    source install/setup.bash
    
    echo "ROS2 workspace 建置完成!"
else
    echo "錯誤: 建置失敗"
    exit 1
fi