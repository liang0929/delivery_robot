#!/bin/bash
# 移除開機自動啟動服務

echo "=== 移除機器人自動啟動服務 ==="

# 停止服務
sudo systemctl stop robot-core.service 2>/dev/null || true
sudo systemctl stop robot-web.service 2>/dev/null || true

# 停用服務
sudo systemctl disable robot-core.service 2>/dev/null || true
sudo systemctl disable robot-web.service 2>/dev/null || true

# 移除服務檔
sudo rm -f /etc/systemd/system/robot-core.service
sudo rm -f /etc/systemd/system/robot-web.service

# 重新載入 systemd
sudo systemctl daemon-reload

echo "=== 移除完成 ==="
