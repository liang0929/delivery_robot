#!/bin/bash
# 安裝開機自動啟動服務

set -e

echo "=== 安裝機器人自動啟動服務 ==="

# 建置前端 production 版本
echo "1. 建置前端 production 版本..."
cd /home/jetson/base_dev/src/robot_web_frontend
npm run build

# 安裝 serve (用於提供靜態檔案)
echo "2. 安裝 serve..."
npm install -g serve

# 複製 systemd 服務檔
echo "3. 安裝 systemd 服務..."
sudo cp /home/jetson/base_dev/scripts/robot-core.service /etc/systemd/system/
sudo cp /home/jetson/base_dev/scripts/robot-web.service /etc/systemd/system/

# 重新載入 systemd
sudo systemctl daemon-reload

# 啟用服務（開機自動啟動）
echo "4. 啟用開機自動啟動..."
sudo systemctl enable robot-core.service
sudo systemctl enable robot-web.service

echo ""
echo "=== 安裝完成 ==="
echo ""
echo "服務管理指令："
echo "  啟動服務:   sudo systemctl start robot-core robot-web"
echo "  停止服務:   sudo systemctl stop robot-core robot-web"
echo "  查看狀態:   sudo systemctl status robot-core robot-web"
echo "  查看日誌:   journalctl -u robot-core -f"
echo "             journalctl -u robot-web -f"
echo ""
echo "重新開機後，服務會自動啟動"
echo "網頁控制: http://<機器人IP>:3000"
