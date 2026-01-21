#!/bin/bash
# Web 前端啟動腳本

# 等待 ROS2 服務啟動
sleep 15

# 載入 nvm 環境
export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"

cd "$HOME/base_dev/src/robot_web_frontend"

# 使用 production build
if [ -d "dist" ]; then
    exec serve -s dist -l 3000
else
    exec npm run dev -- --host
fi
