#!/bin/bash
# Web 前端啟動腳本

# 輪詢等待 rosbridge (port 9090) 就緒，取代固定 sleep
# 最多等待 30 秒，逾時警告後仍繼續（前端會自行重連 rosbridge）
DEADLINE=$((SECONDS + 30))
while [ $SECONDS -lt $DEADLINE ]; do
    if (echo > /dev/tcp/127.0.0.1/9090) 2>/dev/null; then
        echo "rosbridge 已就緒"
        break
    fi
    sleep 1
done
if ! (echo > /dev/tcp/127.0.0.1/9090) 2>/dev/null; then
    echo "[WARN] rosbridge (9090) 未就緒（等待 30 秒逾時），仍繼續啟動前端" >&2
fi

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
