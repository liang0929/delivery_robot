# Robot Development Makefile
# 使用方式: make <target>

.PHONY: help start stop restart status logs build install uninstall dev core web sim sim-core

# 預設顯示幫助
help:
	@echo "=== 機器人開發快捷命令 ==="
	@echo ""
	@echo "服務管理:"
	@echo "  make start      - 啟動所有服務"
	@echo "  make stop       - 停止所有服務"
	@echo "  make restart    - 重啟所有服務 (修改程式碼後使用)"
	@echo "  make status     - 查看服務狀態"
	@echo "  make logs       - 查看即時日誌"
	@echo ""
	@echo "個別服務:"
	@echo "  make core       - 只重啟機器人核心"
	@echo "  make web        - 只重啟前端服務"
	@echo ""
	@echo "開發模式:"
	@echo "  make dev        - 開發模式 (前景執行，方便看日誌)"
	@echo "  make dev-core   - 只啟動核心 (前景)"
	@echo "  make dev-web    - 只啟動前端 (前景)"
	@echo ""
	@echo "模擬模式 (不需要實際硬體):"
	@echo "  make sim        - 模擬模式 + Web 服務"
	@echo "  make sim-core   - 只啟動模擬核心"
	@echo "  make sim-room   - 模擬方形房間場景"
	@echo "  make sim-corridor - 模擬走廊場景"
	@echo ""
	@echo "建置與安裝:"
	@echo "  make build      - 建置 ROS2 套件和前端"
	@echo "  make install    - 安裝 systemd 服務 (開機自動啟動)"
	@echo "  make uninstall  - 移除 systemd 服務"
	@echo ""

# === 服務管理 (使用 systemd) ===

start:
	@echo "啟動服務..."
	sudo systemctl start robot-core robot-web
	@sleep 2
	@make status

stop:
	@echo "停止服務..."
	sudo systemctl stop robot-web robot-core

restart:
	@echo "重啟服務..."
	sudo systemctl restart robot-core
	@sleep 3
	sudo systemctl restart robot-web
	@echo "重啟完成！"
	@make status

status:
	@echo "=== 服務狀態 ==="
	@systemctl is-active robot-core >/dev/null 2>&1 && echo "robot-core: ✅ 運行中" || echo "robot-core: ❌ 未運行"
	@systemctl is-active robot-web >/dev/null 2>&1 && echo "robot-web:  ✅ 運行中" || echo "robot-web:  ❌ 未運行"
	@echo ""
	@echo "網頁控制: http://$$(hostname -I | awk '{print $$1}'):3000"

logs:
	@echo "按 Ctrl+C 退出日誌..."
	journalctl -u robot-core -u robot-web -f

# === 個別服務重啟 ===

core:
	@echo "重啟機器人核心..."
	sudo systemctl restart robot-core
	@echo "完成！"

web:
	@echo "重啟前端服務..."
	sudo systemctl restart robot-web
	@echo "完成！"

# === 開發模式 (前景執行) ===

dev: dev-stop
	@echo "=== 開發模式 (按 Ctrl+C 停止) ==="
	@echo "啟動核心服務..."
	@bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && \
		ros2 launch motor_control bringup.launch.py &'
	@sleep 5
	@echo "啟動前端..."
	@cd src/robot_web_frontend && npm run dev -- --host

dev-core:
	@echo "=== 開發模式: 機器人核心 ==="
	@bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && \
		ros2 launch motor_control bringup.launch.py'

dev-web:
	@echo "=== 開發模式: 前端服務 ==="
	@cd src/robot_web_frontend && npm run dev -- --host

dev-stop:
	@echo "停止開發模式進程..."
	@-pkill -f "ros2 launch motor_control" 2>/dev/null || true
	@-pkill -f "npm run dev" 2>/dev/null || true
	@-pkill -f "vite" 2>/dev/null || true

# === 模擬模式 (不需要實際硬體) ===

sim: dev-stop
	@echo "=== 模擬模式 + Web 服務 (按 Ctrl+C 停止) ==="
	@echo "場景: room (5m x 5m 方形房間)"
	@bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && \
		ros2 launch motor_control bringup.launch.py simulation:=true &'
	@sleep 5
	@echo "啟動前端..."
	@cd src/robot_web_frontend && npm run dev -- --host

sim-core:
	@echo "=== 模擬模式: 機器人核心 ==="
	@bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && \
		ros2 launch motor_control bringup.launch.py simulation:=true enable_web:=false'

sim-room:
	@echo "=== 模擬模式: 方形房間場景 ==="
	@bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && \
		ros2 launch motor_control bringup.launch.py simulation:=true sim_scene:=room'

sim-corridor:
	@echo "=== 模擬模式: 走廊場景 ==="
	@bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && \
		ros2 launch motor_control bringup.launch.py simulation:=true sim_scene:=corridor'

# === 建置 ===

build:
	@echo "=== 建置 ROS2 套件 ==="
	@bash -c 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'
	@echo ""
	@echo "=== 建置前端 ==="
	@cd src/robot_web_frontend && npm run build
	@echo ""
	@echo "建置完成！"

build-ros:
	@bash -c 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'

build-web:
	@cd src/robot_web_frontend && npm run build

# === 安裝/移除服務 ===

install:
	@echo "安裝 systemd 服務..."
	@cd src/robot_web_frontend && npm run build
	@sudo cp scripts/robot-core.service /etc/systemd/system/
	@sudo cp scripts/robot-web.service /etc/systemd/system/
	@sudo systemctl daemon-reload
	@sudo systemctl enable robot-core robot-web
	@echo ""
	@echo "✅ 安裝完成！服務將在開機時自動啟動"
	@echo "   使用 'make start' 立即啟動"

uninstall:
	@echo "移除 systemd 服務..."
	@./scripts/uninstall_services.sh
	@echo "✅ 移除完成"
