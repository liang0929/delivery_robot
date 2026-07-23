#!/bin/bash
# 機器人核心啟動腳本

# 輪詢等待硬體裝置節點 (udev) 出現，取代固定 sleep
# 最多等待 30 秒，逾時警告後仍繼續（launch 內有各自的錯誤處理）
WAIT_DEVICES="/dev/motor /dev/lidar"
DEADLINE=$((SECONDS + 30))
while [ $SECONDS -lt $DEADLINE ]; do
    ALL_READY=true
    for dev in $WAIT_DEVICES; do
        [ -e "$dev" ] || ALL_READY=false
    done
    $ALL_READY && break
    sleep 1
done
for dev in $WAIT_DEVICES; do
    if [ ! -e "$dev" ]; then
        echo "[WARN] 裝置 $dev 未出現（等待 30 秒逾時），仍繼續啟動" >&2
    fi
done

# Source ROS2 環境
source /opt/ros/humble/setup.bash
source "$HOME/base_dev/install/setup.bash"

# 設定環境變數
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# 硬體選項：預設關閉 IMU 與實體急停，因為目前皆未接線。
#   - IMU 未接：BNO055 節點會在 init 拋例外崩潰
#   - 急停未接：GPIO 腳位浮接會被判定為「按下」，馬達永久拒收 cmd_vel
# 接上硬體後，於 .env 設 ENABLE_IMU=true / ENABLE_ESTOP=true 即可，不必改此腳本。
ENABLE_IMU="${ENABLE_IMU:-false}"
ENABLE_ESTOP="${ENABLE_ESTOP:-false}"

# 啟動機器人核心
exec ros2 launch motor_control bringup.launch.py \
    enable_imu:="${ENABLE_IMU}" \
    enable_estop:="${ENABLE_ESTOP}"
