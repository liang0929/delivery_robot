#!/bin/bash
# pico_sensor_hub 啟動腳本（獨立於 robot-core 的 systemd unit）
#
# 刻意不併進 robot_bringup.sh：Pico 是 USB CDC 裝置，插拔與韌體更新只該讓
# 感測資料停止，不該有機會連帶重啟馬達控制鏈（見 src/pico_sensor_hub/README.md
# 「為什麼是獨立 package」第 2 點：失效域不同）。

# Source ROS2 環境
source /opt/ros/humble/setup.bash
source "$HOME/base_dev/install/setup.bash"

# 設定環境變數（必須與 robot_bringup.sh 同一組，兩個服務要落在同一個 DDS
# 網域，battery_guard 才訂得到本服務發的 /pico/voltage）
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/base_dev/scripts/fastdds_udp_only.xml"

# 這裡刻意不做 `rm -f /dev/shm/fastrtps_*`。robot_bringup.sh 清得掉，是因為它
# 啟動時全系統沒有其他 DDS 進程；本服務隨時可能在 robot-core 執行中被單獨重啟，
# 清掉共用的 SHM 區段會打死正在跑的節點。何況 profile 已停用 SHM transport。

# 序列埠：udev rule 固定的節點（firmware/pico_sensor_hub/README.md §3.3）。
# 沒裝 rule 時在 .env 設 PICO_PORT=/dev/ttyACM0 覆寫，不必改此腳本。
PICO_PORT="${PICO_PORT:-/dev/pico_sensor_hub}"

# 不等待裝置出現（robot_bringup.sh 有那段輪詢是因為 lidar/motor 節點開不到埠就
# 直接死）。pico_sensor_node 內建重連：開埠失敗只 log 警告，每 reconnect_period
# 秒重試，Pico 晚接或熱插拔都會自己接回來。

exec ros2 launch pico_sensor_hub pico_sensor_hub.launch.py port:="${PICO_PORT}"
