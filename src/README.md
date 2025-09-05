motor_control -> motor control and get motor info. publish odomt
```
主要輸入 / 輸出（contract）
輸入：
ROS2 topic: cmd_vel (geometry_msgs/Twist)
UART: 來自 ESP32 的 JSON 行為訊息（string line）
輸出：
UART: 發送 JSON 或二進位馬達命令到 ESP32
ROS2 topic: odom (nav_msgs/Odometry)
TF: transform odom -> base_footprint
錯誤模式：
串口不可用 / 斷線
非 JSON 的串列資料（會當作除錯字串）
JSON 欄位缺失時使用預設值
資料流與重要函式
初始化 (__init__):

宣告與讀取參數（uart_port, baudrate, 車輪尺寸等）。
建立 serial 連線 (connect_serial)。
建立 ROS2 subscription (cmd_vel) 與 publisher (odom)、TF broadcaster。
啟動背景線程 receive_data_thread 負責非同步讀 serial。
建立定時器 safety_timer 每 0.1s 呼叫 safety_check。
接收速度命令 (cmd_vel_callback):

限制線速度與角速度到參數上限。
呼叫 diff_drive_kinematics 計算左右輪速度（m/s）。
呼叫 send_motor_command 將命令送到 ESP32（預設用 JSON 行，各值四捨五入到小數點 3 位）。
運動學 (diff_drive_kinematics):

left = linear_x - (angular_z * wheel_separation / 2)
right = linear_x + (angular_z * wheel_separation / 2)
發送命令:

send_motor_command: 建 JSON 包含 type='motor_cmd', left_vel, right_vel, timestamp，並透過 serial.write 發送（換行結尾）。
send_binary_command: （可選）提供一個二進位封包範例，包含起始位元、命令類型、轉換為整數的速度、checksum。
接收與處理來自 ESP32 的資料:

receive_data_thread: 持續檢查 serial_conn.in_waiting，讀一行並呼叫 process_received_data。
process_received_data: 嘗試 json.loads；若 type == 'odometry' -> update_odometry；type == 'sensor' -> process_sensor_data；type == 'status' -> log。
非 JSON 字串若以 'ESP32:' 開頭則紀錄為 info。
里程計發布:

update_odometry 更新 self.odom_data 後呼叫 publish_odometry 與 publish_tf。
publish_odometry 建 Odometry 訊息並 publish 到 odom topic（包含簡化協方差）。
publish_tf 建 TransformStamped，使用 odom_data 的 x,y,theta 產生 z,w 四元數並發送。
安全與結束:

safety_check：若超過 1 秒沒收到 cmd，呼叫 send_motor_command(0,0)。
destroy_node：停止接收執行緒、發送停止命令、關閉 serial。
訊息/封包格式（預期）
發送到 ESP32（JSON 範例）： {"type":"motor_cmd","left_vel":0.123,"right_vel":0.123,"timestamp":167...}\n
從 ESP32 收到（JSON 範例）：
odometry: {"type":"odometry","x":..., "y":..., "theta":..., "vx":..., "vy":..., "vth":...}
sensor: {"type":"sensor","sensor_type":"battery","voltage":12.3}
status: {"type":"status","message":"ok"}
```
ros-imu-bno055 -> imu bno055 driver
slliar_ros2 -> A2M12 lidar drive
ros2_teleop_keyboard -> keybord control