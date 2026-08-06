# pico_sensor_hub (ROS 2)

把 Pico 感測器集線板的序列輸出橋接進 ROS 2：
8 通道 HC-SR04 超音波距離 → `sensor_msgs/Range`，
INA226 電源量測 → `std_msgs/Float32`。

韌體端在 `firmware/pico_sensor_hub/`（本 package 要求 **韌體 v0.2.0 以上**，
因為 `$PW` 的欄位數在該版由 5 變 6）。

## 為什麼是獨立 package

沒有併進 `motor_control`，理由有三：

1. **職責不同。** `motor_control` 是差速驅動的控制與里程計，本 package 是
   一塊獨立感測器硬體的介面。混在一起的話，動超音波要重建整個馬達 package。
2. **失效域不同。** Pico 斷線只該讓感測資料停止；不該有任何機會影響馬達控制的
   啟動路徑或依賴。
3. **與韌體目錄同名對應。** `firmware/pico_sensor_hub` ↔ `src/pico_sensor_hub`，
   協定改動時兩邊要一起看，同名最省事。

## Topics

| Topic | 型別 | 說明 |
|---|---|---|
| `pico/ultrasonic/<位置>` | `sensor_msgs/Range` | 8 個，位置見下表 |
| `pico/voltage` | `std_msgs/Float32` | 電池母線電壓 (V) |
| `pico/current` | `std_msgs/Float32` | 電流 (A)，**放電為正、充電為負** |
| `pico/power` | `std_msgs/Float32` | 功率 (W) |

8 個超音波位置（順序即韌體 `$US` 的 `d1..d8`）：
`front_left`, `front_right`, `right_front`, `right_rear`,
`rear_right`, `rear_left`, `left_rear`, `left_front`。
`frame_id` 為 `ultrasonic_` + 位置，例如 `ultrasonic_front_left`。

`pico/voltage` 與 `motor/voltage`（`motor_control`）量的是**同一顆 7S 電池母線**，
但來自兩個互相獨立的量測鏈，可以交叉比對。本 package **只發布數據，不做任何
電壓門檻判斷**——22.4V 警告 / 21.7V 停機屬於 ROS 層後續工單（`docs/power_design.md` §2）。

### 🔴 無效值：絕不是 0

| 韌體值 | 意義 | `Range.range` |
|---|---|---|
| `-1` | 逾時無回波（含感測器未接線） | `+Inf` |
| `-2` | 通道故障（ECHO 卡高電位） | `NaN` |

`0` 在 `Range` 語意裡是「障礙物貼著感測器」，會讓下游避障誤觸發急停。
`-1` 給 `+Inf` 是因為量測有效但範圍內沒東西（等同無限遠）；
`-2` 是量測本身壞掉，只能給 `NaN`。**下游務必兩者都處理**——
`math.isinf` 與 `math.isnan` 都要檢查，或直接用 `math.isfinite`。

電源側的無效（`$PW` 的 `ok=0`，代表 INA226 讀取失敗）採**整組不發布**：
`Float32` 沒有 header 也沒有狀態欄，發 `NaN` 的話下游只要漏一個 `isnan` 檢查，
`NaN` 就會進到門檻比較裡（`NaN` 的比較永遠為 False，會安靜地讓低電壓保護失效）。
不發布則讓下游用「topic 多久沒更新」判斷，這是既有且不會被忽略的機制。
無效原因會在節點 log 出來（`$PW ok=0` 警告 + `$ST` flags）。

## 執行

前置：裝好 udev rule（`firmware/pico_sensor_hub/README.md` §3.3），
讓序列埠固定在 `/dev/pico_sensor_hub`。沒有它的話 Pico 每次重新列舉
都可能換到別的 `ttyACM` 節點，節點會連不回來。

```bash
colcon build --packages-select pico_sensor_hub
source install/setup.bash

# 真實硬體
ros2 launch pico_sensor_hub pico_sensor_hub.launch.py
# 換序列埠
ros2 launch pico_sensor_hub pico_sensor_hub.launch.py port:=/dev/ttyACM0

# 不接硬體
ros2 launch pico_sensor_hub mock_pico_sensor_hub.launch.py
```

查看資料：

```bash
ros2 topic echo /pico/ultrasonic/front_left
ros2 topic hz /pico/ultrasonic/front_left     # 應為 10 Hz
ros2 topic echo /pico/voltage
```

## 參數

真節點（預設值見 `config/pico_sensor_hub.yaml`）：

| 參數 | 預設 | 說明 |
|---|---|---|
| `port` | `/dev/pico_sensor_hub` | 序列埠 |
| `baudrate` | 115200 | CDC 虛擬埠，實際不影響傳輸 |
| `frame_prefix` | `ultrasonic_` | `frame_id` 前綴 |
| `field_of_view` | 0.26 | rad，HC-SR04 約 15° |
| `min_range` / `max_range` | 0.02 / 4.0 | m |
| `reconnect_period` | 1.0 | 斷線後重試間隔（秒） |
| `data_timeout` | 3.0 | 連著但這麼久沒有效資料就重連（秒） |

mock 節點另有 `publish_frequency`(10.0)、`timeout_channels`([3])、
`fault_channels`([6])、`battery_voltage`(25.2)、`power_invalid`(false)。
`timeout_channels` / `fault_channels` 讓下游可以隨時把某個通道切成
`+Inf` 或 `NaN`，測自己的無效值處理。

## 韌體斷線時的行為

序列讀取跑在背景執行緒，任何 I/O 例外只讓連線重來，節點不會死：

- 開埠失敗 / 讀取例外 → log 警告，每 `reconnect_period` 秒重試
- 連著但 `data_timeout` 秒沒有有效資料 → 主動斷開重連
  （USB 被拔掉時核心可能只是讓 read 一直回空而不拋例外，
  沒有這層檢查節點會安靜地永遠不再發布，比 crash 更難查）
- 校驗失敗 → 計數並 throttled 警告，丟掉該行後繼續
- 欄位數不符（例如韌體還是 v0.1.x）→ log error 提示版本不符，不發布錯誤資料

## 測試

```bash
colcon test --packages-select pico_sensor_hub
colcon test-result --verbose --test-result-base build/pico_sensor_hub
```

`test/test_protocol.py` 不需要硬體也不需要 rclpy，涵蓋校驗、各訊息解析，
以及**無效值映射**這條最高風險路徑（哨兵值絕不可變成 0）。

## 目錄結構

```
src/pico_sensor_hub/
  pico_sensor_hub/
    protocol.py            ← 純解析：XOR 校驗、$US/$PW/$ST/$ID、無效值映射
    publishers.py          ← 真節點與 mock 共用的發布層（保證兩者 API 一致）
    pico_sensor_node.py    ← 真節點：序列讀取執行緒、重連、健康檢查
    mock_pico_sensor.py    ← mock 節點：不接硬體
  launch/  config/  test/
```
