# dock_pose_bridge (ROS 2)

把 AprilTag 的 TF 轉成 `opennav_docking` 唯一認得的外部偵測輸入：

```
apriltag_ros ──/tf──> (dock_camera_optical_frame → tag36h11:0)
                          │  lookup_transform（最新可用）
                          ↓
          detected_dock_pose (geometry_msgs/PoseStamped)
                          ↓
          opennav_docking / SimpleChargingDock
```

## 為什麼需要這一段

偵測端 `apriltag_ros`（christianrauch 版）**只把位姿發到 `/tf`**——它的
`detections` 訊息裡根本沒有 pose 欄位。接收端 `SimpleChargingDock` 則
**只訂閱硬編碼的 `detected_dock_pose`**（`PoseStamped`）。中間這段在 Humble
沒有現成套件可用：官方推薦的 `image_proc/TrackMarkerNode` 只存在於 rolling，
而且它是 OpenCV ArUco 不是 AprilTag。

## 為什麼是獨立 package

理由與 `pico_sensor_hub` 同一套：

1. **職責不同。** 這是一個純粹的訊息型別轉接器，不屬於馬達控制也不屬於感測器集線板。
2. **失效域不同。** 相機或偵測器掛掉只該讓 dock 流程停下，不該有任何機會影響
   既有的啟動路徑。
3. **相依乾淨。** 只依賴 `rclpy` / `tf2_ros` / `geometry_msgs`——刻意**不**依賴
   `opennav_docking` 的任何訊息型別，所以那些包還沒安裝時本 package 一樣
   build 得起來、測得過。

## Topics

| 方向 | Topic | 型別 |
|---|---|---|
| Sub | `/tf`, `/tf_static` | `tf2_msgs/TFMessage` |
| Pub | `detected_dock_pose` | `geometry_msgs/PoseStamped` |

## 🔴 時間戳語意

發出去的 `header.stamp` 一律是**該 transform 自己的時間戳**，不是 `now()`。
下游有兩個地方直接吃這個值：`external_detection_timeout` 的過期判定，
以及 docking server 把 pose 轉到工作 frame 時的 TF 內插點。填 `now()` 會讓
「相機早就看不到 tag」偽裝成永遠新鮮——而且畫面上一切看起來都正常。

這條語意有測試覆蓋（`test/test_pose_bridge.py`、`test/test_dock_pose_bridge_node.py`）。

## tag 不見了怎麼辦

**不發布**，並限流 warn。判定有兩道：

1. `lookup_transform` 失敗（tag 從沒出現過、TF 鏈斷掉）。
2. 查得到、但 transform 已經比 `max_transform_age_sec`（預設 0.5s）舊。

第 2 道不可省：tf buffer 有 10 秒快取，**tag 離開畫面後 lookup 仍會成功並
一直回同一筆舊值**，只靠第 1 道會安靜地餵十秒鐘的過期位姿。

同一筆 transform 也不會重發——查詢頻率高於相機幀率時本來就會重複查到，
重送同一個 stamp 對下游沒有任何新資訊。

## 參數

見 `config/dock_pose_bridge.yaml`（預設值與節點的 `PARAMS` 表一致）。

## 執行

感知套件（`v4l2_camera` / `image_proc` / `apriltag_ros`）與 `opennav_docking`
目前都**尚未安裝**。現在只有 bridge 本身起得來：

```bash
ros2 launch dock_pose_bridge dock_perception.launch.py bridge_only:=true
```

（沒有 tag TF 可查，會每 5 秒限流 warn 一次，這是預期行為。）

感知套件裝好之後起整條鏈：

```bash
ros2 launch dock_pose_bridge dock_perception.launch.py
```

`docking_server` 那段以註解形式放在 launch 檔尾，待 `opennav_docking` 安裝後啟用。

## config/ 裡的其他檔

這三個檔**現在還沒有人讀**，都是待 `opennav_docking` / 感知套件安裝後由 launch 引用：

| 檔案 | 內容 |
|---|---|
| `docking_server.yaml` | DockingServer + SimpleChargingDock 全部參數，非預設值逐項註記理由與出處 |
| `dock_database.yaml` | `home_dock` 的位姿（**目前是佔位值 [0,0,0]，待實測**） |
| `camera_apriltag.yaml` | v4l2_camera（YUYV→mono8、1280×720）＋ apriltag_ros（36h11、120mm、id 0） |

## 測試

```bash
colcon test --packages-select dock_pose_bridge
colcon test-result --test-result-base build/dock_pose_bridge --verbose
```

不需要任何未安裝的套件：TF 是直接寫進節點的 buffer（`TransformListener` 收到
`/tf` 之後做的就是這件事），發布端用假 publisher 攔下來檢查。這樣測試不依賴
DDS，也不會被車上真實的 `/tf` 污染。
