# 專案待修復問題清單

> 最後更新: 2026-07-22（修正過時檔案路徑）
>
> 注意：歷史項目（20、23、28、29 等）提到的 `modbus_motor_controller.py`
> 已於後續重構移除，相關修復記錄僅供追溯；前端亦於 2026-07 重寫
> （commit `b01dd11`），舊 `services/` 路徑不再存在。

---

## 🔴 新發現 - 高優先級 (High Priority)

### 28. ~~Modbus 控制器缺少 state_lock~~ ✅ 已修復
- **Commit**: `fcae0d2`
- **修復**: 添加 `state_lock` 保護里程計狀態和方向變數

### 29. ~~Modbus 控制器保留未使用的 TF Broadcaster~~ ✅ 已修復
- **Commit**: `fcae0d2`
- **修復**: 移除未使用的 TF Broadcaster 和 publish_tf() 方法

### 30. ~~Nav2 recoveries_server 使用已棄用 API~~ ✅ 已修復
- **Commit**: `666f9cf`
- **修復**: 更新為 behavior_server 和 nav2_behaviors/*

### 31. ~~API Server rclpy 與 FastAPI 多線程問題~~ ✅ 已修復
- **Commit**: `15fdf37`
- **修復**: NavigatorManager 所有公開方法添加 _lock 線程鎖保護

---

## 🟡 新發現 - 中優先級 (Medium Priority)

### 32. ~~mock_motor_controller 缺少 NaN/Inf 驗證~~ ✅ 已修復
- **Commit**: `01047cf`
- **修復**: 在 cmd_vel_callback 添加 NaN/Inf 輸入驗證

### 33. ~~QoS 配置不一致~~ ✅ 已修復
- **Commit**: `01047cf`
- **修復**: 統一 QoS 配置，添加 RELIABLE 和 VOLATILE

### 34. ~~里程計角速度反轉硬編碼~~ ✅ 已修復
- **Commit**: `8413fbb`
- **修復**: 新增 `invert_angular_velocity` 參數（預設 True）

### 35. ~~Nav2 inflation_radius 太小~~ ✅ 已修復
- **Commit**: `8af736b`
- **修復**: 將 local_costmap 和 global_costmap 的 inflation_radius 從 0.05/0.1 調整為 0.35

### 36. ~~EKF 從 odom 取得 vyaw 可能不準確~~ ✅ 已修復
- **Commit**: `d086a56`
- **修復**: 將 odom0_config 的 vyaw 設為 false，角速度完全由 IMU 提供

---

## 🟢 新發現 - 低優先級 (Low Priority)

### 37. ~~重複的協方差矩陣定義~~ ✅ 已修復
- **Commit**: `5e6fa22`
- **修復**: 新增 odom_constants.py 共用模組，三個控制器共用常數

### 38. ~~Costmap 同時使用 obstacle_layer 和 voxel_layer~~ ✅ 已修復
- **Commit**: `f470e50`
- **修復**: 移除 voxel_layer，2D LiDAR 只需 obstacle_layer

### 39. ~~缺少完整的 Type Hints~~ ✅ 已修復
- **Commit**: `8d405fa`
- **修復**: 為三個控制器添加完整的返回類型註解

### 40. ~~魔術數字未定義為常數~~ ✅ 已修復
- **Commit**: `738ef68`
- **修復**: 提取為 `RPM_DEADZONE = 10.0` 類常數

### 41. ~~use_sim_time 參數未傳遞~~ ✅ 已修復
- **Commit**: `b47d866`
- **修復**: 將 use_sim_time 傳遞給 hs_motor_controller、mock_motor_controller 和 ekf_node

---

## 嚴重問題 (Critical)

### 19. ~~nav2_commander rclpy 重複初始化~~ ✅ 已修復
- **Commit**: `9984a2f`
- **修復**: 添加 `_ensure_rclpy_initialized()` 函數，只在需要時初始化和關閉

### 20. ~~Modbus 控制器 wheel_separation 預設值不一致~~ ✅ 已修復
- **Commit**: `30062e1`
- **修復**: 將預設值從 `0.381` 改為 `0.27`，與配置檔一致

### 1. ~~路徑注入漏洞~~ ✅ 已修復
- **Commit**: `4fe836c`
- **修復**: 添加 `validate_map_path()` 函數，驗證路徑在預期目錄內

### 2. ~~State Manager 競態條件~~ ✅ 已修復
- **Commit**: `fb98c31`
- **修復**: 添加 `_verify_process_started()` 驗證進程啟動成功

### 3. ~~串口資源洩漏~~ ✅ 已修復
- **Commit**: `26a4132`
- **修復**: 添加 `_close_serial_safely()` 確保串口完全釋放

---

## 中等問題 (Medium)

### 4. ~~Subprocess 管道死鎖風險~~ ✅ 已修復
- **Commit**: `fb98c31`
- **修復**: 將 `stdout/stderr=PIPE` 改為 `DEVNULL`

### 5. ~~rclpy 初始化競態條件~~ ✅ 已修復
- **Commit**: `98976e6`
- **修復**: 添加全局 `ensure_rclpy_initialized()` 函數和鎖

### 6. ~~參數配置不一致~~ ✅ 已修復
- **Commit**: `c1cd30e`
- **修復**: 統一代碼預設值與配置文件一致

### 7. ~~發布消息時持有鎖~~ ✅ 經檢查無此問題
- **說明**: `publish_odometry()` 已在鎖外調用，代碼正確

### 8. ~~TF 配置不匹配~~ ✅ 已修復
- **Commit**: `af84cc6`
- **修復**:
  - Nav2 costmap 統一使用 `base_footprint`
  - 移除 Modbus Motor Controller 的重複 TF 發布

### 21. ~~里程計更新頻率不匹配~~ ✅ 已修復
- **Commit**: `d4690cb`
- **修復**: 將 Modbus 控制器的 `odom_frequency` 默認值從 20Hz 改為 50Hz，與 HS 控制器一致

### 22. ~~時間差為零未檢查~~ ✅ 已修復
- **Commit**: `9a472c9`
- **修復**: 在兩個控制器的 `update_odometry` 中添加 `if dt <= 0: return` 檢查

### 23. ~~Modbus 控制器缺少參數驗證~~ ✅ 已修復
- **位置**: `src/motor_control/motor_control/modbus_motor_controller.py`
- **修復**: 添加 `_validate_parameters()` 方法，驗證 odom_frequency、wheel_radius、wheel_separation、gear_ratio、min_rpm、max_rpm 等參數

### 24. ~~destroy_node() 異常處理不完整~~ ✅ 已修復
- **Commit**: `7cd6b0b`
- **修復**: 使用 try-finally 確保即使 `client.close()` 拋出異常，`super().destroy_node()` 也會被執行

### 25. ~~Launch 配置讀取無異常處理~~ ✅ 已修復
- **位置**: `src/motor_control/launch/bringup.launch.py`
- **修復**: 添加 try-except 處理 FileNotFoundError 和 YAMLError，提供默認 TF 配置

### 26. ~~方向變數非原子更新~~ ✅ 經檢查無此問題
- **說明**: `state_lock` 已保護 `logical_dir_a/b` 的讀寫操作（寫入在 538-544 行，讀取在 460-465 行）

---

## 輕微問題 (Minor)

### 9. ~~時間處理混用~~ ✅ 已修復
- **Commit**: `edce9d7`
- **修復**: 將安全超時檢查從 `time.time()` 改為 `self.get_clock().now()`

### 10. ~~狀態廣播效率~~ ✅ 經檢查無此問題
- **說明**: `status_broadcast_loop()` 已在 626 行檢查 `connection_count == 0` 並跳過

### 11. ~~QoS 配置不完整~~ ✅ 已修復
- **Commit**: `615ad27`
- **修復**: 顯式設置 `reliability=RELIABLE` 和 `durability=VOLATILE`

### 12. 異常處理過於寬泛
- **位置**: `src/robot_api_server/robot_api_server/main.py` 多處
- **問題**: 大量 `except Exception` 難以診斷問題
- **建議**: 捕獲具體異常類型
- [ ] 待優化

### 13. ~~cmd_vel NaN 輸入未驗證~~ ✅ 已修復
- **Commit**: `36afdd4`
- **修復**: 在 `cmd_vel_callback()` 中添加 `math.isnan()` 和 `math.isinf()` 檢查

### 14. 前端 API 返回未驗證
- **位置**: `src/robot_web_frontend/src/api/client.ts`（前端重寫後的 fetch 封裝；原 `services/api.service.ts` 已移除）
- **問題**: API 調用未驗證返回結構
- **建議**: 增加運行時類型驗證
- [ ] 待優化

### 15. ~~Launch 硬編碼路徑~~ ✅ 已修復
- **位置**: `src/nav2/launch/autonomous_navigation.launch.py`
- **修復**: 添加 `get_default_map_path()` 函數，檢查默認地圖是否存在，若不存在則嘗試查找其他可用地圖並發出警告

### 16. ~~日誌方法過時~~ ✅ 已修復
- **Commit**: `e16b4ff`
- **修復**: 將所有 `warn()` 更新為 `warning()`

### 27. ~~Launch 設備路徑硬編碼~~ ✅ 已修復
- **Commit**: `8ae36c6`
- **修復**: 新增 `lidar_port` 和 `imu_device` launch 參數

### 17. 健康檢查頻率
- **位置**: `src/robot_api_server/robot_api_server/main.py`（重寫後為 `_mission_monitor_loop`，0.5 秒輪詢；健康監控在 `state.start_health_monitor()`）
- **問題**: 輪詢間隔可能過於頻繁
- **建議**: 考慮增加間隔或使用事件驅動
- [ ] 待評估

### 18. ~~廣播任務優化~~ ✅ 經檢查無此問題
- **說明**: 與問題 10 相同，已在 `status_broadcast_loop()` 中實現連接檢查

---

## 已修復問題

### ✅ Bug 1: 馬達控制狀態線程安全
- **Commit**: `8781549`
- **修復**: 添加 `state_lock` 保護並發狀態訪問

### ✅ Bug 2: 方向同步問題
- **修復**: 通過 Bug 1 的鎖機制一併解決

### ✅ Bug 3: API Server 同步/非同步混用
- **Commit**: `8dc04bf`
- **修復**: 所有阻塞操作改用 `asyncio.to_thread()`

### ✅ Bug 4: 進程清理競態條件
- **Commit**: `245c54b`
- **修復**: 添加 `_wait_for_process_cleanup()` 驗證進程終止

### ✅ Bug 5: Launch 啟動順序
- **Commit**: `3267bf2`
- **修復**: 使用 `TimerAction` 控制啟動順序

---

## 修復優先級建議

1. **立即修復** (嚴重問題):
   - ~~問題 1: 路徑注入漏洞~~ ✅
   - ~~問題 2: State Manager 競態~~ ✅
   - ~~問題 3: 串口資源洩漏~~ ✅
   - ~~問題 19: nav2_commander rclpy 重複初始化~~ ✅
   - ~~問題 20: Modbus 控制器 wheel_separation 預設值~~ ✅

2. **短期修復** (中等問題):
   - ~~問題 4-8~~ ✅
   - ~~問題 21: 里程計更新頻率不匹配~~ ✅
   - ~~問題 22: 時間差為零未檢查~~ ✅
   - ~~問題 23: Modbus 控制器缺少參數驗證~~ ✅
   - ~~問題 24: destroy_node() 異常處理~~ ✅
   - ~~問題 25: Launch 配置讀取異常處理~~ ✅
   - ~~問題 26: 方向變數非原子更新~~ ✅

3. **中期改進** (輕微問題):
   - ~~問題 9: 時間處理混用~~ ✅
   - ~~問題 10: 狀態廣播效率~~ ✅
   - ~~問題 11: QoS 配置不完整~~ ✅
   - 問題 12: 異常處理過於寬泛
   - ~~問題 13: cmd_vel NaN 輸入未驗證~~ ✅
   - 問題 14: 前端 API 返回未驗證
   - ~~問題 15: Launch 硬編碼路徑~~ ✅
   - ~~問題 16: 日誌方法過時~~ ✅
   - 問題 17: 健康檢查頻率（待評估）
   - ~~問題 18: 廣播任務優化~~ ✅
   - ~~問題 27: Launch 設備路徑硬編碼~~ ✅

---

## 🚀 Jetson Orin NX 效能優化 (Performance Optimization)

> 目標：充分利用 Jetson Orin NX 16GB 的硬體資源，降低 CPU 使用率、減少延遲波動

### 🔴 高優先級 - 立即可做

### 42. ~~CPU 親和性未設置~~ ✅ 已修復
- **位置**: `src/motor_control/launch/bringup.launch.py`, `src/nav2/launch/autonomous_navigation.launch.py`
- **修復**: 使用 `taskset` 綁定節點到指定核心
  - 核心 0-1：馬達控制（實時性最高）
  - 核心 2-3：LiDAR/IMU 處理
  - 核心 4-5：EKF/AMCL 定位
  - 核心 6-7：Web 服務/API
- **新增參數**: `cpu_affinity:=true/false`（預設啟用）
- **預期效果**: 上下文切換減少 70%，延遲波動降低

### 43. ~~IMU 發布頻率過高~~ ✅ 已修復
- **位置**: `src/motor_control/launch/bringup.launch.py`
- **修復**: 將 IMU 發布頻率從 100Hz 降至 50Hz（真實硬體和模擬）
- **預期效果**: CPU 使用率降低 5-10%

### 44. SLAM Toolbox 記憶體和處理優化
- **位置**: `src/nav2/config/slam_toolbox_params.yaml`
- **問題**: 緩衝區和處理頻率可進一步優化
- **建議**:
  ```yaml
  throttle_scans: 2              # 每 2 次掃描處理一次
  stack_size_to_use: 20000000    # 20MB（從 40MB 降低）
  scan_buffer_size: 5            # 從 10 降低
  tf_buffer_duration: 15.0       # 從 30.0 降低
  ```
- **預期效果**: CPU 20-30% 降低，RAM 25% 降低
- [ ] 待實作

---

### 🟡 中優先級 - 短期改進

### 45. GPU/CUDA 完全未使用
- **位置**: 系統級
- **問題**: Jetson Orin NX 的 GPU 資源完全閒置
- **建議**:
  - SLAM 掃描匹配可用 CUDA 加速
  - 點雲濾波可用 GPU 處理
  - 未來如加入視覺，使用 TensorRT 推理
- **預期效果**: 計算密集任務 50-100x 加速
- [ ] 待評估

### 46. LiDAR 最大範圍可縮減
- **位置**: `src/nav2/config/nav2_params.yaml`
- **問題**: LiDAR 處理 12m 範圍，但室內多數情況 8m 足夠
- **建議**: 將 `laser_max_range` 從 12.0 降至 8.0
- **預期效果**: 計算量減少 20%
- [ ] 待實作

### 47. QoS 隊列深度可優化
- **位置**: 各 Python 節點
- **問題**: 所有話題統一使用 depth=10，未按頻率調整
- **建議**:
  - 高頻話題 (50+ Hz): depth=5
  - 中頻話題 (10-50 Hz): depth=10
  - 低頻話題 (<10 Hz): depth=20
- **預期效果**: 記憶體使用降低 15%
- [ ] 待實作

### 48. EKF 噪聲協方差可微調
- **位置**: `src/motor_control/config/hs_motor_config.yaml`
- **問題**: 當前協方差矩陣可能導致收斂較慢
- **建議**:
  - 位置噪聲：0.05 → 0.03（加速收斂）
  - 方向噪聲：0.06 → 0.02（更信任 IMU）
- **預期效果**: EKF 收斂速度提升
- [ ] 待評估

---

### 🟢 低優先級 - 長期優化

### 49. 共享記憶體未配置
- **位置**: `/etc/sysctl.conf`
- **問題**: 系統共享記憶體未針對 ROS2 DDS 優化
- **建議**:
  ```bash
  kernel.shmmax=2147483648
  kernel.shmall=524288
  net.core.rmem_max=134217728
  net.core.wmem_max=134217728
  ```
- **預期效果**: DDS 通訊效能提升
- [ ] 待實作

### 50. 實時線程優先級未設置
- **位置**: `src/motor_control/motor_control/hs_motor_controller.py`
- **問題**: 馬達控制線程未使用實時調度 (SCHED_FIFO)
- **建議**: 為關鍵線程設置實時優先級（需 root 權限）
- **預期效果**: 延遲確定性提升，波動 <5ms
- [ ] 待評估

### 51. 串口緩衝區可增大
- **位置**: `src/motor_control/motor_control/hs_motor_controller.py:128`
- **問題**: 使用預設串口緩衝區大小
- **建議**: 增大至 4096 bytes
  ```python
  self.serial_conn.set_buffer_size(rx_size=4096, tx_size=4096)
  ```
- **預期效果**: 丟包率降低
- [ ] 待實作

### 52. API Server 連接池未實作
- **位置**: `src/robot_api_server/robot_api_server/main.py`
- **問題**: 每次請求可能創建新線程
- **建議**: 使用 `ThreadPoolExecutor` 限制並發數
- **預期效果**: 吞吐量提升 30-40%
- [ ] 待實作

---

## 📊 效能優化預期總結

| 優化類別 | CPU 降低 | RAM 降低 | 延遲改善 |
|---------|---------|---------|---------|
| CPU 親和性 (42) | 15-20% | - | 30-40% |
| IMU 頻率 (43) | 5-10% | - | - |
| SLAM 優化 (44) | 20-30% | 25% | - |
| LiDAR 範圍 (46) | 10-15% | - | - |
| QoS 優化 (47) | - | 15% | 5-10% |
| **總計** | **40-50%** | **35-40%** | **35-50%** |
