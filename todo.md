# 專案待修復問題清單

> 最後更新: 2026-01-26 (修復問題 28-30, 32-35, 37, 40-41)

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

### 31. API Server rclpy 與 FastAPI 多線程問題
- **位置**: `src/robot_api_server/robot_api_server/main.py:1059-1146`
- **問題**: `NavigatorManager` 在多個 API 請求間共享，FastAPI 是多線程的
- **風險**: rclpy 操作可能在非預期線程執行
- **建議**: 使用 `MultiThreadedExecutor` 或確保 rclpy 操作在專用線程
- [ ] 待評估

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

### 36. EKF 從 odom 取得 vyaw 可能不準確
- **位置**: `src/motor_control/config/hs_motor_config.yaml:38-42`
- **問題**: `odom0_config` 使用 vyaw（角速度），但輪式里程計的角速度不如 IMU 準確
- **建議**: 考慮只從 IMU 獲取角速度，將 odom0 的 vyaw 設為 false
- [ ] 待評估

---

## 🟢 新發現 - 低優先級 (Low Priority)

### 37. ~~重複的協方差矩陣定義~~ ✅ 已修復
- **Commit**: `5e6fa22`
- **修復**: 新增 odom_constants.py 共用模組，三個控制器共用常數

### 38. Costmap 同時使用 obstacle_layer 和 voxel_layer
- **位置**: `src/nav2/config/nav2_params.yaml:170, 225`
- **問題**: 兩個 layer 都訂閱 `/scan`，可能造成重複處理
- **建議**: 對於 2D LiDAR，通常只需要 `obstacle_layer`
- [ ] 待評估

### 39. 缺少完整的 Type Hints
- **位置**: 多個 Python 檔案
- **問題**: 部分函數缺少返回類型提示
- **建議**: 添加完整類型註解，例如 `def update_odometry(self) -> None:`
- [ ] 待優化

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
- **位置**: `src/robot_web_frontend/src/services/api.service.ts`
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
- **位置**: `src/robot_api_server/robot_api_server/main.py:64`
- **問題**: 每 2 秒檢查一次可能過於頻繁
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
