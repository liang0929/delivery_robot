# 專案待修復問題清單

> 最後更新: 2026-01-22

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

### 21. 里程計更新頻率不匹配
- **位置**: `src/motor_control/motor_control/modbus_motor_controller.py:105`
- **問題**: Modbus 控制器 `odom_frequency=20Hz`，HS 控制器 `control_frequency=50Hz`
- **影響**: 切換控制器時里程計更新頻率改變，可能導致 EKF 融合不穩定
- **建議**: 統一兩個控制器的頻率參數
- [ ] 待修復

### 22. 時間差為零未檢查
- **位置**: `src/motor_control/motor_control/hs_motor_controller.py:477`
- **問題**: 計算 `dt` 後未檢查是否為零
- **影響**: 若連續兩次呼叫在同一時刻，`dt=0` 會導致位置無更新
- **建議**: 添加 `if dt <= 0: return` 檢查
- [ ] 待修復

### 23. Modbus 控制器缺少參數驗證
- **位置**: `src/motor_control/motor_control/modbus_motor_controller.py`
- **問題**: 缺少 `_validate_parameters()` 方法（HS 控制器有）
- **影響**: 無效頻率參數（如 0 或負數）會導致除零錯誤
- **建議**: 添加參數驗證函數
- [ ] 待修復

### 24. destroy_node() 異常處理不完整
- **位置**: `src/motor_control/motor_control/modbus_motor_controller.py:386-387`
- **問題**: `client.close()` 異常未捕獲，可能導致 `super().destroy_node()` 未執行
- **建議**: 使用 try-finally 確保節點正確銷毀
- [ ] 待修復

### 25. Launch 配置讀取無異常處理
- **位置**: `src/motor_control/launch/bringup.launch.py:31-32`
- **問題**: 讀取 `tf_config.yaml` 時無異常處理
- **影響**: 檔案不存在或格式無效會導致 launch 失敗，無清晰錯誤訊息
- **建議**: 添加 try-except 和預設值
- [ ] 待修復

### 26. 方向變數非原子更新
- **位置**: `src/motor_control/motor_control/hs_motor_controller.py:513-525`
- **問題**: `logical_dir_a/b` 更新與讀取之間可能發生上下文切換
- **影響**: 里程計計算使用不一致的方向值
- **建議**: 使用鎖保護或原子操作
- [ ] 待修復

---

## 輕微問題 (Minor)

### 9. 時間處理混用
- **位置**: `src/motor_control/motor_control/hs_motor_controller.py:110, 454`
- **問題**: 混用 `time.time()` 和 `self.get_clock().now()`，模擬環境會不一致
- **建議**: 統一使用 `self.get_clock()`
- [ ] 待修復

### 10. 狀態廣播效率
- **位置**: `src/robot_api_server/robot_api_server/main.py:462-479`
- **問題**: 即使無 WebSocket 客戶端仍每秒執行 `get_full_status()`
- **建議**: 無連接時暫停廣播任務
- [ ] 待優化

### 11. QoS 配置不完整
- **位置**: `src/motor_control/motor_control/hs_motor_controller.py:80`
- **問題**: `QoSProfile(depth=10)` 未設置 reliability, durability
- **建議**: 顯式設置完整 QoS 參數
- [ ] 待修復

### 12. 異常處理過於寬泛
- **位置**: `src/robot_api_server/robot_api_server/main.py` 多處
- **問題**: 大量 `except Exception` 難以診斷問題
- **建議**: 捕獲具體異常類型
- [ ] 待優化

### 13. cmd_vel NaN 輸入未驗證
- **位置**: `src/motor_control/motor_control/hs_motor_controller.py:465-478`
- **問題**: `cmd_vel_callback()` 未驗證 NaN 值
- **建議**: 檢查 `math.isnan()` 和 `math.isinf()`
- [ ] 待修復

### 14. 前端 API 返回未驗證
- **位置**: `src/robot_web_frontend/src/services/api.service.ts`
- **問題**: API 調用未驗證返回結構
- **建議**: 增加運行時類型驗證
- [ ] 待優化

### 15. Launch 硬編碼路徑
- **位置**: `src/nav2/launch/autonomous_navigation.launch.py:40`
- **問題**: `~/base_dev/src/map/map.yaml` 硬編碼
- **建議**: 使用 ROS 包路徑或環境變數
- [ ] 待修復

### 16. 日誌方法過時
- **位置**:
  - `src/motor_control/motor_control/hs_motor_controller.py:387, 402, 441, 443`
  - `src/motor_control/motor_control/modbus_motor_controller.py:380`
- **問題**: `self.get_logger().warn()` 應為 `warning()`
- **建議**: 更新為標準方法名
- [ ] 待修復

### 27. Launch 設備路徑硬編碼
- **位置**: `src/motor_control/launch/bringup.launch.py:86, 101`
- **問題**: LiDAR (`/dev/lidar`) 和 IMU (`/dev/i2c-7`) 設備路徑硬編碼
- **建議**: 使用參數或環境變數配置
- [ ] 待修復

### 17. 健康檢查頻率
- **位置**: `src/robot_api_server/robot_api_server/main.py:64`
- **問題**: 每 2 秒檢查一次可能過於頻繁
- **建議**: 考慮增加間隔或使用事件驅動
- [ ] 待評估

### 18. 廣播任務優化
- **位置**: `src/robot_api_server/robot_api_server/main.py:462-479`
- **問題**: 無連接時仍每秒檢查
- **建議**: 實現連接驅動的廣播機制
- [ ] 待優化

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
   - 問題 21: 里程計更新頻率不匹配
   - 問題 22: 時間差為零未檢查
   - 問題 23: Modbus 控制器缺少參數驗證
   - 問題 24: destroy_node() 異常處理
   - 問題 25: Launch 配置讀取異常處理
   - 問題 26: 方向變數非原子更新

3. **中期改進** (輕微問題):
   - 問題 9-18, 27: 代碼質量改進
