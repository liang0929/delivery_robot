# 專案待修復問題清單

> 最後更新: 2026-01-22

---

## 嚴重問題 (Critical)

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

### 8. TF 配置可能不匹配
- **位置**: `bringup.launch.py` vs `autonomous_navigation.launch.py`
- **問題**: 兩個 launch 文件可能發布衝突的 TF
- **建議**: 確認各節點使用的 frame_id 一致
- [ ] 待檢查

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
- **位置**: `src/motor_control/motor_control/hs_motor_controller.py:188`
- **問題**: `self.get_logger().warn()` 應為 `warning()`
- **建議**: 更新為標準方法名
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

1. **立即修復** (安全相關):
   - 問題 1: 路徑注入漏洞
   - 問題 2: State Manager 競態
   - 問題 3: 串口資源洩漏

2. **短期修復** (穩定性):
   - 問題 4: Subprocess 管道
   - 問題 5: rclpy 初始化
   - 問題 6: 參數一致性
   - 問題 7: 鎖內發布

3. **中期改進** (代碼質量):
   - 問題 9-18: 輕微問題
