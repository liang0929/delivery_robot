# 專案待修復問題清單

> 最後更新: 2026-01-26

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
