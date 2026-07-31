# AMR 專案重構計劃

> 建立日期：2026-02-06
> 狀態：部分完成（最後校對：2026-07-22）

> **2026-07-22 狀態更新**：Phase 1（拆分 main.py）已在 Winstec Robot API v1.1
> 重構（commit `d643599` 起的系列提交）中實質完成——`main.py` 現約 150 行，
> 拆分為 `routers/`（而非本計畫原訂的 `routes/`）、`models.py`、`ros_bridge.py`、
> `store.py`、`ws_server.py` 等模組。下文「現況分析」保留的是撰寫當時的狀態；
> 其餘階段（`nav2` → `robot_navigation` 重命名、`robot_bringup` 統一入口等）尚未執行。

---

## 現況分析

### 問題清單

| 檔案 | 行數 | 問題 |
|------|------|------|
| `robot_api_server/main.py` | ~~1874~~ → 152 | ~~單一檔案過大，職責混雜~~ ✅ 已拆分為多模組 |
| `motor_control/` | - | 混合馬達、模擬、工具，職責不清 |
| `nav2/` | - | 命名與官方 nav2 套件混淆 |
| 配置檔 | - | 參數分散在多處，難以維護 |
| 啟動流程 | - | 需多個指令，無統一入口 |

### 目前結構

```
src/
├── motor_control/          # 馬達 + 模擬 + 工具混雜
├── nav2/                   # 命名混淆
├── robot_api_server/       # ✅ 已模組化（main.py ~150 行 + routers/ 等）
├── robot_description/      # OK
├── robot_web_frontend/     # OK
├── ros-imu-bno055/         # 第三方
└── sllidar_ros2/           # 第三方
```

---

## 目標架構

```
base_dev/
├── src/
│   ├── robot_bringup/           # 統一啟動入口
│   │   ├── launch/
│   │   │   ├── robot.launch.py  # 唯一入口
│   │   │   └── simulation.launch.py
│   │   └── config/
│   │       ├── robot_params.yaml
│   │       └── hardware_params.yaml
│   │
│   ├── robot_hardware/          # 硬體驅動層
│   │   ├── motor/
│   │   │   ├── hs_driver.py     # 純協議通訊
│   │   │   ├── motor_node.py    # ROS2 節點
│   │   │   └── kinematics.py    # 運動學計算
│   │   └── config/
│   │       └── motor_config.yaml
│   │
│   ├── robot_simulation/        # 模擬層（獨立）
│   │   ├── nodes/
│   │   │   ├── mock_motor.py
│   │   │   ├── mock_lidar.py
│   │   │   └── mock_imu.py
│   │   ├── worlds/
│   │   │   ├── empty.yaml
│   │   │   ├── room.yaml
│   │   │   └── corridor.yaml
│   │   └── launch/
│   │       └── simulation.launch.py
│   │
│   ├── robot_navigation/        # 導航層（重命名）
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   └── slam_params.yaml
│   │   ├── launch/
│   │   │   ├── navigation.launch.py
│   │   │   └── mapping.launch.py
│   │   └── behavior_trees/
│   │
│   ├── robot_api/               # API 層（重構）
│   │   ├── api/
│   │   │   ├── main.py          # < 100 行
│   │   │   └── routes/
│   │   │       ├── navigation.py
│   │   │       ├── mapping.py
│   │   │       ├── teleop.py
│   │   │       └── system.py
│   │   ├── services/
│   │   │   ├── process_manager.py
│   │   │   ├── ros_bridge.py
│   │   │   └── map_service.py
│   │   └── models/
│   │       └── schemas.py
│   │
│   ├── robot_description/       # 保持不變
│   ├── robot_web_frontend/      # 保持不變
│   ├── ros-imu-bno055/          # 第三方，保持
│   └── sllidar_ros2/            # 第三方，保持
│
├── config/                      # 全域配置
│   └── logging.yaml
├── maps/                        # 地圖檔案
└── logs/                        # 日誌輸出
```

---

## 重構階段

### Phase 1：拆分 main.py（優先級：高）

**目標**：1874 行 → 多個模組，每個 < 300 行

**步驟**：

1. 建立目錄結構
   ```bash
   mkdir -p src/robot_api_server/robot_api_server/{routes,services,models}
   ```

2. 抽取 Pydantic 模型
   ```
   models/schemas.py
   - NavigationGoal
   - MapInfo
   - RobotStatus
   - ...
   ```

3. 抽取服務層
   ```
   services/process_manager.py  (~300 行)
   - start_slam()
   - stop_slam()
   - start_navigation()
   - stop_navigation()
   - start_robot_core()
   - stop_robot_core()
   - _terminate_process_safely()

   services/ros_bridge.py  (~200 行)
   - RosBridgeNode
   - publish_goal()
   - publish_initial_pose()
   - get_robot_pose()

   services/map_service.py  (~150 行)
   - list_maps()
   - save_map()
   - load_map()
   - delete_map()
   ```

4. 抽取路由
   ```
   routes/navigation.py
   - POST /navigation/start
   - POST /navigation/stop
   - POST /navigation/goal
   - GET  /navigation/status

   routes/mapping.py
   - POST /mapping/start
   - POST /mapping/stop
   - POST /mapping/save

   routes/teleop.py
   - POST /teleop/cmd_vel
   - POST /teleop/stop

   routes/system.py
   - GET  /health
   - GET  /status
   - POST /robot/start
   - POST /robot/stop
   ```

5. 簡化 main.py
   ```python
   # main.py (< 100 行)
   from fastapi import FastAPI
   from .routes import navigation, mapping, teleop, system
   from .services.process_manager import ProcessManager

   app = FastAPI(title="Robot API")
   manager = ProcessManager()

   app.include_router(navigation.router, prefix="/navigation")
   app.include_router(mapping.router, prefix="/mapping")
   app.include_router(teleop.router, prefix="/teleop")
   app.include_router(system.router)

   @app.on_event("startup")
   async def startup():
       manager.start_health_monitor()

   @app.on_event("shutdown")
   async def shutdown():
       manager.cleanup()
   ```

**驗收標準**：
- [ ] main.py < 100 行
- [ ] 每個模組 < 300 行
- [ ] 所有 API 端點正常運作
- [ ] 單元測試通過

---

### Phase 2：統一啟動入口（優先級：高）

**目標**：一個指令啟動所有模式

**建立 robot_bringup 套件**：

```bash
ros2 pkg create robot_bringup --build-type ament_python
```

**統一啟動檔**：

```python
# robot_bringup/launch/robot.launch.py

def generate_launch_description():
    mode = LaunchConfiguration('mode')  # teleop / mapping / navigation
    simulation = LaunchConfiguration('simulation')

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='teleop'),
        DeclareLaunchArgument('simulation', default_value='false'),
        DeclareLaunchArgument('map', default_value=''),

        # 核心節點（總是啟動）
        IncludeLaunchDescription('robot_hardware/bringup.launch.py'),

        # 根據模式啟動對應功能
        IncludeLaunchDescription('robot_navigation/mapping.launch.py',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'mapping'"]))),

        IncludeLaunchDescription('robot_navigation/navigation.launch.py',
            condition=IfCondition(PythonExpression(["'", mode, "' == 'navigation'"]))),
    ])
```

**使用方式**：

```bash
# 遙控模式
ros2 launch robot_bringup robot.launch.py

# 建圖模式
ros2 launch robot_bringup robot.launch.py mode:=mapping

# 導航模式
ros2 launch robot_bringup robot.launch.py mode:=navigation map:=office

# 模擬模式
ros2 launch robot_bringup robot.launch.py simulation:=true mode:=navigation
```

**驗收標準**：
- [ ] 單一指令可啟動任何模式
- [ ] 參數傳遞正確
- [ ] 模擬/真實無縫切換

---

### Phase 3：分離模擬層（優先級：中）

**目標**：模擬和真實硬體完全解耦

**建立 robot_simulation 套件**：

```bash
ros2 pkg create robot_simulation --build-type ament_python
```

**移動檔案**：

```
motor_control/mock_motor_controller.py → robot_simulation/nodes/mock_motor.py
motor_control/mock_lidar.py            → robot_simulation/nodes/mock_lidar.py
motor_control/mock_imu.py              → robot_simulation/nodes/mock_imu.py
```

**統一模擬參數**：

```yaml
# robot_simulation/config/simulation_params.yaml
simulation:
  # 與真實硬體保持一致！
  wheel_separation: 0.27
  wheel_radius: 0.065
  max_linear_vel: 0.05   # 改為與真實一致
  max_angular_vel: 0.4

  # 模擬專用
  noise:
    odom_linear: 0.01
    odom_angular: 0.02
    imu_orientation: 0.001
```

**驗收標準**：
- [ ] 模擬參數與真實硬體一致
- [ ] motor_control 不再包含模擬代碼
- [ ] 模擬可獨立測試

---

### Phase 4：硬體驅動重構（優先級：中）

**目標**：分離協議層和 ROS 層

**重構 hs_motor_controller.py**：

```python
# robot_hardware/motor/hs_driver.py
# 純 Python，無 ROS 依賴，可單獨測試

class HSProtocol:
    """HS 協議通訊層"""

    def __init__(self, port: str, baudrate: int = 115200):
        self.serial = serial.Serial(port, baudrate)

    def build_command(self, rpm_a: int, rpm_b: int, dir_a: int, dir_b: int) -> bytes:
        """建構命令封包"""
        ...

    def parse_response(self, data: bytes) -> dict:
        """解析回應"""
        ...

    def send_speed_command(self, rpm_a: int, rpm_b: int, dir_a: int, dir_b: int) -> dict:
        """發送速度命令並返回狀態"""
        cmd = self.build_command(rpm_a, rpm_b, dir_a, dir_b)
        self.serial.write(cmd)
        response = self.serial.read(...)
        return self.parse_response(response)


# robot_hardware/motor/kinematics.py
# 純數學計算

class DifferentialKinematics:
    """差動驅動運動學"""

    def __init__(self, wheel_separation: float, wheel_radius: float, gear_ratio: float):
        self.wheel_separation = wheel_separation
        self.wheel_radius = wheel_radius
        self.gear_ratio = gear_ratio

    def twist_to_wheel_vel(self, linear: float, angular: float) -> tuple[float, float]:
        """Twist → 左右輪速度 (m/s)"""
        left = linear - (angular * self.wheel_separation / 2.0)
        right = linear + (angular * self.wheel_separation / 2.0)
        return left, right

    def wheel_vel_to_rpm(self, vel: float) -> float:
        """輪速 (m/s) → 馬達 RPM"""
        wheel_rpm = abs(vel) / (2 * math.pi * self.wheel_radius) * 60.0
        return wheel_rpm * self.gear_ratio


# robot_hardware/motor/motor_node.py
# ROS2 封裝

class MotorNode(Node):
    """馬達控制 ROS2 節點"""

    def __init__(self):
        super().__init__('motor_controller')

        # 載入參數
        ...

        # 初始化驅動和運動學
        self.driver = HSProtocol(self.serial_port, self.baudrate)
        self.kinematics = DifferentialKinematics(...)

        # ROS2 介面
        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', 10)
```

**驗收標準**：
- [ ] HSProtocol 可獨立單元測試
- [ ] DifferentialKinematics 可獨立單元測試
- [ ] ROS 節點僅處理 ROS 相關邏輯

---

### Phase 5：配置統一（優先級：低）

**目標**：所有參數集中管理

**建立全域配置結構**：

```yaml
# robot_bringup/config/robot_params.yaml
robot:
  # 物理參數（共用）
  wheel_separation: 0.27
  wheel_radius: 0.065
  gear_ratio: 20.0

  # 速度限制
  max_linear_vel: 0.05
  max_angular_vel: 0.4

# robot_bringup/config/hardware_params.yaml
hardware:
  motor:
    port: /dev/motor
    baudrate: 115200
    device_id: 127
    min_rpm: 100.0
    max_rpm: 3000.0

  lidar:
    port: /dev/lidar
    baudrate: 256000

  imu:
    device: /dev/i2c-7
    address: 0x28
```

---

## 快速勝利（可立即執行）

這些改動風險低、效益高，可先執行：

### 1. 重命名 nav2 → robot_navigation

```bash
# 避免與官方 nav2 套件混淆
mv src/nav2 src/robot_navigation
# 更新 package.xml 和 setup.py
```

### 2. 統一模擬參數

```yaml
# bringup.launch.py 中的模擬參數改為與真實一致
max_linear_vel: 0.05   # 原本 0.5
max_angular_vel: 0.4   # 原本 1.0
```

### 3. 抽取 API schemas

```python
# 先把 Pydantic 模型抽出來
# models/schemas.py
class NavigationGoal(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
```

---

## 時程估計

| Phase | 工作量 | 風險 | 建議順序 |
|-------|--------|------|----------|
| Phase 1：拆分 main.py | 2-3 天 | 中 | 1 |
| Phase 2：統一啟動 | 1-2 天 | 低 | 2 |
| Phase 3：分離模擬 | 1 天 | 低 | 3 |
| Phase 4：硬體重構 | 2-3 天 | 高 | 4 |
| Phase 5：配置統一 | 0.5 天 | 低 | 5 |
| 快速勝利 | 0.5 天 | 低 | 0（先做）|

---

## 注意事項

1. **向後相容**：重構過程中保持 API 端點不變
2. **逐步遷移**：每個 Phase 完成後完整測試再進入下一階段
3. **Git 分支**：每個 Phase 使用獨立分支，完成後合併
4. **文檔同步**：更新 README 和啟動說明

---

## 參考資源

- [ROS2 Package Design](https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html)
- [FastAPI Project Structure](https://fastapi.tiangolo.com/tutorial/bigger-applications/)
- [Python Project Structure](https://docs.python-guide.org/writing/structure/)
