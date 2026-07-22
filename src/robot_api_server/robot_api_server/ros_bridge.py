"""rclpy 橋接層：節點生命週期、Nav2 導航、手動移動、電壓、模式切換。

設計原則（沿用先前併發修正的精神）：

- rclpy 只初始化一次，執行緒安全
- 狀態鎖內只做「狀態檢查 + 過渡狀態轉移」，spawn/清理/等待等慢操作在鎖外
- 阻塞呼叫（goToPose、cancelTask…）由呼叫端以
  ``asyncio.to_thread`` 包裝，不得在事件迴圈上直接執行
- goal 世代握手：狀態推進後、goal 實際送出前不得用 ``isTaskComplete()`` 判斷完成
- ``/initialpose`` 發布前先輪詢 ``get_subscription_count()`` 確認 AMCL 已訂閱
- ROS 不可用時全部優雅降級，不讓 API server crash
"""

import io
import math
import os
import signal
import subprocess
import threading
import time
from enum import Enum
from typing import Optional, Tuple
from uuid import uuid4

from .config import (
    BATTERY_MAX_V,
    BATTERY_MIN_V,
    MANUAL_ANGULAR_SPEED,
    MANUAL_LINEAR_SPEED,
    MANUAL_PUBLISH_HZ,
    MAP_PATH,
    WORKSPACE_ROOT,
)
from .conversions import (
    cm_to_m,
    deg_to_yaw,
    m_to_cm,
    quaternion_to_yaw,
    voltage_to_battery,
    yaw_to_deg,
    yaw_to_quaternion,
)
from .logging_config import get_logger
from .models import Direction, EventCode, Location, OpMode, RobotStatus

logger = get_logger(__name__)


# --- ROS 相依：不可用時優雅降級 ---
ROS_AVAILABLE = True
ROS_IMPORT_ERROR: Optional[str] = None
try:
    import rclpy
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import OccupancyGrid
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float32
except Exception as e:  # pragma: no cover - 取決於執行環境
    ROS_AVAILABLE = False
    ROS_IMPORT_ERROR = str(e)
    logger.error(f"ROS packages unavailable, running in degraded mode: {e}")

NAV2_AVAILABLE = ROS_AVAILABLE
try:
    if ROS_AVAILABLE:
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except Exception as e:  # pragma: no cover
    NAV2_AVAILABLE = False
    logger.error(f"nav2_simple_commander unavailable: {e}")

# 等待 Nav2 就緒的上限。冷啟動時全部節點 active 約需 10-20 秒，留足餘裕。
NAV2_READY_TIMEOUT_SEC = float(os.environ.get('ROBOT_NAV2_READY_TIMEOUT', '40'))

try:
    from PIL import Image
    PIL_AVAILABLE = True
except Exception:  # pragma: no cover
    PIL_AVAILABLE = False


# --- rclpy 全局初始化管理 ---
_rclpy_init_lock = threading.Lock()
_rclpy_initialized = False


def ensure_rclpy_initialized() -> bool:
    """執行緒安全地確保 rclpy 只初始化一次"""
    global _rclpy_initialized
    if not ROS_AVAILABLE:
        return False
    with _rclpy_init_lock:
        if _rclpy_initialized:
            return True
        try:
            rclpy.init()
            _rclpy_initialized = True
            logger.info("rclpy initialized successfully")
            return True
        except RuntimeError as e:
            # 只有「已初始化」才視為成功；其他 RuntimeError 是真正的初始化失敗
            if 'already' in str(e).lower():
                _rclpy_initialized = True
                logger.info("rclpy already initialized elsewhere")
                return True
            logger.error(f"Failed to initialize rclpy: {e}")
            return False
        except Exception as e:
            logger.error(f"Failed to initialize rclpy: {e}")
            return False


class SlamStatus(str, Enum):
    IDLE = "idle"
    STARTING = "starting"
    MAPPING = "mapping"
    SAVING = "saving"
    STOPPING = "stopping"


class NavStatus(str, Enum):
    IDLE = "idle"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"


class ProcessError(RuntimeError):
    """子程序生命週期操作失敗"""


class RobotStateManager:
    """SLAM / Navigation / Robot core 子程序的執行緒安全管理器。"""

    HEALTH_CHECK_INTERVAL = 2.0

    # pkill -f 用的殘留進程清理 pattern。
    # 自己啟動的進程一律優先透過 Popen 的 pgid（_terminate_process_safely）終止，
    # pkill 只用來清理上次 server 異常結束留下的孤兒進程。
    # pattern 必須錨定完整指令或「套件/執行檔」路徑，避免誤殺
    # （例如舊 pattern "nav2_" 會殺掉存圖中的 nav2_map_server/map_saver_cli）。
    NAV_CLEANUP_PATTERNS = [
        "ros2 launch nav2 autonomous_navigation",
        "nav2_map_server/map_server",
        "nav2_amcl/amcl",
        "nav2_controller/controller_server",
        "nav2_planner/planner_server",
        "nav2_smoother/smoother_server",
        "nav2_behaviors/behavior_server",
        "nav2_bt_navigator/bt_navigator",
        "nav2_waypoint_follower/waypoint_follower",
        "nav2_velocity_smoother/velocity_smoother",
        "nav2_lifecycle_manager/lifecycle_manager",
        "motor_control/map_relay",
    ]
    SLAM_CLEANUP_PATTERNS = [
        "ros2 launch nav2 mapping.launch",
        "slam_toolbox/async_slam_toolbox_node",
        "slam_toolbox/sync_slam_toolbox_node",
        "motor_control/map_relay",
    ]

    def __init__(self):
        self._lock = threading.Lock()
        self._slam_process: Optional[subprocess.Popen] = None
        self._slam_status = SlamStatus.IDLE
        self._nav_process: Optional[subprocess.Popen] = None
        self._nav_status = NavStatus.IDLE
        self._current_map: Optional[str] = None
        self._robot_core_process: Optional[subprocess.Popen] = None
        self._robot_core_running = False
        self._robot_core_transition = False
        self._e_stop_active = False
        self._health_thread: Optional[threading.Thread] = None
        self._health_stop_event = threading.Event()
        self._crash_info: dict = {}
        # 導航停止/崩潰時的善後 callback（在鎖外呼叫）
        self.on_navigation_down = None

    # --- E-Stop ---
    @property
    def e_stop_active(self) -> bool:
        with self._lock:
            return self._e_stop_active

    @e_stop_active.setter
    def e_stop_active(self, value: bool):
        with self._lock:
            self._e_stop_active = value

    # --- 狀態 ---
    @property
    def slam_status(self) -> SlamStatus:
        with self._lock:
            return self._slam_status

    @property
    def nav_status(self) -> NavStatus:
        with self._lock:
            return self._nav_status

    @property
    def current_map(self) -> Optional[str]:
        with self._lock:
            return self._current_map

    @current_map.setter
    def current_map(self, value: Optional[str]):
        with self._lock:
            self._current_map = value

    @property
    def is_busy(self) -> bool:
        """任一子系統處於過渡狀態"""
        with self._lock:
            return (
                self._slam_status in (SlamStatus.STARTING, SlamStatus.STOPPING)
                or self._nav_status in (NavStatus.STARTING, NavStatus.STOPPING)
                or self._robot_core_transition
            )

    def op_mode(self) -> OpMode:
        with self._lock:
            if self._slam_status != SlamStatus.IDLE:
                return OpMode.EXPLORE
            return OpMode.NAVIGATE

    # --- 進程工具 ---
    def _wait_for_process_cleanup(self, patterns: list, timeout: float = 5.0) -> bool:
        start_time = time.time()
        while time.time() - start_time < timeout:
            all_terminated = True
            for pattern in patterns:
                try:
                    result = subprocess.run(["pgrep", "-f", pattern], capture_output=True)
                    if result.returncode == 0:
                        all_terminated = False
                        break
                except Exception:
                    pass
            if all_terminated:
                return True
            time.sleep(0.2)
        return False

    def _cleanup_processes(self, patterns: list, label: str):
        for pattern in patterns:
            try:
                subprocess.run(["pkill", "-f", pattern], capture_output=True)
            except Exception as e:
                logger.warning(f"Failed to kill processes matching '{pattern}': {e}")
        if not self._wait_for_process_cleanup(patterns, timeout=3.0):
            logger.warning(f"Some {label} processes may still be running after cleanup")

    def _terminate_process_safely(
        self, process: subprocess.Popen, name: str, timeout: int = 5
    ) -> bool:
        if process is None:
            return True
        try:
            pgid = os.getpgid(process.pid)
            logger.info(f"Sending SIGTERM to {name} (pgid={pgid})")
            os.killpg(pgid, signal.SIGTERM)
            try:
                process.wait(timeout=timeout)
                logger.info(f"{name} terminated gracefully")
                return True
            except subprocess.TimeoutExpired:
                pass
            logger.warning(f"{name} did not terminate, sending SIGKILL")
            os.killpg(pgid, signal.SIGKILL)
            try:
                process.wait(timeout=3)
                return True
            except subprocess.TimeoutExpired:
                logger.error(f"Failed to kill {name}")
                return False
        except ProcessLookupError:
            return True
        except Exception as e:
            logger.error(f"Error terminating {name}: {e}")
            return False

    def _verify_process_started(
        self, process: subprocess.Popen, name: str, wait_time: float = 0.5
    ) -> bool:
        try:
            time.sleep(wait_time)
            exit_code = process.poll()
            if exit_code is not None:
                logger.error(f"{name} process exited immediately with code {exit_code}")
                return False
            return True
        except Exception as e:
            logger.error(f"Error verifying {name} process: {e}")
            return False

    # --- SLAM ---
    def start_slam(self) -> None:
        with self._lock:
            if self._slam_status != SlamStatus.IDLE:
                raise ProcessError("Mapping is already running or busy")
            if self._nav_status in (NavStatus.STARTING, NavStatus.STOPPING):
                raise ProcessError("Navigation is busy")
            nav_running = self._nav_status == NavStatus.RUNNING
            self._slam_status = SlamStatus.STARTING

        process: Optional[subprocess.Popen] = None
        try:
            # 導航運行中：先走正常停止路徑（以自己持有的 pgid 終止），而非直接 pkill
            if nav_running:
                logger.info("Navigation is running; stopping it before starting SLAM")
                try:
                    self.stop_navigation()
                except ProcessError:
                    pass
                if self.on_navigation_down is not None:
                    try:
                        self.on_navigation_down()
                    except Exception as e:
                        logger.warning(f"Navigation-down handler failed: {e}")

            self._cleanup_processes(self.NAV_CLEANUP_PATTERNS, "navigation")

            process = subprocess.Popen(
                ["ros2", "launch", "nav2", "mapping.launch.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            if not self._verify_process_started(process, "SLAM"):
                raise ProcessError("SLAM process failed to start")
        except ProcessError:
            if process is not None:
                self._terminate_process_safely(process, "SLAM", timeout=3)
            with self._lock:
                self._slam_status = SlamStatus.IDLE
            raise
        except Exception as e:
            if process is not None:
                self._terminate_process_safely(process, "SLAM", timeout=3)
            with self._lock:
                self._slam_status = SlamStatus.IDLE
            raise ProcessError(f"Failed to start mapping: {e}")

        with self._lock:
            self._slam_process = process
            self._slam_status = SlamStatus.MAPPING
            self._crash_info.pop('slam', None)

    def stop_slam(self) -> None:
        with self._lock:
            if self._slam_status not in (SlamStatus.MAPPING, SlamStatus.SAVING):
                raise ProcessError("Mapping is not running")
            process = self._slam_process
            self._slam_process = None
            self._slam_status = SlamStatus.STOPPING
        try:
            if process and not self._terminate_process_safely(process, "SLAM"):
                logger.error("SLAM process may still be running")
        finally:
            with self._lock:
                self._slam_status = SlamStatus.IDLE

    def begin_map_save(self) -> None:
        """原子性地從 MAPPING 進入 SAVING"""
        with self._lock:
            if self._slam_status != SlamStatus.MAPPING:
                raise ProcessError("Mapping is not running; cannot save map")
            self._slam_status = SlamStatus.SAVING

    def end_map_save(self) -> None:
        with self._lock:
            if self._slam_status == SlamStatus.SAVING:
                self._slam_status = SlamStatus.MAPPING

    # --- Navigation ---
    def start_navigation(self, map_name: Optional[str] = None) -> str:
        with self._lock:
            if self._nav_status != NavStatus.IDLE:
                raise ProcessError("Navigation is already running")
            if self._slam_status != SlamStatus.IDLE:
                slam_running = True
            else:
                slam_running = False
            self._nav_status = NavStatus.STARTING

        process: Optional[subprocess.Popen] = None
        try:
            if slam_running:
                logger.info("Mapping is running; stopping it before starting navigation")
                try:
                    self.stop_slam()
                except ProcessError:
                    pass

            if map_name:
                map_yaml = os.path.join(MAP_PATH, f"{map_name}.yaml")
                if not os.path.exists(map_yaml):
                    raise ProcessError(f"Map '{map_name}' not found")
            else:
                map_yaml = os.path.join(MAP_PATH, "map.yaml")

            self._cleanup_processes(self.SLAM_CLEANUP_PATTERNS, "SLAM")
            self._cleanup_processes(self.NAV_CLEANUP_PATTERNS, "navigation")

            process = subprocess.Popen(
                ["ros2", "launch", "nav2", "autonomous_navigation.launch.py",
                 f"map:={map_yaml}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            if not self._verify_process_started(process, "Navigation"):
                raise ProcessError("Navigation process failed to start")
        except ProcessError:
            if process is not None:
                self._terminate_process_safely(process, "Navigation", timeout=3)
            with self._lock:
                self._nav_status = NavStatus.IDLE
            raise
        except Exception as e:
            if process is not None:
                self._terminate_process_safely(process, "Navigation", timeout=3)
            with self._lock:
                self._nav_status = NavStatus.IDLE
            raise ProcessError(f"Failed to start navigation: {e}")

        with self._lock:
            self._nav_process = process
            self._nav_status = NavStatus.RUNNING
            self._crash_info.pop('navigation', None)
            self._current_map = map_name if map_name else "map"
            return self._current_map

    def stop_navigation(self) -> None:
        with self._lock:
            if self._nav_status != NavStatus.RUNNING:
                raise ProcessError("Navigation is not running")
            process = self._nav_process
            self._nav_process = None
            self._nav_status = NavStatus.STOPPING
        try:
            if process and not self._terminate_process_safely(process, "Navigation"):
                logger.error("Navigation process may still be running")
        finally:
            with self._lock:
                self._nav_status = NavStatus.IDLE

    # --- Robot Core ---
    @property
    def robot_core_running(self) -> bool:
        with self._lock:
            return self._robot_core_running

    def start_robot_core(self) -> None:
        with self._lock:
            if self._robot_core_running or self._robot_core_transition:
                raise ProcessError("Robot core is already running")
            self._robot_core_transition = True

        process: Optional[subprocess.Popen] = None
        try:
            existing = subprocess.run(
                ["pgrep", "-f", "ros2 launch motor_control bringup"], capture_output=True
            )
            if existing.returncode == 0:
                raise ProcessError("Robot core is already running outside this API")

            process = subprocess.Popen(
                ["ros2", "launch", "motor_control", "bringup.launch.py", "enable_web:=false"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            if not self._verify_process_started(process, "Robot Core"):
                raise ProcessError("Robot core process failed to start")
        except ProcessError:
            if process is not None:
                self._terminate_process_safely(process, "Robot Core", timeout=3)
            with self._lock:
                self._robot_core_transition = False
            raise
        except Exception as e:
            if process is not None:
                self._terminate_process_safely(process, "Robot Core", timeout=3)
            with self._lock:
                self._robot_core_transition = False
            raise ProcessError(f"Failed to start robot core: {e}")

        with self._lock:
            self._robot_core_process = process
            self._robot_core_running = True
            self._robot_core_transition = False
            self._crash_info.pop('robot_core', None)

    def stop_robot_core(self) -> None:
        with self._lock:
            if not self._robot_core_running:
                raise ProcessError("Robot core is not running")
            process = self._robot_core_process
            self._robot_core_process = None
            self._robot_core_running = False
            self._robot_core_transition = True
        try:
            if process and not self._terminate_process_safely(process, "Robot Core"):
                logger.error("Robot Core process may still be running")
        finally:
            with self._lock:
                self._robot_core_transition = False

    # --- Health Monitor ---
    def start_health_monitor(self):
        if self._health_thread is not None:
            return
        self._health_stop_event.clear()
        self._health_thread = threading.Thread(target=self._health_check_loop, daemon=True)
        self._health_thread.start()

    def stop_health_monitor(self):
        self._health_stop_event.set()
        if self._health_thread:
            self._health_thread.join(timeout=3)
            self._health_thread = None

    def _health_check_loop(self):
        while not self._health_stop_event.is_set():
            crashed = self._check_processes()
            # 善後 callback 必須在鎖外執行
            if 'navigation' in crashed and self.on_navigation_down is not None:
                try:
                    self.on_navigation_down()
                except Exception as e:
                    logger.error(f"Navigation-down handler failed: {e}")
            self._health_stop_event.wait(self.HEALTH_CHECK_INTERVAL)

    def _check_processes(self) -> list:
        crashed = []
        now = time.strftime('%Y-%m-%dT%H:%M:%S')
        with self._lock:
            if self._slam_process and self._slam_status == SlamStatus.MAPPING:
                ret = self._slam_process.poll()
                if ret is not None:
                    self._crash_info['slam'] = {'exit_code': ret, 'time': now}
                    self._slam_process = None
                    self._slam_status = SlamStatus.IDLE
                    crashed.append('slam')

            if self._nav_process and self._nav_status == NavStatus.RUNNING:
                ret = self._nav_process.poll()
                if ret is not None:
                    self._crash_info['navigation'] = {'exit_code': ret, 'time': now}
                    self._nav_process = None
                    self._nav_status = NavStatus.IDLE
                    crashed.append('navigation')

            if self._robot_core_process and self._robot_core_running:
                ret = self._robot_core_process.poll()
                if ret is not None:
                    self._crash_info['robot_core'] = {'exit_code': ret, 'time': now}
                    self._robot_core_process = None
                    self._robot_core_running = False
                    crashed.append('robot_core')
        return crashed

    @property
    def crash_info(self) -> dict:
        with self._lock:
            return self._crash_info.copy()

    def cleanup(self):
        logger.info("Starting cleanup of all processes...")
        self.stop_health_monitor()
        with self._lock:
            processes = [
                (self._slam_process, "SLAM"),
                (self._nav_process, "Navigation"),
                (self._robot_core_process, "Robot Core"),
            ]
            self._slam_process = None
            self._nav_process = None
            self._robot_core_process = None
            self._slam_status = SlamStatus.IDLE
            self._nav_status = NavStatus.IDLE
            self._robot_core_running = False
            self._robot_core_transition = False
        for process, name in processes:
            if process:
                self._terminate_process_safely(process, name, timeout=3)
        logger.info("Cleanup completed")


class NavigatorManager:
    """BasicNavigator 的生命週期管理（執行緒安全）"""

    def __init__(self):
        self.navigator = None
        self._nav2_ready = False
        self._lock = threading.Lock()
        # 序列化就緒檢查：避免多個執行緒同時對同一個 navigator node spin
        self._ready_lock = threading.Lock()

    @property
    def is_ready(self) -> bool:
        with self._lock:
            return self._nav2_ready and self.navigator is not None

    def ensure_nav2_ready(self) -> bool:
        if not NAV2_AVAILABLE:
            return False
        with self._lock:
            if self._nav2_ready and self.navigator is not None:
                return True

        # 慢路徑整段以 _ready_lock 序列化（併發 spin 同一個 node 會拋錯）
        with self._ready_lock:
            with self._lock:
                if self._nav2_ready and self.navigator is not None:
                    return True
                if self.navigator is None:
                    if not ensure_rclpy_initialized():
                        logger.error("Failed to initialize rclpy")
                        return False
                    logger.info("Creating BasicNavigator...")
                    try:
                        self.navigator = BasicNavigator()
                    except Exception as e:
                        logger.error(f"Failed to create BasicNavigator: {e}")
                        return False
                nav = self.navigator

            # 就緒判定：刻意「不」使用 BasicNavigator.waitUntilNav2Active()。
            #
            # 該函式的 _waitForInitialPose() 有兩個會直接毀掉定位的行為：
            #   1. 迴圈中不斷呼叫 _setInitialPose()，發布的是 BasicNavigator 自己
            #      的 initial_pose——未經 setInitialPose() 設定時是全零、frame_id
            #      為空字串的訊息，會把使用者手動指定的位姿覆蓋掉。
            #   2. 它等待 /amcl_pose，而 AMCL 只在濾波器更新時才發布，更新又需要
            #      機器人移動超過 update_min_d（0.25 m）。靜止的機器人永遠等不到，
            #      於是無限迴圈。
            #
            # 改為直接檢查真正的就緒條件：bt_navigator 已 active、且 map→odom
            # 存在（後者是 AMCL 完成定位的唯一可靠證據）。
            deadline = time.time() + NAV2_READY_TIMEOUT_SEC

            if not self._wait_node_active('bt_navigator', deadline):
                logger.error("bt_navigator 未在時限內進入 active")
                return False

            while time.time() < deadline:
                if bridge.is_localized():
                    break
                time.sleep(0.3)
            else:
                logger.error(
                    "Nav2 已啟動但 AMCL 未定位（map→odom 不存在）；"
                    "請先在前端指定機器人在地圖上的實際位置"
                )
                return False

            with self._lock:
                if self.navigator is not nav:
                    return False  # 等待期間被 reset
                self._nav2_ready = True
            logger.info("Nav2 已就緒")
            return True

    @staticmethod
    def _wait_node_active(node_name: str, deadline: float) -> bool:
        """輪詢 lifecycle 狀態直到 active 或逾時。

        用 ros2 CLI 而非直接建 service client，避免與 BasicNavigator 的
        executor 競用同一個 node。
        """
        while time.time() < deadline:
            try:
                out = subprocess.run(
                    ["ros2", "lifecycle", "get", f"/{node_name}"],
                    capture_output=True, text=True, timeout=5,
                )
                if 'active' in out.stdout:
                    return True
            except Exception:
                pass
            time.sleep(1.0)
        return False

    def reset(self):
        """導航進程停止或崩潰後呼叫。

        不使用 lifecycleShutdown()——它會對 nav2 lifecycle 服務發請求，
        在 nav2 已死亡時可能無限期阻塞；進程本身由 RobotStateManager 終止。
        """
        with self._lock:
            self._nav2_ready = False
            nav = self.navigator
            self.navigator = None
        if nav is not None:
            try:
                nav.destroy_node()
            except Exception as e:
                logger.warning(f"Error destroying navigator node: {e}")

    def send_goal(self, x_m: float, y_m: float, yaw_rad: float) -> None:
        with self._lock:
            if self.navigator is None:
                raise RuntimeError("Navigator not initialized")
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = x_m
            goal_pose.pose.position.y = y_m
            qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)
            goal_pose.pose.orientation.z = qz
            goal_pose.pose.orientation.w = qw
            logger.info(f"Sending goal: x={x_m:.3f}, y={y_m:.3f}, yaw={math.degrees(yaw_rad):.1f}")
            self.navigator.goToPose(goal_pose)

    def is_task_complete(self) -> bool:
        with self._lock:
            if self.navigator is None or not self._nav2_ready:
                return True
            try:
                return self.navigator.isTaskComplete()
            except Exception as e:
                # 無法判定時回 False，避免被誤判為「已完成」而錯誤推進狀態機
                logger.warning(f"Task complete check failed: {e}")
                return False

    def get_result(self):
        with self._lock:
            if self.navigator is None or not self._nav2_ready:
                return None
            try:
                return self.navigator.getResult()
            except Exception as e:
                logger.debug(f"Get result failed: {e}")
                return None

    def cancel_task(self):
        with self._lock:
            # 未就緒時沒有可取消的目標；且此時可能有執行緒正在
            # 就緒檢查與 cancel 會 spin 同一個 node，不可併發
            if self.navigator is not None and self._nav2_ready:
                try:
                    self.navigator.cancelTask()
                except Exception as e:
                    logger.warning(f"Cancel task failed: {e}")

    def result_to_event_code(self, result) -> EventCode:
        if not NAV2_AVAILABLE or result is None:
            return EventCode.ABORT
        if result == TaskResult.SUCCEEDED:
            return EventCode.COMPLETE
        if result == TaskResult.CANCELED:
            return EventCode.ABORT
        return EventCode.STUCK


class MissionTracker:
    """導航任務的世代握手。

    狀態推進（begin）到 goal 實際送出（dispatched）之間，
    ``isTaskComplete()`` 反映的是上一段航程的結果，不得據此判定完成。
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._kind: Optional[str] = None       # "point" | "charging" | None
        self._generation = 0
        self._awaiting_goal = False

    def begin(self, kind: str) -> int:
        with self._lock:
            self._generation += 1
            self._kind = kind
            self._awaiting_goal = True
            return self._generation

    def dispatched(self, generation: int) -> None:
        with self._lock:
            if generation == self._generation:
                self._awaiting_goal = False

    def abort(self, generation: Optional[int] = None) -> None:
        with self._lock:
            if generation is not None and generation != self._generation:
                return
            self._kind = None
            self._awaiting_goal = False

    def snapshot(self) -> Tuple[Optional[str], int, bool]:
        with self._lock:
            return self._kind, self._generation, self._awaiting_goal

    def finish(self, generation: int) -> Optional[str]:
        """把任務標記為完成，回傳其 kind（若世代已過期則回 None）"""
        with self._lock:
            if generation != self._generation or self._kind is None:
                return None
            kind = self._kind
            self._kind = None
            self._awaiting_goal = False
            return kind

    @property
    def active(self) -> bool:
        with self._lock:
            return self._kind is not None


class RosBridge:
    """單一常駐 ROS 節點：電壓 / e-stop / 即時地圖訂閱、cmd_vel 與 initialpose 發布。

    以獨立的 SingleThreadedExecutor 在背景執行緒 spin，
    避免與 BasicNavigator 的 spin 衝突。
    """

    NODE_NAME = 'robot_api_bridge'

    def __init__(self):
        self._lock = threading.Lock()
        self._node = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        self._voltage: Optional[float] = None
        self._e_stop = False
        self._latest_map = None
        self._scan_count = 0  # 就緒探測用：確認 /scan 確實在發布
        self._latest_pose: Optional[Tuple[float, float, float]] = None  # (x_m, y_m, yaw_rad)

        self._cmd_vel_pub = None
        self._initialpose_pub = None
        self._tf_buffer = None
        self._tf_listener = None
        self._manual_twist = (0.0, 0.0)  # (linear, angular)

    # --- 生命週期 ---
    def start(self) -> bool:
        if not ROS_AVAILABLE:
            logger.warning("ROS unavailable; RosBridge not started (degraded mode)")
            return False
        if self._thread is not None:
            return True
        if not ensure_rclpy_initialized():
            return False
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._spin_loop, daemon=True)
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_event.set()
        executor = self._executor
        if executor is not None:
            try:
                executor.shutdown()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=3)
            self._thread = None

    def _spin_loop(self) -> None:
        try:
            node = rclpy.create_node(self.NODE_NAME)
            sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
            latched_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )

            node.create_subscription(Float32, '/motor/voltage', self._on_voltage, sensor_qos)
            node.create_subscription(Bool, '/e_stop', self._on_e_stop, latched_qos)
            node.create_subscription(OccupancyGrid, '/map', self._on_map, latched_qos)

            self._cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
            self._initialpose_pub = node.create_publisher(
                PoseWithCovarianceStamped, '/initialpose', 10
            )

            # TF：map → base_link 在建圖與導航模式下都可用
            try:
                from tf2_ros import Buffer, TransformListener
                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, node)
            except Exception as e:
                logger.warning(f"tf2_ros unavailable, pose will fall back to AMCL topic: {e}")
                node.create_subscription(
                    PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl_pose, 10
                )

            # /scan 只用來確認雷射確實在發布（就緒探測用），不保留內容。
            # 感測器資料是 BEST_EFFORT，QoS 必須相容否則收不到。
            try:
                from sensor_msgs.msg import LaserScan
                from rclpy.qos import qos_profile_sensor_data
                node.create_subscription(
                    LaserScan, '/scan', self._on_scan, qos_profile_sensor_data
                )
            except Exception as e:  # pragma: no cover
                logger.warning(f"無法訂閱 /scan（就緒探測將略過此項）: {e}")

            period = 1.0 / max(1.0, MANUAL_PUBLISH_HZ)
            node.create_timer(period, self._publish_manual_twist)

            self._node = node
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(node)
            logger.info("RosBridge node started")
            self._executor.spin()
        except Exception as e:
            if self._stop_event.is_set():
                logger.info("RosBridge node stopped")
            else:
                logger.error(f"RosBridge thread failed: {e!r}")
        finally:
            try:
                if self._node is not None:
                    self._node.destroy_node()
            except Exception:
                pass
            self._node = None

    # --- 訂閱 callback ---
    def _on_voltage(self, msg) -> None:
        with self._lock:
            self._voltage = float(msg.data)

    def _on_e_stop(self, msg) -> None:
        with self._lock:
            self._e_stop = bool(msg.data)
        state.e_stop_active = bool(msg.data)

    def _on_map(self, msg) -> None:
        with self._lock:
            self._latest_map = msg

    def _on_scan(self, msg) -> None:
        # 只計數，不保留內容——就緒探測只需要知道雷射有沒有在發布
        with self._lock:
            self._scan_count += 1

    def _on_amcl_pose(self, msg) -> None:
        q = msg.pose.pose.orientation
        with self._lock:
            self._latest_pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                quaternion_to_yaw(q.x, q.y, q.z, q.w),
            )

    # --- 手動移動 ---
    def set_manual_direction(self, direction: Direction) -> None:
        """設定持續發布的 cmd_vel。

        馬達端有 1 秒 watchdog，因此以定時器持續發布，直到 direction=stop。
        """
        if direction == Direction.FORWARD:
            twist = (MANUAL_LINEAR_SPEED, 0.0)
        elif direction == Direction.BACKWARD:
            twist = (-MANUAL_LINEAR_SPEED, 0.0)
        elif direction == Direction.LEFT:      # 逆時針
            twist = (0.0, MANUAL_ANGULAR_SPEED)
        elif direction == Direction.RIGHT:     # 順時針
            twist = (0.0, -MANUAL_ANGULAR_SPEED)
        else:
            twist = (0.0, 0.0)
        with self._lock:
            self._manual_twist = twist
        # stop 立即送一次零速，不等定時器
        if twist == (0.0, 0.0):
            self._publish_twist(0.0, 0.0)

    def _publish_manual_twist(self) -> None:
        with self._lock:
            linear, angular = self._manual_twist
        if linear == 0.0 and angular == 0.0:
            return
        self._publish_twist(linear, angular)

    def _publish_twist(self, linear: float, angular: float) -> None:
        pub = self._cmd_vel_pub
        if pub is None:
            return
        try:
            msg = Twist()
            msg.linear.x = float(linear)
            msg.angular.z = float(angular)
            pub.publish(msg)
        except Exception as e:
            logger.debug(f"Failed to publish cmd_vel: {e}")

    def stop_motion(self) -> None:
        self.set_manual_direction(Direction.STOP)

    # --- 讀取狀態 ---
    def battery(self) -> int:
        with self._lock:
            voltage = self._voltage
        if voltage is None:
            return 0
        return voltage_to_battery(voltage, BATTERY_MIN_V, BATTERY_MAX_V)

    def voltage(self) -> Optional[float]:
        with self._lock:
            return self._voltage

    def pose(self) -> Optional[Tuple[float, float, float]]:
        """回傳 (x_m, y_m, yaw_rad)，取不到時回 None"""
        buffer_ = self._tf_buffer
        if buffer_ is not None:
            try:
                from rclpy.time import Time
                tf = buffer_.lookup_transform('map', 'base_link', Time())
                t = tf.transform.translation
                q = tf.transform.rotation
                return (t.x, t.y, quaternion_to_yaw(q.x, q.y, q.z, q.w))
            except Exception:
                pass
        with self._lock:
            return self._latest_pose

    # --- 導航就緒探測 ---
    def is_localized(self) -> bool:
        """AMCL 是否已完成定位（以 map→odom 是否存在為準）。

        這是唯一可靠的判準：AMCL 會在收到初始位姿後記錄 "Setting pose"，
        但那只代表訊息被收下，不代表粒子濾波器已更新並開始發布轉換。
        """
        buffer_ = self._tf_buffer
        if buffer_ is None:
            return False
        try:
            from rclpy.time import Time
            return buffer_.can_transform('map', 'odom', Time())
        except Exception:
            return False

    def scan_is_flowing(self, min_hz: float = 3.0, window: float = 2.0) -> bool:
        """/scan 是否穩定在發布。AMCL 沒有掃描就不會更新濾波器。"""
        with self._lock:
            count0 = self._scan_count
        time.sleep(window)
        with self._lock:
            count1 = self._scan_count
        return (count1 - count0) / window >= min_hz

    def wait_for_navigation_ready(
        self,
        settle_sec: float = 2.0,
        probe_timeout: float = NAV2_READY_TIMEOUT_SEC,
    ) -> Tuple[bool, str]:
        """啟動導航後的就緒探測，回傳 (是否就緒, 說明)。

        用 ROS 狀態當判準而非固定 sleep——固定 sleep 在 Jetson 上不可靠，
        且失敗時無法分辨卡在哪一步。順序刻意與 Nav2 的相依關係一致：

          1. /map 已收到          （AMCL 沒有地圖不會處理掃描）
          2. /scan 穩定發布       （沒有掃描濾波器不會更新）
          3. TF buffer 沉澱       （剛啟動時 buffer 是空的，查詢會失敗）
          4. 發布初始位姿          （此時 AMCL 的訂閱必然已建立）
          5. 等待 map→odom 出現   （唯一能證明定位真的成功的訊號）
        """
        deadline = time.time() + probe_timeout

        # 1. 地圖
        while time.time() < deadline:
            with self._lock:
                if self._latest_map is not None:
                    break
            time.sleep(0.3)
        else:
            return False, "逾時：未收到 /map，map_server 可能未啟動或地圖檔無效"

        # 2. 掃描
        if not self.scan_is_flowing():
            return False, "逾時：/scan 未穩定發布，LiDAR 可能未連線"

        # 3. 讓 TF buffer 累積足夠歷史，否則 AMCL 的 odom 查詢會失敗
        time.sleep(settle_sec)

        # 4. 已經定位就不必再送（例如重複呼叫）
        if self.is_localized():
            return True, "已完成定位"

        # 5. 發布初始位姿並等待定位生效
        self.publish_initial_pose(0.0, 0.0, 0.0)
        while time.time() < deadline:
            if self.is_localized():
                return True, "定位完成（初始位姿設於地圖原點）"
            time.sleep(0.5)

        return False, (
            "逾時：已送出初始位姿但 AMCL 未發布 map→odom。"
            "最可能的原因是機器人目前的實際位置與地圖原點差距過大，"
            "掃描無法與地圖匹配——請在前端手動指定機器人在地圖上的實際位置。"
        )

    def location(self) -> Optional[Location]:
        """回傳 API 單位的 Location（公分整數 + 度）"""
        pose = self.pose()
        if pose is None:
            return None
        x_m, y_m, yaw = pose
        return Location(x=m_to_cm(x_m), y=m_to_cm(y_m), orientation=yaw_to_deg(yaw))

    # --- 即時地圖（🟡 /maps/live/*）---
    def live_map_metadata(self) -> Optional[dict]:
        with self._lock:
            grid = self._latest_map
        if grid is None:
            return None
        info = grid.info
        return {
            "resolution": float(info.resolution),
            "origin": [
                float(info.origin.position.x),
                float(info.origin.position.y),
                float(quaternion_to_yaw(
                    info.origin.orientation.x, info.origin.orientation.y,
                    info.origin.orientation.z, info.origin.orientation.w,
                )),
            ],
            "width": int(info.width),
            "height": int(info.height),
            "negate": 0,
            "occupied_thresh": 0.65,
            "free_thresh": 0.196,
        }

    def live_map_png(self) -> Optional[bytes]:
        """把最新的 /map OccupancyGrid 轉成 PNG（與 map_server 的 pgm 慣例一致）"""
        if not PIL_AVAILABLE:
            return None
        with self._lock:
            grid = self._latest_map
        if grid is None:
            return None
        width, height = int(grid.info.width), int(grid.info.height)
        if width <= 0 or height <= 0:
            return None

        # occupancy: -1 未知 → 205、0 自由 → 254、100 佔據 → 0
        pixels = bytearray(width * height)
        data = grid.data
        for i, value in enumerate(data):
            if value < 0:
                pixels[i] = 205
            elif value >= 65:
                pixels[i] = 0
            elif value <= 25:
                pixels[i] = 254
            else:
                pixels[i] = 205

        try:
            img = Image.frombytes('L', (width, height), bytes(pixels))
            # OccupancyGrid 的 row 0 在下方，影像慣例是上方
            img = img.transpose(Image.FLIP_TOP_BOTTOM)
            buf = io.BytesIO()
            img.save(buf, format='PNG')
            return buf.getvalue()
        except Exception as e:
            logger.error(f"Failed to render live map: {e}")
            return None

    # --- initialpose ---
    def publish_initial_pose(
        self,
        x_m: float,
        y_m: float,
        yaw_rad: float,
        cov_xy: float = 0.25,
        cov_yaw: float = 0.06853891945200942,
        timeout: float = 10.0,
    ) -> bool:
        """發布 /initialpose 給 AMCL。

        以 ``get_subscription_count()`` 輪詢確認 AMCL 已訂閱後才發布，
        避免 DDS discovery 未完成導致訊息遺失。回傳是否確認有訂閱者。
        """
        if not ROS_AVAILABLE:
            return False
        pub = self._initialpose_pub
        node = self._node
        owns_node = False
        if pub is None or node is None:
            # bridge 尚未啟動：臨時建一個 node（名稱加亂數避免併發同名）
            if not ensure_rclpy_initialized():
                return False
            node = rclpy.create_node(f'initial_pose_pub_{uuid4().hex[:8]}')
            pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
            owns_node = True

        try:
            deadline = time.time() + timeout
            while pub.get_subscription_count() == 0 and time.time() < deadline:
                time.sleep(0.1)
            has_subscriber = pub.get_subscription_count() > 0
            if not has_subscriber:
                logger.warning(f"No subscriber on /initialpose after {timeout:.1f}s")

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x = float(x_m)
            msg.pose.pose.position.y = float(y_m)
            msg.pose.pose.position.z = 0.0
            _, _, qz, qw = yaw_to_quaternion(yaw_rad)
            msg.pose.pose.orientation.z = qz
            msg.pose.pose.orientation.w = qw
            msg.pose.covariance[0] = cov_xy
            msg.pose.covariance[7] = cov_xy
            msg.pose.covariance[35] = cov_yaw

            pub.publish(msg)
            time.sleep(0.3)  # 給 DDS 傳輸時間
            logger.info(f"Published initial pose: x={x_m:.3f}, y={y_m:.3f}, yaw={yaw_rad:.3f}")
            return has_subscriber
        finally:
            if owns_node:
                try:
                    node.destroy_node()
                except Exception:
                    pass


# --- 全域實例 ---
state = RobotStateManager()
nav_manager = NavigatorManager()
mission = MissionTracker()
bridge = RosBridge()


def handle_navigation_down() -> None:
    """導航停止或崩潰後的共用善後"""
    try:
        nav_manager.reset()
    except Exception as e:
        logger.warning(f"Navigator reset failed: {e}")
    mission.abort()


state.on_navigation_down = handle_navigation_down


# --- 對 router 的高階介面 ---
def robot_status() -> RobotStatus:
    """依子系統狀態推導規格 §7.2 的 Robot Status"""
    if state.is_busy:
        return RobotStatus.SWITCHING_MODE
    if state.slam_status != SlamStatus.IDLE:
        return RobotStatus.IDLE
    if state.nav_status != NavStatus.RUNNING:
        return RobotStatus.INIT
    kind, _, _ = mission.snapshot()
    if kind == 'charging':
        return RobotStatus.GO_CHARGING
    if kind == 'point':
        return RobotStatus.MOVING
    return RobotStatus.IDLE


def current_map() -> Optional[str]:
    return state.current_map


class Nav2NotReadyError(RuntimeError):
    """Nav2 未就緒（通常是 AMCL 尚未定位）。與其他導航失敗區分，
    讓端點能回傳語意正確的錯誤碼而非 ROBOT_BUSY。"""


def navigate_to(location: Location, kind: str = 'point') -> int:
    """送出導航目標（阻塞，須以 asyncio.to_thread 包裝）。回傳 mission 世代。"""
    generation = mission.begin(kind)
    if not nav_manager.ensure_nav2_ready():
        mission.abort(generation)
        raise Nav2NotReadyError(
            "Nav2 未就緒；請先設定初始位姿（relocate）讓 AMCL 完成定位"
        )
    try:
        nav_manager.send_goal(
            cm_to_m(location.x), cm_to_m(location.y), deg_to_yaw(location.orientation)
        )
    except Exception:
        mission.abort(generation)
        raise
    mission.dispatched(generation)
    return generation


def stop_motion() -> None:
    """軟停止：取消導航目標並歸零 cmd_vel"""
    mission.abort()
    bridge.stop_motion()
    nav_manager.cancel_task()


def save_map(map_name: str) -> None:
    """用 nav2_map_server 存下目前建圖結果（阻塞）"""
    map_path = os.path.join(MAP_PATH, map_name)
    cmd = (
        f"source /opt/ros/humble/setup.bash && "
        f"source {WORKSPACE_ROOT}/install/setup.bash && "
        f"ros2 run nav2_map_server map_saver_cli -f {map_path} -t /map_saver "
        f"--ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=10000.0"
    )
    state.begin_map_save()
    try:
        result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=30)
        if result.returncode != 0:
            raise ProcessError(f"Map save failed: {result.stderr.strip()[:200]}")
    except subprocess.TimeoutExpired:
        raise ProcessError("Map save timed out")
    finally:
        state.end_map_save()


def shutdown_system() -> None:
    """執行系統關機（POST /shutdown，事件先送）"""
    try:
        subprocess.Popen(["sudo", "shutdown", "-h", "now"])
    except Exception as e:
        logger.error(f"Failed to shut down: {e}")
        raise ProcessError(str(e))
