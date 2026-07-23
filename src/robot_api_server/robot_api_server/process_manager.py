"""SLAM / Navigation / Robot core 子程序的執行緒安全生命週期管理。

拆分自 ``ros_bridge.py``，逐字搬移，並發模型（``threading.Lock`` 用法、
鎖邊界）與行為完全不變：鎖內只做「狀態檢查 + 過渡狀態轉移」，
spawn/清理/等待等慢操作一律在鎖外。
"""

import os
import signal
import subprocess
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, Optional

from .config import MAP_PATH
from .logging_config import get_logger
from .models import OpMode

logger = get_logger(__name__)


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


@dataclass
class ManagedProcess:
    """單一子程序（SLAM / Navigation / Robot Core）的 Popen 控制代碼容器。

    純值物件、不持有自己的鎖：``process`` 欄位的所有讀寫仍完全由
    ``RobotStateManager.self._lock`` 保護，時機與重構前逐字相同，這裡
    只是把「三份 Popen 控制代碼」搬到各自的容器裡，取代原本
    ``self._slam_process`` / ``self._nav_process`` / ``self._robot_core_process``
    三個平行欄位。

    ``terminate_safely`` / ``verify_started`` 是從原本
    ``RobotStateManager._terminate_process_safely`` /
    ``._verify_process_started`` 逐字搬移過來的無鎖工具方法：操作對象是
    呼叫端傳入、尚未寫回 ``self.process`` 的本地 Popen 變數，鎖語意不變。
    """

    name: str
    process: Optional[subprocess.Popen] = None

    def terminate_safely(self, process: subprocess.Popen, timeout: int = 5) -> bool:
        if process is None:
            return True
        try:
            pgid = os.getpgid(process.pid)
            logger.info(f"Sending SIGTERM to {self.name} (pgid={pgid})")
            os.killpg(pgid, signal.SIGTERM)
            try:
                process.wait(timeout=timeout)
                logger.info(f"{self.name} terminated gracefully")
                return True
            except subprocess.TimeoutExpired:
                pass
            logger.warning(f"{self.name} did not terminate, sending SIGKILL")
            os.killpg(pgid, signal.SIGKILL)
            try:
                process.wait(timeout=3)
                return True
            except subprocess.TimeoutExpired:
                logger.error(f"Failed to kill {self.name}")
                return False
        except ProcessLookupError:
            return True
        except Exception as e:
            logger.error(f"Error terminating {self.name}: {e}")
            return False

    def verify_started(self, process: subprocess.Popen, wait_time: float = 0.5) -> bool:
        try:
            time.sleep(wait_time)
            exit_code = process.poll()
            if exit_code is not None:
                logger.error(f"{self.name} process exited immediately with code {exit_code}")
                return False
            return True
        except Exception as e:
            logger.error(f"Error verifying {self.name} process: {e}")
            return False


class RobotStateManager:
    """SLAM / Navigation / Robot core 子程序的執行緒安全管理器。

    對外可觀察行為（狀態轉移、e_stop、health monitor、crash info、
    current_map…）與鎖語意（鎖內只做狀態轉移、spawn/terminate 等慢操作
    一律在鎖外）與重構前完全相同；三個子程序各自的 ``Popen`` 控制代碼
    現在委由 ``ManagedProcess`` 值物件持有，``RobotStateManager`` 扮演
    協調三個 ``ManagedProcess`` 實例的 orchestrator。
    """

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
        self._slam = ManagedProcess(name="SLAM")
        self._slam_status = SlamStatus.IDLE
        self._nav = ManagedProcess(name="Navigation")
        self._nav_status = NavStatus.IDLE
        self._current_map: Optional[str] = None
        self._robot_core = ManagedProcess(name="Robot Core")
        self._robot_core_running = False
        self._robot_core_transition = False
        self._e_stop_active = False
        self._health_thread: Optional[threading.Thread] = None
        self._health_stop_event = threading.Event()
        self._crash_info: Dict[str, Dict[str, Any]] = {}
        # 導航停止/崩潰時的善後 callback（在鎖外呼叫）
        self.on_navigation_down: Optional[Callable[[], None]] = None

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

    def _spawn_managed(
        self,
        managed: "ManagedProcess",
        *,
        precheck_and_transition: Callable[[], Any],
        prepare: Callable[[Any], list],
        verify_failed_message: str,
        on_rollback: Callable[[], None],
        start_failed_prefix: str,
        on_commit: Callable[[subprocess.Popen], Any],
    ) -> Any:
        """三個 start_* 共用骨架：鎖內檢查/過渡 → 鎖外 spawn/verify →
        失敗鎖內回滾並重丟例外 → 成功鎖內提交狀態。

        鎖的邊界與原本逐字相同：只有 ``precheck_and_transition``、
        ``on_rollback``、``on_commit`` 在 ``self._lock`` 保護下執行；
        ``prepare``（跨子系統呼叫、cleanup_processes、map 檢查等）與
        ``subprocess.Popen`` / ``verify_started`` 一律在鎖外執行，順序
        與重構前完全一致。
        """
        with self._lock:
            context = precheck_and_transition()

        process: Optional[subprocess.Popen] = None
        try:
            argv = prepare(context)
            process = subprocess.Popen(
                argv,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            if not managed.verify_started(process):
                raise ProcessError(verify_failed_message)
        except ProcessError:
            if process is not None:
                managed.terminate_safely(process, timeout=3)
            with self._lock:
                on_rollback()
            raise
        except Exception as e:
            if process is not None:
                managed.terminate_safely(process, timeout=3)
            with self._lock:
                on_rollback()
            raise ProcessError(f"{start_failed_prefix}: {e}")

        with self._lock:
            return on_commit(process)

    # --- SLAM ---
    def start_slam(self) -> None:
        def precheck_and_transition():
            if self._slam_status != SlamStatus.IDLE:
                raise ProcessError("Mapping is already running or busy")
            if self._nav_status in (NavStatus.STARTING, NavStatus.STOPPING):
                raise ProcessError("Navigation is busy")
            nav_running = self._nav_status == NavStatus.RUNNING
            self._slam_status = SlamStatus.STARTING
            return nav_running

        def prepare(nav_running: bool) -> list:
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
            return ["ros2", "launch", "nav2", "mapping.launch.py"]

        def on_rollback():
            self._slam_status = SlamStatus.IDLE

        def on_commit(process: subprocess.Popen):
            self._slam.process = process
            self._slam_status = SlamStatus.MAPPING
            self._crash_info.pop('slam', None)

        self._spawn_managed(
            self._slam,
            precheck_and_transition=precheck_and_transition,
            prepare=prepare,
            verify_failed_message="SLAM process failed to start",
            on_rollback=on_rollback,
            start_failed_prefix="Failed to start mapping",
            on_commit=on_commit,
        )

    def stop_slam(self) -> None:
        with self._lock:
            if self._slam_status not in (SlamStatus.MAPPING, SlamStatus.SAVING):
                raise ProcessError("Mapping is not running")
            process = self._slam.process
            self._slam.process = None
            self._slam_status = SlamStatus.STOPPING
        try:
            if process and not self._slam.terminate_safely(process):
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
        def precheck_and_transition():
            if self._nav_status != NavStatus.IDLE:
                raise ProcessError("Navigation is already running")
            if self._slam_status != SlamStatus.IDLE:
                slam_running = True
            else:
                slam_running = False
            self._nav_status = NavStatus.STARTING
            return slam_running

        def prepare(slam_running: bool) -> list:
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

            return ["ros2", "launch", "nav2", "autonomous_navigation.launch.py",
                    f"map:={map_yaml}"]

        def on_rollback():
            self._nav_status = NavStatus.IDLE

        def on_commit(process: subprocess.Popen):
            self._nav.process = process
            self._nav_status = NavStatus.RUNNING
            self._crash_info.pop('navigation', None)
            self._current_map = map_name if map_name else "map"
            return self._current_map

        return self._spawn_managed(
            self._nav,
            precheck_and_transition=precheck_and_transition,
            prepare=prepare,
            verify_failed_message="Navigation process failed to start",
            on_rollback=on_rollback,
            start_failed_prefix="Failed to start navigation",
            on_commit=on_commit,
        )

    def stop_navigation(self) -> None:
        with self._lock:
            if self._nav_status != NavStatus.RUNNING:
                raise ProcessError("Navigation is not running")
            process = self._nav.process
            self._nav.process = None
            self._nav_status = NavStatus.STOPPING
        try:
            if process and not self._nav.terminate_safely(process):
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
        def precheck_and_transition():
            if self._robot_core_running or self._robot_core_transition:
                raise ProcessError("Robot core is already running")
            self._robot_core_transition = True

        def prepare(_context) -> list:
            existing = subprocess.run(
                ["pgrep", "-f", "ros2 launch motor_control bringup"], capture_output=True
            )
            if existing.returncode == 0:
                raise ProcessError("Robot core is already running outside this API")
            return ["ros2", "launch", "motor_control", "bringup.launch.py", "enable_web:=false"]

        def on_rollback():
            self._robot_core_transition = False

        def on_commit(process: subprocess.Popen):
            self._robot_core.process = process
            self._robot_core_running = True
            self._robot_core_transition = False
            self._crash_info.pop('robot_core', None)

        self._spawn_managed(
            self._robot_core,
            precheck_and_transition=precheck_and_transition,
            prepare=prepare,
            verify_failed_message="Robot core process failed to start",
            on_rollback=on_rollback,
            start_failed_prefix="Failed to start robot core",
            on_commit=on_commit,
        )

    def stop_robot_core(self) -> None:
        with self._lock:
            if not self._robot_core_running:
                raise ProcessError("Robot core is not running")
            process = self._robot_core.process
            self._robot_core.process = None
            self._robot_core_running = False
            self._robot_core_transition = True
        try:
            if process and not self._robot_core.terminate_safely(process):
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
            if self._slam.process and self._slam_status == SlamStatus.MAPPING:
                ret = self._slam.process.poll()
                if ret is not None:
                    self._crash_info['slam'] = {'exit_code': ret, 'time': now}
                    self._slam.process = None
                    self._slam_status = SlamStatus.IDLE
                    crashed.append('slam')

            if self._nav.process and self._nav_status == NavStatus.RUNNING:
                ret = self._nav.process.poll()
                if ret is not None:
                    self._crash_info['navigation'] = {'exit_code': ret, 'time': now}
                    self._nav.process = None
                    self._nav_status = NavStatus.IDLE
                    crashed.append('navigation')

            if self._robot_core.process and self._robot_core_running:
                ret = self._robot_core.process.poll()
                if ret is not None:
                    self._crash_info['robot_core'] = {'exit_code': ret, 'time': now}
                    self._robot_core.process = None
                    self._robot_core_running = False
                    crashed.append('robot_core')
        return crashed

    @property
    def crash_info(self) -> Dict[str, Dict[str, Any]]:
        with self._lock:
            return self._crash_info.copy()

    def cleanup(self):
        logger.info("Starting cleanup of all processes...")
        self.stop_health_monitor()
        with self._lock:
            managed_processes = [
                (self._slam, self._slam.process),
                (self._nav, self._nav.process),
                (self._robot_core, self._robot_core.process),
            ]
            self._slam.process = None
            self._nav.process = None
            self._robot_core.process = None
            self._slam_status = SlamStatus.IDLE
            self._nav_status = NavStatus.IDLE
            self._robot_core_running = False
            self._robot_core_transition = False
        for managed, process in managed_processes:
            if process:
                managed.terminate_safely(process, timeout=3)
        logger.info("Cleanup completed")


__all__ = [
    'SlamStatus', 'NavStatus', 'ProcessError', 'ManagedProcess', 'RobotStateManager',
]
