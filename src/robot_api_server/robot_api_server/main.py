import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import threading
import subprocess
import os
import signal
import time
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, Response
from PIL import Image
import io
import yaml
from pydantic import BaseModel
import uvicorn
from typing import Optional, Set, List
from enum import Enum
from uuid import uuid4
from datetime import datetime
from contextlib import asynccontextmanager
import asyncio
import json

from .logging_config import setup_logging, get_logger

# --- Logging Setup ---
setup_logging()
logger = get_logger(__name__)

# --- rclpy 全局初始化管理 ---
_rclpy_init_lock = threading.Lock()
_rclpy_initialized = False


def ensure_rclpy_initialized() -> bool:
    """線程安全地確保 rclpy 只初始化一次"""
    global _rclpy_initialized
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
            raise
        except Exception as e:
            logger.error(f"Failed to initialize rclpy: {e}")
            return False

# --- Pydantic Models ---
class Goal(BaseModel):
    x: float
    y: float
    yaw_deg: float

class MapSaveRequest(BaseModel):
    map_name: str

class NavigationStartRequest(BaseModel):
    map_name: Optional[str] = None

class InitialPoseRequest(BaseModel):
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0  # 弧度

class SlamStatus(str, Enum):
    IDLE = "idle"
    STARTING = "starting"  # 過渡狀態：啟動中
    MAPPING = "mapping"
    SAVING = "saving"
    STOPPING = "stopping"  # 過渡狀態：停止中

class NavStatus(str, Enum):
    IDLE = "idle"
    STARTING = "starting"  # 過渡狀態：啟動中
    RUNNING = "running"
    STOPPING = "stopping"  # 過渡狀態：停止中


# --- Waypoint Models ---
class WaypointBase(BaseModel):
    name: str
    x: float
    y: float
    yaw_deg: float


class WaypointCreate(WaypointBase):
    pass


class WaypointUpdate(BaseModel):
    name: Optional[str] = None
    x: Optional[float] = None
    y: Optional[float] = None
    yaw_deg: Optional[float] = None


class Waypoint(WaypointBase):
    id: str
    created_at: str
    updated_at: str


# --- Table Models ---
class TableBase(BaseModel):
    number: int
    name: Optional[str] = None
    x: float
    y: float
    yaw_deg: float
    isActive: bool = True


class TableCreate(TableBase):
    pass


class TableUpdate(BaseModel):
    number: Optional[int] = None
    name: Optional[str] = None
    x: Optional[float] = None
    y: Optional[float] = None
    yaw_deg: Optional[float] = None
    isActive: Optional[bool] = None


class Table(TableBase):
    id: str
    created_at: str
    updated_at: str


# --- Delivery Models ---
class DeliveryStopStatus(str, Enum):
    PENDING = "pending"
    IN_PROGRESS = "in_progress"
    ARRIVED = "arrived"
    COMPLETED = "completed"
    SKIPPED = "skipped"


class DeliveryTaskStatus(str, Enum):
    IDLE = "idle"
    DELIVERING = "delivering"
    AT_TABLE = "at_table"
    RETURNING = "returning"
    STUCK = "stuck"  # 機器人卡住


class DeliveryStop(BaseModel):
    tableId: str
    tableNumber: int
    tableName: Optional[str] = None
    status: DeliveryStopStatus = DeliveryStopStatus.PENDING


class Position(BaseModel):
    x: float
    y: float
    yaw: float


class DeliveryStartRequest(BaseModel):
    tableIds: List[str]
    startPosition: Position
    mapName: Optional[str] = None


class DeliveryTask(BaseModel):
    id: str
    stops: List[DeliveryStop]
    status: DeliveryTaskStatus
    currentStopIndex: int
    startPosition: Position
    createdAt: str


# --- Configuration ---
# 從環境變數讀取配置，提供合理預設值
# ROBOT_WORKSPACE 環境變數優先，否則從當前檔案位置推導
def _get_workspace_root() -> str:
    """取得 workspace 根目錄路徑"""
    # 優先使用環境變數
    env_workspace = os.environ.get('ROBOT_WORKSPACE')
    if env_workspace and os.path.isdir(env_workspace):
        return env_workspace

    # 從當前檔案位置推導: .../src/robot_api_server/robot_api_server/main.py
    # 往上 4 層即為 workspace 根目錄
    current_file = os.path.abspath(__file__)
    workspace = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(current_file))))

    # 驗證路徑有效性（檢查 src 目錄是否存在）
    if os.path.isdir(os.path.join(workspace, 'src')):
        return workspace

    # 最後 fallback（相容舊配置）
    return '/home/robot0/base_dev'

WORKSPACE_ROOT = _get_workspace_root()
DEFAULT_MAP_PATH = os.path.join(WORKSPACE_ROOT, 'map')
MAP_SAVE_PATH = os.environ.get('ROBOT_MAP_PATH', DEFAULT_MAP_PATH)

# 確保地圖目錄存在
os.makedirs(MAP_SAVE_PATH, exist_ok=True)
logger.info(f"Map save path: {MAP_SAVE_PATH}")


# --- Path Validation ---
def validate_map_name(map_name: str) -> str:
    """驗證並返回安全的地圖名稱，防止路徑注入攻擊"""
    # 清理檔名：只允許字母數字和 -_
    safe_name = "".join(c for c in map_name if c.isalnum() or c in ('-', '_'))

    if not safe_name:
        raise HTTPException(status_code=400, detail="Invalid map name after sanitization.")

    # 限制檔名長度
    if len(safe_name) > 64:
        raise HTTPException(status_code=400, detail="Map name too long (max 64 characters).")

    # 構建完整路徑並規範化，驗證沒有目錄遍歷
    full_path = os.path.normpath(os.path.join(MAP_SAVE_PATH, safe_name))
    base_path_normalized = os.path.normpath(MAP_SAVE_PATH)
    if not full_path.startswith(base_path_normalized + os.sep) and full_path != base_path_normalized:
        raise HTTPException(status_code=400, detail="Invalid map path.")

    return safe_name


# --- Waypoint File Operations ---
def get_waypoints_file_path(map_name: str) -> str:
    """取得 waypoints 檔案路徑（使用統一的路徑驗證）"""
    safe_name = validate_map_name(map_name)
    return os.path.join(MAP_SAVE_PATH, f"{safe_name}.waypoints.json")


def load_waypoints(map_name: str) -> List[Waypoint]:
    """載入地圖的 waypoints"""
    file_path = get_waypoints_file_path(map_name)
    if not os.path.exists(file_path):
        return []
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return [Waypoint(**wp) for wp in data]
    except Exception as e:
        logger.error(f"Failed to load waypoints: {e}")
        return []


def save_waypoints(map_name: str, waypoints: List[Waypoint]) -> None:
    """儲存地圖的 waypoints"""
    file_path = get_waypoints_file_path(map_name)
    try:
        data = [wp.model_dump() for wp in waypoints]
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception as e:
        logger.error(f"Failed to save waypoints: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to save waypoints: {str(e)}")


# --- Table File Operations ---
def get_tables_file_path(map_name: str) -> str:
    """取得 tables 檔案路徑"""
    safe_name = validate_map_name(map_name)
    return os.path.join(MAP_SAVE_PATH, f"{safe_name}.tables.json")


def load_tables(map_name: str) -> List[Table]:
    """載入地圖的 tables"""
    file_path = get_tables_file_path(map_name)
    if not os.path.exists(file_path):
        return []
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
        return [Table(**t) for t in data]
    except Exception as e:
        logger.error(f"Failed to load tables: {e}")
        return []


def save_tables(map_name: str, tables: List[Table]) -> None:
    """儲存地圖的 tables"""
    file_path = get_tables_file_path(map_name)
    try:
        data = [t.model_dump() for t in tables]
        with open(file_path, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
    except Exception as e:
        logger.error(f"Failed to save tables: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to save tables: {str(e)}")


# --- Thread-safe State Manager ---
class RobotStateManager:
    """Thread-safe manager for robot state and processes."""

    HEALTH_CHECK_INTERVAL = 2.0  # 每 2 秒檢查一次

    # pkill -f 用的殘留進程清理 pattern。
    # 自己啟動的進程一律優先透過 Popen 的 pgid（_terminate_process_safely）終止，
    # pkill 只用來清理上次 server 異常結束留下的孤兒進程。
    # pattern 必須錨定完整指令或「套件/執行檔」路徑，避免誤殺
    # （例如舊 pattern "nav2_" 會殺掉存圖中的 nav2_map_server/map_saver_cli）。
    NAV_CLEANUP_PATTERNS = [
        "ros2 launch nav2 autonomous_navigation",
        "nav2_map_server/map_server",  # 不會匹配 map_saver_cli
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
        self._current_map: Optional[str] = None  # 當前導航使用的地圖
        self._robot_core_process: Optional[subprocess.Popen] = None
        self._robot_core_running = False
        self._robot_core_transition = False  # 過渡狀態 guard（啟動/停止中）
        self._e_stop_active = False
        # Health monitor
        self._health_thread: Optional[threading.Thread] = None
        self._health_stop_event = threading.Event()
        self._crash_info: dict = {}  # 記錄 crash 資訊
        # 導航停止/崩潰時的善後 callback（在鎖外呼叫；模組載入後由外部注入）
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

    # --- SLAM ---
    @property
    def slam_status(self) -> SlamStatus:
        with self._lock:
            return self._slam_status

    @slam_status.setter
    def slam_status(self, value: SlamStatus):
        with self._lock:
            self._slam_status = value

    @property
    def current_map(self) -> Optional[str]:
        with self._lock:
            return self._current_map

    @current_map.setter
    def current_map(self, value: Optional[str]):
        with self._lock:
            self._current_map = value

    def _wait_for_process_cleanup(self, patterns: list, timeout: float = 5.0) -> bool:
        """等待指定模式的進程完全終止

        Args:
            patterns: 要檢查的進程模式列表
            timeout: 最大等待時間（秒）

        Returns:
            True 如果所有進程都已終止，False 如果超時
        """
        start_time = time.time()
        check_interval = 0.2  # 每 200ms 檢查一次

        while time.time() - start_time < timeout:
            all_terminated = True
            for pattern in patterns:
                try:
                    # pgrep 返回 0 表示找到進程，1 表示沒找到
                    result = subprocess.run(
                        ["pgrep", "-f", pattern],
                        capture_output=True
                    )
                    if result.returncode == 0:
                        all_terminated = False
                        break
                except Exception:
                    pass

            if all_terminated:
                return True

            time.sleep(check_interval)

        return False

    def _cleanup_nav_processes(self):
        """清理導航相關的殘留（孤兒）進程"""
        patterns = self.NAV_CLEANUP_PATTERNS
        for pattern in patterns:
            try:
                subprocess.run(["pkill", "-f", pattern], capture_output=True)
            except Exception as e:
                logger.warning(f"Failed to kill processes matching '{pattern}': {e}")

        # 等待進程真正終止，避免競態條件
        if not self._wait_for_process_cleanup(patterns, timeout=3.0):
            logger.warning("Some navigation processes may still be running after cleanup")

    def _cleanup_slam_processes(self):
        """清理建圖相關的殘留（孤兒）進程"""
        patterns = self.SLAM_CLEANUP_PATTERNS
        for pattern in patterns:
            try:
                subprocess.run(["pkill", "-f", pattern], capture_output=True)
            except Exception as e:
                logger.warning(f"Failed to kill processes matching '{pattern}': {e}")

        # 等待進程真正終止，避免競態條件
        if not self._wait_for_process_cleanup(patterns, timeout=3.0):
            logger.warning("Some SLAM processes may still be running after cleanup")

    def _terminate_process_safely(self, process: subprocess.Popen, name: str, timeout: int = 5) -> bool:
        """安全終止進程，先 SIGTERM 後 SIGKILL"""
        if process is None:
            return True

        try:
            pgid = os.getpgid(process.pid)

            # 第一階段：SIGTERM
            logger.info(f"Sending SIGTERM to {name} (pgid={pgid})")
            os.killpg(pgid, signal.SIGTERM)

            try:
                process.wait(timeout=timeout)
                logger.info(f"{name} terminated gracefully")
                return True
            except subprocess.TimeoutExpired:
                pass

            # 第二階段：SIGKILL
            logger.warning(f"{name} did not terminate, sending SIGKILL")
            os.killpg(pgid, signal.SIGKILL)

            try:
                process.wait(timeout=3)
                logger.info(f"{name} killed forcefully")
                return True
            except subprocess.TimeoutExpired:
                logger.error(f"Failed to kill {name}")
                return False

        except ProcessLookupError:
            logger.info(f"{name} already terminated")
            return True
        except Exception as e:
            logger.error(f"Error terminating {name}: {e}")
            return False

    def _verify_process_started(self, process: subprocess.Popen, name: str, wait_time: float = 0.5) -> bool:
        """驗證進程是否成功啟動並存活"""
        try:
            # 等待短暫時間後檢查進程是否仍在運行
            time.sleep(wait_time)
            exit_code = process.poll()
            if exit_code is not None:
                logger.error(f"{name} process exited immediately with code {exit_code}")
                return False
            return True
        except Exception as e:
            logger.error(f"Error verifying {name} process: {e}")
            return False

    def start_slam(self) -> dict:
        # 鎖內只做狀態檢查與過渡狀態轉移，慢操作（清理/spawn/驗證）在鎖外執行
        with self._lock:
            if self._slam_status != SlamStatus.IDLE:
                raise HTTPException(status_code=400, detail="Mapping is already running or busy.")
            if self._nav_status in (NavStatus.STARTING, NavStatus.STOPPING):
                raise HTTPException(status_code=409, detail="Navigation is busy; try again later.")
            nav_running = self._nav_status == NavStatus.RUNNING
            self._slam_status = SlamStatus.STARTING

        process: Optional[subprocess.Popen] = None
        try:
            # 導航運行中：先走正常停止路徑（以自己持有的 pgid 終止），而非直接 pkill
            if nav_running:
                logger.info("Navigation is running; stopping it before starting SLAM")
                try:
                    self.stop_navigation()
                except HTTPException:
                    pass  # 可能已被其他請求或健康監控搶先停止
                if self.on_navigation_down is not None:
                    try:
                        self.on_navigation_down()
                    except Exception as e:
                        logger.warning(f"Navigation-down handler failed: {e}")

            # 再清理可能殘留的導航孤兒進程
            self._cleanup_nav_processes()

            # 使用 DEVNULL 避免管道緩衝區滿導致死鎖
            process = subprocess.Popen(
                ["ros2", "launch", "nav2", "mapping.launch.py"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )

            # 驗證進程確實啟動成功
            if not self._verify_process_started(process, "SLAM"):
                raise HTTPException(status_code=500, detail="SLAM process failed to start.")
        except HTTPException:
            with self._lock:
                self._slam_status = SlamStatus.IDLE
            raise
        except Exception as e:
            if process is not None:
                self._terminate_process_safely(process, "SLAM", timeout=3)
            with self._lock:
                self._slam_status = SlamStatus.IDLE
            raise HTTPException(status_code=500, detail=f"Failed to start mapping: {str(e)}")

        with self._lock:
            self._slam_process = process
            self._slam_status = SlamStatus.MAPPING
            # 成功重啟後清除舊的 crash 紀錄，避免 /health 永久誤報
            self._crash_info.pop('slam', None)
            return {"message": "Mapping started.", "status": self._slam_status}

    def stop_slam(self) -> dict:
        # 鎖內取得進程所有權並進入過渡狀態，實際終止在鎖外執行
        with self._lock:
            if self._slam_status != SlamStatus.MAPPING:
                raise HTTPException(status_code=400, detail="Mapping is not running.")
            process = self._slam_process
            self._slam_process = None
            self._slam_status = SlamStatus.STOPPING

        try:
            if process:
                if not self._terminate_process_safely(process, "SLAM"):
                    logger.error("SLAM process may still be running")
        finally:
            with self._lock:
                self._slam_status = SlamStatus.IDLE
        return {"message": "Mapping stopped.", "status": SlamStatus.IDLE}

    def begin_map_save(self):
        """原子性地從 MAPPING 進入 SAVING；未在建圖時直接拒絕，避免污染狀態機"""
        with self._lock:
            if self._slam_status != SlamStatus.MAPPING:
                raise HTTPException(status_code=400, detail="Mapping is not running; cannot save map.")
            self._slam_status = SlamStatus.SAVING

    def end_map_save(self):
        """存圖結束，恢復進入 SAVING 前的狀態（MAPPING）"""
        with self._lock:
            if self._slam_status == SlamStatus.SAVING:
                self._slam_status = SlamStatus.MAPPING

    # --- Navigation ---
    @property
    def nav_status(self) -> NavStatus:
        with self._lock:
            return self._nav_status

    def start_navigation(self, map_name: Optional[str] = None) -> dict:
        # 先驗證地圖名稱，防止路徑注入
        if map_name:
            map_name = validate_map_name(map_name)

        # 鎖內只做狀態檢查與過渡狀態轉移，慢操作在鎖外執行
        with self._lock:
            if self._nav_status != NavStatus.IDLE:
                raise HTTPException(status_code=400, detail="Navigation is already running.")

            if self._slam_status != SlamStatus.IDLE:
                raise HTTPException(status_code=400, detail="Cannot start navigation while mapping is running.")

            self._nav_status = NavStatus.STARTING

        process: Optional[subprocess.Popen] = None
        try:
            if map_name:
                map_yaml = os.path.join(MAP_SAVE_PATH, f"{map_name}.yaml")
                if not os.path.exists(map_yaml):
                    raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found.")
            else:
                map_yaml = os.path.join(MAP_SAVE_PATH, "map.yaml")

            # 先清理可能殘留的建圖和導航進程
            self._cleanup_slam_processes()
            self._cleanup_nav_processes()

            # 使用 DEVNULL 避免管道緩衝區滿導致死鎖
            process = subprocess.Popen(
                ["ros2", "launch", "nav2", "autonomous_navigation.launch.py", f"map:={map_yaml}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )

            # 驗證進程確實啟動成功
            if not self._verify_process_started(process, "Navigation"):
                raise HTTPException(status_code=500, detail="Navigation process failed to start.")
        except HTTPException:
            with self._lock:
                self._nav_status = NavStatus.IDLE
            raise
        except Exception as e:
            if process is not None:
                self._terminate_process_safely(process, "Navigation", timeout=3)
            with self._lock:
                self._nav_status = NavStatus.IDLE
            raise HTTPException(status_code=500, detail=f"Failed to start navigation: {str(e)}")

        with self._lock:
            self._nav_process = process
            self._nav_status = NavStatus.RUNNING
            # 成功重啟後清除舊的 crash 紀錄，避免 /health 永久誤報
            self._crash_info.pop('navigation', None)
            # 記錄當前使用的地圖名稱
            self._current_map = map_name if map_name else "map"
            return {"message": f"Navigation started with map: {map_yaml}", "status": self._nav_status}

    def stop_navigation(self) -> dict:
        # 鎖內取得進程所有權並進入過渡狀態，實際終止在鎖外執行
        with self._lock:
            if self._nav_status != NavStatus.RUNNING:
                raise HTTPException(status_code=400, detail="Navigation is not running.")
            process = self._nav_process
            self._nav_process = None
            self._nav_status = NavStatus.STOPPING

        try:
            if process:
                if not self._terminate_process_safely(process, "Navigation"):
                    logger.error("Navigation process may still be running")
        finally:
            with self._lock:
                self._nav_status = NavStatus.IDLE
        return {"message": "Navigation stopped.", "status": NavStatus.IDLE}

    # --- Robot Core ---
    @property
    def robot_core_running(self) -> bool:
        with self._lock:
            return self._robot_core_running

    def start_robot_core(self) -> dict:
        # 鎖內只做狀態檢查與過渡狀態轉移，慢操作在鎖外執行
        with self._lock:
            if self._robot_core_running or self._robot_core_transition:
                raise HTTPException(status_code=400, detail="Robot core is already running.")
            self._robot_core_transition = True

        process: Optional[subprocess.Popen] = None
        try:
            # 偵測系統上既有的 bringup（例如 systemd robot-core.service 已在跑），
            # 避免啟動第二份互搶 /dev/motor 等硬體資源
            existing = subprocess.run(
                ["pgrep", "-f", "ros2 launch motor_control bringup"],
                capture_output=True
            )
            if existing.returncode == 0:
                raise HTTPException(
                    status_code=409,
                    detail="Robot core is already running outside this API "
                           "(e.g. managed by systemd robot-core.service). "
                           "Stop it there before starting via API."
                )

            # 使用 DEVNULL 避免管道緩衝區滿導致死鎖
            process = subprocess.Popen(
                ["ros2", "launch", "motor_control", "bringup.launch.py", "enable_web:=false"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid
            )

            # 驗證進程確實啟動成功
            if not self._verify_process_started(process, "Robot Core"):
                raise HTTPException(status_code=500, detail="Robot core process failed to start.")
        except HTTPException:
            with self._lock:
                self._robot_core_transition = False
            raise
        except Exception as e:
            if process is not None:
                self._terminate_process_safely(process, "Robot Core", timeout=3)
            with self._lock:
                self._robot_core_transition = False
            raise HTTPException(status_code=500, detail=f"Failed to start robot core: {str(e)}")

        with self._lock:
            self._robot_core_process = process
            self._robot_core_running = True
            self._robot_core_transition = False
            # 成功重啟後清除舊的 crash 紀錄，避免 /health 永久誤報
            self._crash_info.pop('robot_core', None)
            return {"message": "Robot core started.", "is_running": True}

    def stop_robot_core(self) -> dict:
        # 鎖內取得進程所有權並進入過渡狀態，實際終止在鎖外執行
        with self._lock:
            if not self._robot_core_running:
                raise HTTPException(status_code=400, detail="Robot core is not running.")
            process = self._robot_core_process
            self._robot_core_process = None
            self._robot_core_running = False
            self._robot_core_transition = True

        try:
            if process:
                if not self._terminate_process_safely(process, "Robot Core"):
                    logger.error("Robot Core process may still be running")
        finally:
            with self._lock:
                self._robot_core_transition = False
        return {"message": "Robot core stopped.", "is_running": False}

    # --- Health Monitor ---
    def start_health_monitor(self):
        """啟動背景健康檢查執行緒"""
        if self._health_thread is not None:
            return
        self._health_stop_event.clear()
        self._health_thread = threading.Thread(target=self._health_check_loop, daemon=True)
        self._health_thread.start()

    def stop_health_monitor(self):
        """停止健康檢查執行緒"""
        self._health_stop_event.set()
        if self._health_thread:
            self._health_thread.join(timeout=3)
            self._health_thread = None

    def _health_check_loop(self):
        """健康檢查迴圈"""
        while not self._health_stop_event.is_set():
            crashed = self._check_processes()
            # 善後 callback 必須在鎖外執行（可能觸碰 navigator / delivery 的鎖）
            if 'navigation' in crashed and self.on_navigation_down is not None:
                try:
                    self.on_navigation_down()
                except Exception as e:
                    logger.error(f"Navigation-down handler failed: {e}")
            self._health_stop_event.wait(self.HEALTH_CHECK_INTERVAL)

    def _check_processes(self) -> list:
        """檢查所有子程序是否存活，回傳本輪偵測到 crash 的服務名稱"""
        crashed = []
        with self._lock:
            # 檢查 SLAM 程序
            if self._slam_process and self._slam_status == SlamStatus.MAPPING:
                ret = self._slam_process.poll()
                if ret is not None:
                    self._crash_info['slam'] = {
                        'exit_code': ret,
                        'time': self._get_timestamp()
                    }
                    self._slam_process = None
                    self._slam_status = SlamStatus.IDLE
                    crashed.append('slam')

            # 檢查 Navigation 程序
            if self._nav_process and self._nav_status == NavStatus.RUNNING:
                ret = self._nav_process.poll()
                if ret is not None:
                    self._crash_info['navigation'] = {
                        'exit_code': ret,
                        'time': self._get_timestamp()
                    }
                    self._nav_process = None
                    self._nav_status = NavStatus.IDLE
                    crashed.append('navigation')

            # 檢查 Robot Core 程序
            if self._robot_core_process and self._robot_core_running:
                ret = self._robot_core_process.poll()
                if ret is not None:
                    self._crash_info['robot_core'] = {
                        'exit_code': ret,
                        'time': self._get_timestamp()
                    }
                    self._robot_core_process = None
                    self._robot_core_running = False
                    crashed.append('robot_core')

        return crashed

    def _get_timestamp(self) -> str:
        """取得時間戳記"""
        from datetime import datetime
        return datetime.now().isoformat()

    @property
    def crash_info(self) -> dict:
        """取得最近的 crash 資訊"""
        with self._lock:
            return self._crash_info.copy()

    def clear_crash_info(self, service: Optional[str] = None):
        """清除 crash 資訊"""
        with self._lock:
            if service:
                self._crash_info.pop(service, None)
            else:
                self._crash_info.clear()

    # --- Cleanup ---
    def cleanup(self):
        """Clean up all running processes."""
        logger.info("Starting cleanup of all processes...")
        self.stop_health_monitor()
        # 鎖內取得進程所有權並重置狀態，實際終止在鎖外執行
        with self._lock:
            processes = [
                (self._slam_process, "SLAM"),
                (self._nav_process, "Navigation"),
                (self._robot_core_process, "Robot Core")
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

    def get_status_snapshot(self) -> dict:
        """線程安全地獲取完整狀態快照"""
        with self._lock:
            return {
                "robot_core_running": self._robot_core_running,
                "slam_status": self._slam_status,
                "nav_status": self._nav_status,
                "crash_info": self._crash_info.copy(),
                "e_stop_active": self._e_stop_active,
            }


# --- Global State Manager Instance ---
state = RobotStateManager()


# --- WebSocket Connection Manager ---
class ConnectionManager:
    """管理 WebSocket 連線"""

    def __init__(self):
        self._connections: Set[WebSocket] = set()
        self._lock = asyncio.Lock()

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        async with self._lock:
            self._connections.add(websocket)

    async def disconnect(self, websocket: WebSocket):
        async with self._lock:
            self._connections.discard(websocket)

    async def broadcast(self, message: dict):
        """廣播訊息給所有連線

        鎖內只複製連線集合，實際送出在鎖外並行執行，
        避免單一慢速 client 拖垮所有廣播與新連線。
        """
        async with self._lock:
            connections = list(self._connections)
        if not connections:
            return

        results = await asyncio.gather(
            *(conn.send_json(message) for conn in connections),
            return_exceptions=True
        )

        dead_connections = {
            conn for conn, result in zip(connections, results)
            if isinstance(result, Exception)
        }
        if dead_connections:
            for conn, result in zip(connections, results):
                if isinstance(result, Exception):
                    logger.debug(f"WebSocket connection lost: {result}")
            async with self._lock:
                self._connections -= dead_connections

    @property
    def connection_count(self) -> int:
        return len(self._connections)


ws_manager = ConnectionManager()


# --- Background Task Tracking ---
# event loop 對 task 只持弱引用，fire-and-forget 的 task 必須保留強引用
# 否則可能在執行途中被 GC 回收
_background_tasks: Set[asyncio.Task] = set()


def spawn_background_task(coro) -> asyncio.Task:
    """建立背景任務並保留強引用，完成後自動移除"""
    task = asyncio.create_task(coro)
    _background_tasks.add(task)
    task.add_done_callback(_background_tasks.discard)
    return task


async def status_broadcast_loop():
    """定期廣播狀態更新"""
    last_status = {}
    while True:
        await asyncio.sleep(1)  # 每秒檢查一次
        if ws_manager.connection_count == 0:
            continue

        # 取得當前狀態（使用 asyncio.to_thread 避免阻塞事件循環）
        current_status = await asyncio.to_thread(get_full_status)

        # 只在狀態變化時廣播
        if current_status != last_status:
            await ws_manager.broadcast({
                "type": "status_update",
                "data": current_status
            })
            last_status = current_status


def get_full_status() -> dict:
    """取得完整系統狀態（線程安全）"""
    # 使用原子快照避免狀態不一致
    snapshot = state.get_status_snapshot()

    is_complete = nav_manager.is_task_complete()
    feedback = nav_manager.get_feedback()
    distance_remaining = feedback.distance_remaining if feedback else None

    return {
        "robot_core": {
            "running": snapshot["robot_core_running"],
        },
        "slam": {
            "status": snapshot["slam_status"].value,
            "is_mapping": snapshot["slam_status"] == SlamStatus.MAPPING,
        },
        "navigation": {
            "status": snapshot["nav_status"].value,
            "nav_running": snapshot["nav_status"] == NavStatus.RUNNING,
            "is_complete": is_complete,
            "distance_remaining": distance_remaining,
        },
        "e_stop": {
            "active": snapshot["e_stop_active"],
        },
        "crash_info": snapshot["crash_info"],
    }


# --- E-Stop ROS2 Subscriber Thread ---
async def _delivery_monitor_loop():
    """背景任務：監控送餐導航狀態，偵測到達或卡住並推進狀態機"""
    while True:
        try:
            task = delivery_manager.current_task
            if task and task.status in [DeliveryTaskStatus.DELIVERING, DeliveryTaskStatus.RETURNING]:
                # goal 尚未發送時只等待：此刻 is_task_complete() 反映的是
                # 上一段航程的結果，據此推進狀態機會誤判「已到達」
                if delivery_manager.awaiting_goal:
                    await asyncio.sleep(0.5)
                    continue
                # Navigator 未就緒時跳過，避免 is_task_complete() 誤判為 True
                if not nav_manager.is_ready:
                    await asyncio.sleep(0.5)
                    continue
                if await asyncio.to_thread(nav_manager.is_stuck):
                    delivery_manager.mark_stuck()
                    logger.warning("Robot is stuck! Task status changed to STUCK")
                elif await asyncio.to_thread(nav_manager.is_task_complete):
                    if task.status == DeliveryTaskStatus.DELIVERING:
                        delivery_manager.mark_arrived()
                        logger.info("Arrived at table")
                    elif task.status == DeliveryTaskStatus.RETURNING:
                        delivery_manager.complete_return()
                        logger.info("Returned to start position")
        except Exception as e:
            logger.error(f"Delivery monitor error: {e}")
        await asyncio.sleep(0.5)


def _e_stop_subscriber_thread():
    """背景線程：訂閱 /e_stop topic 更新 API Server 狀態"""
    try:
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rclpy.executors import SingleThreadedExecutor
        from std_msgs.msg import Bool

        ensure_rclpy_initialized()
        node = rclpy.create_node('api_e_stop_listener')

        e_stop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        def _cb(msg):
            state.e_stop_active = msg.data

        node.create_subscription(Bool, '/e_stop', _cb, e_stop_qos)
        logger.info("E-Stop subscriber thread started")
        # 使用獨立的 executor 避免與 BasicNavigator 的 spin 衝突
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except Exception as e:
        logger.error(f"E-Stop subscriber thread failed: {e}")


# --- Lifespan for cleanup ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    state.start_health_monitor()
    # 啟動 E-Stop 訂閱背景線程
    e_stop_thread = threading.Thread(target=_e_stop_subscriber_thread, daemon=True)
    e_stop_thread.start()
    # 啟動狀態廣播任務
    broadcast_task = asyncio.create_task(status_broadcast_loop())
    # 啟動送餐狀態監控任務
    delivery_monitor_task = asyncio.create_task(_delivery_monitor_loop())
    yield
    delivery_monitor_task.cancel()
    broadcast_task.cancel()
    try:
        await delivery_monitor_task
    except asyncio.CancelledError:
        pass
    try:
        await broadcast_task
    except asyncio.CancelledError:
        pass
    state.cleanup()


# --- FastAPI App ---
app = FastAPI(title="Robot Control API", lifespan=lifespan)

# CORS 設定：從環境變數讀取允許的來源
# 預設允許所有來源 (開發環境)，生產環境應設定 CORS_ORIGINS 環境變數
ALLOWED_ORIGINS = os.environ.get("CORS_ORIGINS", "").split(",") if os.environ.get("CORS_ORIGINS") else ["*"]
# 過濾空字串
ALLOWED_ORIGINS = [origin.strip() for origin in ALLOWED_ORIGINS if origin.strip()]

# 根據 CORS 規範：當 allow_credentials=True 時，allow_origins 不能為 ["*"]
# 參考：https://developer.mozilla.org/en-US/docs/Web/HTTP/CORS/Errors/CORSNotSupportingCredentials
ALLOW_ALL_ORIGINS = "*" in ALLOWED_ORIGINS
ALLOW_CREDENTIALS = not ALLOW_ALL_ORIGINS

logger.info(f"CORS allowed origins: {ALLOWED_ORIGINS}, credentials: {ALLOW_CREDENTIALS}")

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=ALLOW_CREDENTIALS,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)


# --- WebSocket Endpoint ---
@app.websocket("/ws/status")
async def websocket_status(websocket: WebSocket):
    """WebSocket 端點：即時狀態更新"""
    await ws_manager.connect(websocket)
    try:
        # 連線時立即發送當前狀態（使用 to_thread 避免阻塞事件循環）
        initial_status = await asyncio.to_thread(get_full_status)
        await websocket.send_json({
            "type": "status_update",
            "data": initial_status
        })
        # 保持連線，等待客戶端斷線
        while True:
            # 接收心跳或指令（目前只是保持連線）
            data = await websocket.receive_text()
            if data == "ping":
                await websocket.send_text("pong")
    except WebSocketDisconnect:
        pass
    finally:
        await ws_manager.disconnect(websocket)


# --- Robot Core Endpoints ---
@app.post("/robot/start")
async def start_robot_core():
    """Start robot core system (motor controller, LiDAR, IMU, EKF)."""
    return await asyncio.to_thread(state.start_robot_core)

@app.post("/robot/stop")
async def stop_robot_core():
    """Stop robot core system."""
    return await asyncio.to_thread(state.stop_robot_core)

@app.post("/robot/start_lidar")
async def start_lidar_motor():
    """Start LiDAR motor by calling /start_motor service."""
    def _start_lidar():
        return subprocess.run(
            ["ros2", "service", "call", "/start_motor", "std_srvs/srv/Empty"],
            capture_output=True,
            text=True,
            timeout=10
        )
    try:
        result = await asyncio.to_thread(_start_lidar)
        if result.returncode != 0:
            raise HTTPException(status_code=500, detail=f"Failed to start LiDAR: {result.stderr}")
        return {"message": "LiDAR motor started."}
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=500, detail="LiDAR start timed out.")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to start LiDAR: {str(e)}")

@app.get("/robot/e_stop")
async def get_e_stop_status():
    """Get E-Stop status."""
    return {"active": state.e_stop_active}

@app.get("/robot/status")
async def get_robot_status():
    """Get robot core status."""
    return {"is_running": state.robot_core_running}

@app.get("/health")
async def get_health_status():
    """Get overall system health status including crash info."""
    crash_info = state.crash_info
    return {
        "robot_core": {
            "running": state.robot_core_running,
            "crashed": "robot_core" in crash_info,
            "last_crash": crash_info.get("robot_core"),
        },
        "slam": {
            "status": state.slam_status,
            "crashed": "slam" in crash_info,
            "last_crash": crash_info.get("slam"),
        },
        "navigation": {
            "status": state.nav_status,
            "crashed": "navigation" in crash_info,
            "last_crash": crash_info.get("navigation"),
        },
    }

@app.post("/health/clear")
async def clear_crash_info(service: Optional[str] = None):
    """Clear crash info for a specific service or all services."""
    state.clear_crash_info(service)
    return {"message": f"Crash info cleared for: {service or 'all'}"}


# --- Navigation Endpoints ---
@app.post("/navigate_to_goal")
async def navigate_to_goal(goal: Goal):
    """Send navigation goal to Nav2 stack."""
    # 檢查導航是否已啟動
    if state.nav_status != NavStatus.RUNNING:
        raise HTTPException(status_code=400, detail="Navigation is not running. Please start navigation first.")

    # 確保 Nav2 已準備好（使用 asyncio.to_thread 避免阻塞事件循環）
    is_ready = await asyncio.to_thread(nav_manager.ensure_nav2_ready)
    if not is_ready:
        raise HTTPException(status_code=503, detail="Nav2 is not ready. Please wait and try again.")

    try:
        logger.info(f"Received goal: x={goal.x}, y={goal.y}, yaw={goal.yaw_deg}")
        await asyncio.to_thread(nav_manager.send_goal, goal.x, goal.y, goal.yaw_deg)
        return {"message": "Goal received, navigation started."}
    except RuntimeError as e:
        logger.error(f"Failed to send goal: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/navigation/cancel")
async def cancel_navigation():
    """Cancel current navigation goal."""
    try:
        await asyncio.to_thread(nav_manager.cancel_task)
        return {"message": "Navigation cancelled."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/navigation/status")
async def get_navigation_status():
    """Get current navigation status."""
    try:
        # 使用 asyncio.to_thread 避免阻塞事件循環
        is_complete = await asyncio.to_thread(nav_manager.is_task_complete)
        feedback = await asyncio.to_thread(nav_manager.get_feedback)
        distance_remaining = feedback.distance_remaining if feedback else None

        return {
            "is_complete": is_complete,
            "distance_remaining": distance_remaining,
            "nav_running": state.nav_status == NavStatus.RUNNING,
        }
    except RuntimeError as e:
        logger.warning(f"Navigation status check failed: {e}")
        return {"is_complete": True, "distance_remaining": None, "nav_running": state.nav_status == NavStatus.RUNNING}

@app.get("/maps/list")
async def list_maps():
    """List all available maps in the map directory."""
    try:
        maps = []
        if os.path.exists(MAP_SAVE_PATH):
            for file in os.listdir(MAP_SAVE_PATH):
                if file.endswith('.yaml'):
                    map_name = file[:-5]
                    yaml_path = os.path.join(MAP_SAVE_PATH, file)
                    pgm_path = os.path.join(MAP_SAVE_PATH, f"{map_name}.pgm")
                    if os.path.exists(pgm_path):
                        maps.append({
                            "name": map_name,
                            "yaml_path": yaml_path,
                            "pgm_path": pgm_path,
                        })
        return {"maps": maps, "default": "map"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to list maps: {str(e)}")


@app.get("/maps/{map_name}/image")
async def get_map_image(map_name: str):
    """Get the map image as PNG (converted from PGM)."""
    safe_name = validate_map_name(map_name)
    pgm_path = os.path.join(MAP_SAVE_PATH, f"{safe_name}.pgm")

    if not os.path.exists(pgm_path):
        raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found.")

    try:
        # 將 PGM 轉換為 PNG
        with Image.open(pgm_path) as img:
            png_buffer = io.BytesIO()
            img.save(png_buffer, format="PNG")
            png_buffer.seek(0)
            return Response(content=png_buffer.read(), media_type="image/png")
    except Exception as e:
        logger.error(f"Failed to convert map image: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to load map image: {str(e)}")


@app.get("/maps/{map_name}/metadata")
async def get_map_metadata(map_name: str):
    """Get the map metadata (from YAML file)."""
    safe_name = validate_map_name(map_name)
    yaml_path = os.path.join(MAP_SAVE_PATH, f"{safe_name}.yaml")
    pgm_path = os.path.join(MAP_SAVE_PATH, f"{safe_name}.pgm")

    if not os.path.exists(yaml_path):
        raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found.")

    try:
        with open(yaml_path, 'r') as f:
            map_yaml = yaml.safe_load(f)

        # 使用 PIL 解析 PGM 尺寸（正確處理註解、分行尺寸等合法格式）
        width, height = 0, 0
        if os.path.exists(pgm_path):
            try:
                with Image.open(pgm_path) as img:
                    width, height = img.size
            except Exception as e:
                logger.error(f"Failed to read PGM size for '{safe_name}': {e}")

        return {
            "resolution": map_yaml.get("resolution", 0.05),
            "origin": map_yaml.get("origin", [0, 0, 0]),
            "width": width,
            "height": height,
            "negate": map_yaml.get("negate", 0),
            "occupied_thresh": map_yaml.get("occupied_thresh", 0.65),
            "free_thresh": map_yaml.get("free_thresh", 0.196),
        }
    except Exception as e:
        # 細節只留在 log，不把內部錯誤資訊回傳給 client
        logger.error(f"Failed to read map metadata for '{safe_name}': {e}")
        raise HTTPException(status_code=500, detail="Failed to read map metadata.")

@app.post("/navigation/start")
async def start_navigation(request: NavigationStartRequest = None):
    """Start autonomous navigation by launching autonomous_navigation.launch.py."""
    map_name = request.map_name if request else None
    return await asyncio.to_thread(state.start_navigation, map_name)

@app.post("/navigation/stop")
async def stop_navigation():
    """Stop autonomous navigation."""
    result = await asyncio.to_thread(state.stop_navigation)
    # 導航進程已停止：重置 navigator、將進行中送餐標記為卡住
    await asyncio.to_thread(_handle_navigation_down)
    return result

def _publish_initial_pose(x: float, y: float, yaw: float,
                          cov_xy: float = 0.25,
                          cov_yaw: float = 0.06853891945200942,
                          timeout: float = 10.0) -> bool:
    """發布 /initialpose 給 AMCL（在工作執行緒中呼叫，yaw 為弧度）。

    以 get_subscription_count() 輪詢確認 AMCL 已訂閱後才發布，
    避免 DDS discovery 未完成導致訊息遺失。回傳是否確認有訂閱者。
    """
    ensure_rclpy_initialized()
    # node 名稱加亂數後綴，避免併發請求建立同名 node
    node = rclpy.create_node(f'initial_pose_pub_{uuid4().hex[:8]}')
    try:
        publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

        # 等待訂閱者（AMCL）出現
        deadline = time.time() + timeout
        while publisher.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.1)
        has_subscriber = publisher.get_subscription_count() > 0
        if not has_subscriber:
            logger.warning(f"No subscriber on /initialpose after {timeout:.1f}s")

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        # 從 yaw 計算四元數
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # 設置協方差（對角線元素）
        msg.pose.covariance[0] = cov_xy   # x
        msg.pose.covariance[7] = cov_xy   # y
        msg.pose.covariance[35] = cov_yaw  # yaw

        publisher.publish(msg)
        time.sleep(0.3)  # 給 DDS 傳輸時間
        logger.info(f"Published initial pose: x={x}, y={y}, yaw={yaw}")
        return has_subscriber
    finally:
        node.destroy_node()


@app.post("/navigation/set_initial_pose")
async def set_initial_pose(request: InitialPoseRequest):
    """Set the initial pose for AMCL localization."""
    if state.nav_status != NavStatus.RUNNING:
        raise HTTPException(status_code=400, detail="Navigation is not running. Start navigation first.")

    try:
        has_subscriber = await asyncio.to_thread(
            _publish_initial_pose, request.x, request.y, request.yaw
        )
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to set initial pose: {e}")
        raise HTTPException(status_code=500, detail="Failed to set initial pose.")

    if not has_subscriber:
        raise HTTPException(
            status_code=503,
            detail="AMCL is not subscribed to /initialpose yet; try again later."
        )

    return {"message": "Initial pose set successfully", "x": request.x, "y": request.y, "yaw": request.yaw}


# --- SLAM Endpoints ---
@app.post("/slam/start")
async def start_mapping():
    """Start SLAM mapping by launching mapping.launch.py."""
    return await asyncio.to_thread(state.start_slam)

@app.post("/slam/stop")
async def stop_mapping():
    """Stop SLAM mapping."""
    return await asyncio.to_thread(state.stop_slam)

def validate_map_path(map_name: str, base_path: str) -> str:
    """驗證並返回安全的地圖路徑（重用 validate_map_name）"""
    safe_name = validate_map_name(map_name)
    return os.path.join(base_path, safe_name)


@app.post("/slam/save_map")
async def save_map(request: MapSaveRequest):
    """Save the current map using nav2_map_server."""
    map_name = request.map_name.strip()
    if not map_name:
        raise HTTPException(status_code=400, detail="Map name is required.")

    # 使用安全的路徑驗證函數
    map_path = validate_map_path(map_name, MAP_SAVE_PATH)

    def _save_map():
        # 使用 /map_saver topic，需指定 TRANSIENT_LOCAL QoS 才能接收 latched message
        # 加上 map_timeout 讓 map_saver_cli 等久一點，避免 DDS discovery 來不及配對
        cmd = f"source /opt/ros/humble/setup.bash && source {WORKSPACE_ROOT}/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f {map_path} -t /map_saver --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=10000.0"
        return subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True,
            text=True,
            timeout=30
        )

    # 只有正在建圖時才允許存圖（原子性檢查 + 進入 SAVING）
    state.begin_map_save()
    try:
        result = await asyncio.to_thread(_save_map)

        if result.returncode != 0:
            raise HTTPException(status_code=500, detail=f"Map save failed: {result.stderr}")

        return {
            "message": "Map saved successfully.",
            "map_path": map_path,
            "files": [f"{map_path}.pgm", f"{map_path}.yaml"]
        }
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=500, detail="Map save timed out.")
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to save map: {str(e)}")
    finally:
        state.end_map_save()

@app.get("/slam/status")
async def get_slam_status():
    """Get current SLAM status."""
    return {
        "status": state.slam_status,
        "is_mapping": state.slam_status == SlamStatus.MAPPING,
    }


# --- Waypoint Endpoints ---
@app.get("/maps/{map_name}/waypoints", response_model=List[Waypoint])
async def get_waypoints(map_name: str):
    """Get all waypoints for a specific map."""
    map_name = validate_map_name(map_name)
    # 驗證地圖存在
    map_yaml = os.path.join(MAP_SAVE_PATH, f"{map_name}.yaml")
    if not os.path.exists(map_yaml):
        raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found.")
    return load_waypoints(map_name)


@app.post("/maps/{map_name}/waypoints", response_model=Waypoint)
async def create_waypoint(map_name: str, waypoint: WaypointCreate):
    """Create a new waypoint for a specific map."""
    map_name = validate_map_name(map_name)
    # 驗證地圖存在
    map_yaml = os.path.join(MAP_SAVE_PATH, f"{map_name}.yaml")
    if not os.path.exists(map_yaml):
        raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found.")

    waypoints = load_waypoints(map_name)

    # 建立新的 waypoint
    now = datetime.now().isoformat()
    new_waypoint = Waypoint(
        id=str(uuid4()),
        name=waypoint.name,
        x=waypoint.x,
        y=waypoint.y,
        yaw_deg=waypoint.yaw_deg,
        created_at=now,
        updated_at=now,
    )

    waypoints.append(new_waypoint)
    save_waypoints(map_name, waypoints)

    logger.info(f"Created waypoint '{new_waypoint.name}' for map '{map_name}'")
    return new_waypoint


@app.put("/maps/{map_name}/waypoints/{waypoint_id}", response_model=Waypoint)
async def update_waypoint(map_name: str, waypoint_id: str, update: WaypointUpdate):
    """Update an existing waypoint."""
    map_name = validate_map_name(map_name)
    waypoints = load_waypoints(map_name)

    # 找到要更新的 waypoint
    for i, wp in enumerate(waypoints):
        if wp.id == waypoint_id:
            # 更新欄位
            updated_data = wp.model_dump()
            update_dict = update.model_dump(exclude_unset=True)
            updated_data.update(update_dict)
            updated_data['updated_at'] = datetime.now().isoformat()

            waypoints[i] = Waypoint(**updated_data)
            save_waypoints(map_name, waypoints)

            logger.info(f"Updated waypoint '{waypoints[i].name}' for map '{map_name}'")
            return waypoints[i]

    raise HTTPException(status_code=404, detail=f"Waypoint '{waypoint_id}' not found.")


@app.delete("/maps/{map_name}/waypoints/{waypoint_id}")
async def delete_waypoint(map_name: str, waypoint_id: str):
    """Delete a waypoint."""
    map_name = validate_map_name(map_name)
    waypoints = load_waypoints(map_name)

    # 找到並刪除 waypoint
    for i, wp in enumerate(waypoints):
        if wp.id == waypoint_id:
            deleted_name = wp.name
            waypoints.pop(i)
            save_waypoints(map_name, waypoints)

            logger.info(f"Deleted waypoint '{deleted_name}' from map '{map_name}'")
            return {"message": f"Waypoint '{deleted_name}' deleted."}

    raise HTTPException(status_code=404, detail=f"Waypoint '{waypoint_id}' not found.")


@app.post("/maps/{map_name}/waypoints/{waypoint_id}/navigate")
async def navigate_to_waypoint(map_name: str, waypoint_id: str):
    """Navigate to a specific waypoint."""
    map_name = validate_map_name(map_name)
    # 檢查導航是否已啟動
    if state.nav_status != NavStatus.RUNNING:
        raise HTTPException(status_code=400, detail="Navigation is not running. Please start navigation first.")

    waypoints = load_waypoints(map_name)

    # 找到 waypoint
    target_wp = None
    for wp in waypoints:
        if wp.id == waypoint_id:
            target_wp = wp
            break

    if target_wp is None:
        raise HTTPException(status_code=404, detail=f"Waypoint '{waypoint_id}' not found.")

    # 確保 Nav2 已準備好
    is_ready = await asyncio.to_thread(nav_manager.ensure_nav2_ready)
    if not is_ready:
        raise HTTPException(status_code=503, detail="Nav2 is not ready. Please wait and try again.")

    try:
        logger.info(f"Navigating to waypoint '{target_wp.name}': x={target_wp.x}, y={target_wp.y}, yaw={target_wp.yaw_deg}")
        await asyncio.to_thread(nav_manager.send_goal, target_wp.x, target_wp.y, target_wp.yaw_deg)
        return {"message": f"Navigation to '{target_wp.name}' started."}
    except RuntimeError as e:
        logger.error(f"Failed to navigate to waypoint: {e}")
        raise HTTPException(status_code=500, detail=str(e))


# --- Table API ---
@app.get("/maps/{map_name}/tables", response_model=List[Table])
async def get_tables(map_name: str):
    """Get all tables for a map."""
    map_name = validate_map_name(map_name)
    tables = load_tables(map_name)
    return tables


@app.post("/maps/{map_name}/tables", response_model=Table)
async def create_table(map_name: str, table: TableCreate):
    """Create a new table."""
    map_name = validate_map_name(map_name)
    tables = load_tables(map_name)

    # 檢查桌號是否重複
    for t in tables:
        if t.number == table.number:
            raise HTTPException(status_code=400, detail=f"Table number {table.number} already exists.")

    now = datetime.now().isoformat()
    new_table = Table(
        id=str(uuid4()),
        **table.model_dump(),
        created_at=now,
        updated_at=now
    )

    tables.append(new_table)
    save_tables(map_name, tables)

    logger.info(f"Created table {new_table.number} for map '{map_name}'")
    return new_table


@app.put("/maps/{map_name}/tables/{table_id}", response_model=Table)
async def update_table(map_name: str, table_id: str, update: TableUpdate):
    """Update a table."""
    map_name = validate_map_name(map_name)
    tables = load_tables(map_name)

    for i, t in enumerate(tables):
        if t.id == table_id:
            # 檢查桌號是否重複
            if update.number is not None and update.number != t.number:
                for other in tables:
                    if other.id != table_id and other.number == update.number:
                        raise HTTPException(status_code=400, detail=f"Table number {update.number} already exists.")

            # 更新欄位
            updated_data = t.model_dump()
            update_dict = update.model_dump(exclude_unset=True)
            updated_data.update(update_dict)
            updated_data['updated_at'] = datetime.now().isoformat()

            tables[i] = Table(**updated_data)
            save_tables(map_name, tables)

            logger.info(f"Updated table {tables[i].number} for map '{map_name}'")
            return tables[i]

    raise HTTPException(status_code=404, detail=f"Table '{table_id}' not found.")


@app.delete("/maps/{map_name}/tables/{table_id}")
async def delete_table(map_name: str, table_id: str):
    """Delete a table."""
    map_name = validate_map_name(map_name)
    tables = load_tables(map_name)

    for i, t in enumerate(tables):
        if t.id == table_id:
            deleted_number = t.number
            tables.pop(i)
            save_tables(map_name, tables)

            logger.info(f"Deleted table {deleted_number} from map '{map_name}'")
            return {"message": f"Table {deleted_number} deleted."}

    raise HTTPException(status_code=404, detail=f"Table '{table_id}' not found.")


# --- Delivery Task Manager ---
class DeliveryManager:
    """管理送餐任務"""

    def __init__(self):
        self._lock = threading.Lock()
        self._current_task: Optional[DeliveryTask] = None
        self._current_map: Optional[str] = None
        # goal 世代握手：狀態推進後、導航目標實際發送前為 True。
        # monitor loop 在此期間不得依 is_task_complete() 推進狀態機
        # （否則會拿上一段航程的完成結果誤判「已到達」）。
        self._awaiting_goal = False

    @property
    def current_task(self) -> Optional[DeliveryTask]:
        """回傳當前任務的深拷貝快照，避免呼叫端在鎖外讀寫共享的 mutable 物件"""
        with self._lock:
            if self._current_task is None:
                return None
            return self._current_task.model_copy(deep=True)

    @property
    def awaiting_goal(self) -> bool:
        with self._lock:
            return self._awaiting_goal

    def goal_dispatched(self):
        """導航目標已成功發送，解除等待狀態"""
        with self._lock:
            self._awaiting_goal = False

    def start_delivery(self, map_name: str, table_ids: List[str], start_position: Position) -> DeliveryTask:
        """開始送餐任務"""
        # 檔案 I/O 與站點驗證在鎖外執行，避免持鎖做磁碟讀取
        tables = load_tables(map_name)
        table_map = {t.id: t for t in tables}

        # 建立送餐站點
        stops = []
        for table_id in table_ids:
            if table_id not in table_map:
                raise HTTPException(status_code=404, detail=f"Table '{table_id}' not found.")
            table = table_map[table_id]
            if not table.isActive:
                raise HTTPException(status_code=400, detail=f"Table {table.number} is not active.")
            stops.append(DeliveryStop(
                tableId=table.id,
                tableNumber=table.number,
                tableName=table.name,
                status=DeliveryStopStatus.PENDING
            ))

        if not stops:
            raise HTTPException(status_code=400, detail="No valid tables selected.")

        with self._lock:
            if self._current_task is not None and self._current_task.status != DeliveryTaskStatus.IDLE:
                raise HTTPException(status_code=400, detail="A delivery task is already in progress.")

            # 建立任務
            task = DeliveryTask(
                id=str(uuid4()),
                stops=stops,
                status=DeliveryTaskStatus.DELIVERING,
                currentStopIndex=0,
                startPosition=start_position,
                createdAt=datetime.now().isoformat()
            )

            # 設定第一個站點為進行中
            task.stops[0].status = DeliveryStopStatus.IN_PROGRESS

            self._current_task = task
            self._current_map = map_name
            # 第一個導航目標尚未發送
            self._awaiting_goal = True

            return task.model_copy(deep=True)

    def get_current_stop_table(self) -> Optional[Table]:
        """取得當前站點的桌位資訊"""
        # 鎖內只讀取需要的識別資訊，檔案 I/O 在鎖外執行
        with self._lock:
            if self._current_task is None or self._current_map is None:
                return None
            if self._current_task.currentStopIndex >= len(self._current_task.stops):
                return None

            stop_table_id = self._current_task.stops[self._current_task.currentStopIndex].tableId
            map_name = self._current_map

        tables = load_tables(map_name)
        for t in tables:
            if t.id == stop_table_id:
                return t
        return None

    def mark_stuck(self) -> Optional[DeliveryTask]:
        """標記機器人卡住"""
        with self._lock:
            if self._current_task is None:
                return None
            if self._current_task.status not in [DeliveryTaskStatus.DELIVERING, DeliveryTaskStatus.RETURNING]:
                return self._current_task.model_copy(deep=True)

            self._current_task.status = DeliveryTaskStatus.STUCK
            self._awaiting_goal = False
            return self._current_task.model_copy(deep=True)

    def mark_arrived(self) -> Optional[DeliveryTask]:
        """標記已到達當前桌位"""
        with self._lock:
            if self._current_task is None:
                return None
            # awaiting_goal 期間的「完成」是上一段航程的結果，不得推進狀態機
            if self._current_task.status != DeliveryTaskStatus.DELIVERING or self._awaiting_goal:
                return self._current_task.model_copy(deep=True)

            stop = self._current_task.stops[self._current_task.currentStopIndex]
            stop.status = DeliveryStopStatus.ARRIVED
            self._current_task.status = DeliveryTaskStatus.AT_TABLE

            return self._current_task.model_copy(deep=True)

    def confirm_arrival(self) -> Optional[DeliveryTask]:
        """確認到達並前往下一桌；非 AT_TABLE 狀態回 409"""
        with self._lock:
            if self._current_task is None:
                return None
            if self._current_task.status != DeliveryTaskStatus.AT_TABLE:
                raise HTTPException(
                    status_code=409,
                    detail=f"Cannot confirm arrival in state '{self._current_task.status.value}'."
                )

            # 標記當前站點完成
            stop = self._current_task.stops[self._current_task.currentStopIndex]
            stop.status = DeliveryStopStatus.COMPLETED

            # 前往下一桌
            return self._advance_to_next()

    def skip_table(self) -> Optional[DeliveryTask]:
        """跳過當前桌位；非 AT_TABLE 狀態回 409"""
        with self._lock:
            if self._current_task is None:
                return None
            if self._current_task.status != DeliveryTaskStatus.AT_TABLE:
                raise HTTPException(
                    status_code=409,
                    detail=f"Cannot skip table in state '{self._current_task.status.value}'."
                )

            # 標記當前站點跳過
            stop = self._current_task.stops[self._current_task.currentStopIndex]
            stop.status = DeliveryStopStatus.SKIPPED

            # 前往下一桌
            return self._advance_to_next()

    def _advance_to_next(self) -> DeliveryTask:
        """移動到下一個站點（需要已獲得鎖）"""
        self._current_task.currentStopIndex += 1

        # 檢查是否還有站點
        if self._current_task.currentStopIndex < len(self._current_task.stops):
            # 設定下一個站點為進行中
            next_stop = self._current_task.stops[self._current_task.currentStopIndex]
            next_stop.status = DeliveryStopStatus.IN_PROGRESS
            self._current_task.status = DeliveryTaskStatus.DELIVERING
        else:
            # 所有站點完成，返回出發點
            self._current_task.status = DeliveryTaskStatus.RETURNING

        # 新的導航目標尚未發送
        self._awaiting_goal = True
        return self._current_task.model_copy(deep=True)

    def retry_current_stop(self) -> Optional[DeliveryTask]:
        """STUCK 狀態下重試：重發當前站點（或返程）的導航目標並回到對應狀態"""
        with self._lock:
            if self._current_task is None:
                return None
            if self._current_task.status != DeliveryTaskStatus.STUCK:
                raise HTTPException(
                    status_code=409,
                    detail=f"Can only retry when stuck (current state: '{self._current_task.status.value}')."
                )

            if self._current_task.currentStopIndex < len(self._current_task.stops):
                # 卡在前往某桌的路上：重試該桌
                stop = self._current_task.stops[self._current_task.currentStopIndex]
                stop.status = DeliveryStopStatus.IN_PROGRESS
                self._current_task.status = DeliveryTaskStatus.DELIVERING
            else:
                # 卡在返程路上：重試返回出發點
                self._current_task.status = DeliveryTaskStatus.RETURNING

            self._awaiting_goal = True
            return self._current_task.model_copy(deep=True)

    def complete_return(self) -> Optional[DeliveryTask]:
        """完成返回"""
        with self._lock:
            if self._current_task is None:
                return None
            # 只有「返程 goal 已發送且完成」才算返回；awaiting_goal 期間的完成是舊結果
            if self._current_task.status != DeliveryTaskStatus.RETURNING or self._awaiting_goal:
                return self._current_task.model_copy(deep=True)

            self._current_task.status = DeliveryTaskStatus.IDLE
            task = self._current_task
            self._current_task = None
            self._current_map = None
            self._awaiting_goal = False

            return task

    def cancel_delivery(self):
        """取消送餐任務"""
        with self._lock:
            self._current_task = None
            self._current_map = None
            self._awaiting_goal = False


# 全局送餐管理器
delivery_manager = DeliveryManager()


# --- Delivery API ---
async def _dispatch_delivery_goal(task: Optional[DeliveryTask]) -> None:
    """依任務快照發送當前導航目標；成功後透過 goal_dispatched() 解除 awaiting 狀態。

    只有在 awaiting_goal 為 True（狀態剛推進、目標尚未發送）時才會發送，
    避免重複對同一目標下 goal。
    """
    if task is None:
        return
    if not delivery_manager.awaiting_goal:
        return
    try:
        if task.status == DeliveryTaskStatus.DELIVERING:
            target = delivery_manager.get_current_stop_table()
            if target is None:
                logger.error("Current stop table not found; delivery goal not dispatched")
                return
            x, y, yaw_deg = target.x, target.y, target.yaw_deg
            description = f"table {target.number}"
        elif task.status == DeliveryTaskStatus.RETURNING:
            x, y = task.startPosition.x, task.startPosition.y
            yaw_deg = math.degrees(task.startPosition.yaw)  # Position.yaw 為弧度
            description = "start position"
        else:
            return

        is_ready = await asyncio.to_thread(nav_manager.ensure_nav2_ready)
        if not is_ready:
            logger.warning("Nav2 not ready, delivery goal not dispatched")
            return
        await asyncio.to_thread(nav_manager.send_goal, x, y, yaw_deg)
        delivery_manager.goal_dispatched()
        logger.info(f"Delivery goal dispatched: {description}")
    except Exception as e:
        logger.error(f"Failed to dispatch delivery goal: {e}")


@app.post("/delivery/start", response_model=DeliveryTask)
async def start_delivery(request: DeliveryStartRequest):
    """Start a delivery task."""
    # 決定使用哪個地圖
    if request.mapName:
        map_name = validate_map_name(request.mapName)
    elif state.current_map:
        map_name = state.current_map
    else:
        # 回退：列出可用地圖並使用第一個
        maps_info = []
        for filename in os.listdir(MAP_SAVE_PATH):
            if filename.endswith('.yaml'):
                name = filename[:-5]
                pgm_path = os.path.join(MAP_SAVE_PATH, f"{name}.pgm")
                if os.path.exists(pgm_path):
                    maps_info.append(name)
        if not maps_info:
            raise HTTPException(status_code=400, detail="No maps available.")
        map_name = maps_info[0]

    # 如果導航未啟動，自動啟動導航
    nav_just_started = False
    if state.nav_status != NavStatus.RUNNING:
        logger.info(f"Navigation not running, auto-starting with map: {map_name}")
        try:
            await asyncio.to_thread(state.start_navigation, map_name)
            nav_just_started = True
        except HTTPException as e:
            raise HTTPException(status_code=500, detail=f"Failed to auto-start navigation: {e.detail}")
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to auto-start navigation: {str(e)}")

    # 如果導航剛啟動：先等 Nav2 完全就緒（waitUntilNav2Active 已包含 AMCL active），
    # 再用前端傳來的起點位置設定初始定位；失敗直接回錯，不帶著錯誤定位開始送餐
    if nav_just_started:
        is_ready = await asyncio.to_thread(nav_manager.ensure_nav2_ready)
        if not is_ready:
            raise HTTPException(status_code=503, detail="Nav2 failed to become active after auto-start.")

        logger.info(f"Setting initial pose to start position: x={request.startPosition.x}, y={request.startPosition.y}, yaw={request.startPosition.yaw}")
        try:
            has_subscriber = await asyncio.to_thread(
                _publish_initial_pose,
                request.startPosition.x,
                request.startPosition.y,
                request.startPosition.yaw,
                0.1,   # 較小的協方差 = 更確定的位置
                0.05,
            )
        except Exception as e:
            logger.error(f"Failed to set initial pose: {e}")
            raise HTTPException(status_code=500, detail="Failed to set initial pose for delivery.")

        if not has_subscriber:
            raise HTTPException(
                status_code=503,
                detail="AMCL is not subscribed to /initialpose; cannot set start position."
            )

        logger.info("Initial pose set successfully")
        await asyncio.sleep(1)  # 給 AMCL 時間處理

    task = delivery_manager.start_delivery(map_name, request.tableIds, request.startPosition)

    # 非阻塞地導航到第一個桌位（避免 ensure_nav2_ready 掛住阻塞 HTTP 回應）
    spawn_background_task(_dispatch_delivery_goal(task))

    return task


@app.post("/delivery/confirm", response_model=DeliveryTask)
async def confirm_delivery_arrival():
    """Confirm arrival at current table and proceed to next."""
    task = delivery_manager.confirm_arrival()
    if task is None:
        raise HTTPException(status_code=400, detail="No active delivery task.")

    # 導航到下一個桌位或返回出發點（依快照決策；只有 awaiting 時才會實際發送）
    await _dispatch_delivery_goal(task)

    return task


@app.post("/delivery/skip", response_model=DeliveryTask)
async def skip_delivery_table():
    """Skip current table and proceed to next."""
    task = delivery_manager.skip_table()
    if task is None:
        raise HTTPException(status_code=400, detail="No active delivery task.")

    # 導航到下一個桌位或返回出發點（依快照決策；只有 awaiting 時才會實際發送）
    await _dispatch_delivery_goal(task)

    return task


@app.post("/delivery/retry", response_model=DeliveryTask)
async def retry_delivery():
    """Retry current stop (or the return trip) after the robot got stuck."""
    task = delivery_manager.retry_current_stop()
    if task is None:
        raise HTTPException(status_code=400, detail="No active delivery task.")

    # 重發當前目標
    await _dispatch_delivery_goal(task)

    return task


@app.post("/delivery/cancel")
async def cancel_delivery():
    """Cancel the current delivery task."""
    delivery_manager.cancel_delivery()
    # cancelTask 會 spin rclpy node 並等待 action server 回應，不可在 event loop 上直接呼叫
    await asyncio.to_thread(nav_manager.cancel_task)
    logger.info("Delivery cancelled")
    return {"message": "Delivery cancelled."}


@app.get("/delivery/status")
async def get_delivery_status():
    """Get current delivery task status (pure read, no side effects)."""
    task = delivery_manager.current_task

    distance_remaining = None
    is_stuck = False

    if task and task.status in [DeliveryTaskStatus.DELIVERING, DeliveryTaskStatus.RETURNING]:
        # navigator 鎖可能被正在 spin 的執行緒持有數秒，不可在 event loop 上直接搶
        feedback = await asyncio.to_thread(nav_manager.get_feedback)
        if feedback and hasattr(feedback, 'distance_remaining'):
            distance_remaining = feedback.distance_remaining

    if task and task.status == DeliveryTaskStatus.STUCK:
        is_stuck = True

    return {
        "task": task,
        "distanceRemaining": distance_remaining,
        "isStuck": is_stuck
    }


@app.get("/robot/position")
async def get_robot_position():
    """Get current robot position from odometry."""
    # 這個 API 需要從 ROS 取得當前位置
    # 目前使用 placeholder，實際應用需要訂閱 /amcl_pose 或 /odom
    # 這裡返回預設值，前端應該使用 rosbridge 直接訂閱
    return {"x": 0.0, "y": 0.0, "yaw": 0.0}


# --- Navigator Manager ---
class NavigatorManager:
    """管理 BasicNavigator 的生命週期"""

    def __init__(self):
        self.navigator: Optional[BasicNavigator] = None
        self._nav2_ready = False
        self._lock = threading.Lock()
        # 序列化 waitUntilNav2Active：避免多個執行緒同時對同一個 node spin
        self._ready_lock = threading.Lock()

    @property
    def is_ready(self) -> bool:
        """Navigator 是否已就緒"""
        with self._lock:
            return self._nav2_ready and self.navigator is not None

    def ensure_nav2_ready(self) -> bool:
        """確保 Nav2 已準備好"""
        # 快速檢查（持有狀態鎖）
        with self._lock:
            if self._nav2_ready and self.navigator is not None:
                return True

        # 慢路徑整段以 _ready_lock 序列化：
        # 避免多個執行緒同時對同一個 BasicNavigator node 執行 waitUntilNav2Active（併發 spin 會拋錯）
        with self._ready_lock:
            # 後到者等到鎖後重新檢查，可能前一個 waiter 已完成
            with self._lock:
                if self._nav2_ready and self.navigator is not None:
                    return True
                if self.navigator is None:
                    if not ensure_rclpy_initialized():
                        logger.error("Failed to initialize rclpy")
                        return False
                    logger.info("Creating BasicNavigator...")
                    self.navigator = BasicNavigator()
                nav = self.navigator

            # 等待 Nav2 就緒（不持有狀態鎖，避免阻塞查詢類操作）
            try:
                logger.info("Waiting for Nav2 to become active...")
                nav.waitUntilNav2Active()
                with self._lock:
                    if self.navigator is not nav:
                        # 等待期間被 reset，視為未就緒
                        return False
                    self._nav2_ready = True
                logger.info("Nav2 is active.")
                return True
            except RuntimeError as e:
                logger.error(f"Failed to connect to Nav2: {e}")
                return False

    def reset(self):
        """重置 navigator（導航進程停止或崩潰後呼叫）

        注意：不使用 lifecycleShutdown()——它會對 nav2 lifecycle 服務發請求，
        在 nav2 已死亡時可能無限期阻塞；進程本身由 RobotStateManager 負責終止，
        這裡只需銷毀本地 node。
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

    def send_goal(self, x: float, y: float, yaw_deg: float):
        """發送導航目標（非阻塞，線程安全）"""
        with self._lock:
            if self.navigator is None:
                raise RuntimeError("Navigator not initialized")

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = x
            goal_pose.pose.position.y = y

            yaw_rad = math.radians(yaw_deg)
            goal_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

            logger.info(f"Sending goal: x={x}, y={y}, yaw={yaw_deg}")
            self.navigator.goToPose(goal_pose)
            logger.info("Goal sent (non-blocking)")

    def is_task_complete(self) -> bool:
        """檢查任務是否完成（線程安全）"""
        with self._lock:
            if self.navigator is None or not self._nav2_ready:
                return True
            try:
                return self.navigator.isTaskComplete()
            except RuntimeError as e:
                # 無法判定時回 False，避免被誤判為「已完成」而錯誤推進狀態機
                logger.warning(f"Task complete check failed: {e}")
                return False

    def get_feedback(self):
        """獲取導航反饋（線程安全）"""
        with self._lock:
            if self.navigator is None or not self._nav2_ready:
                return None
            try:
                return self.navigator.getFeedback()
            except RuntimeError as e:
                logger.debug(f"Get feedback failed: {e}")
                return None

    def cancel_task(self):
        """取消當前任務（線程安全）"""
        with self._lock:
            # 未就緒時沒有可取消的目標；且此時可能有執行緒正在
            # waitUntilNav2Active spin 同一個 node，不可併發 spin
            if self.navigator is not None and self._nav2_ready:
                self.navigator.cancelTask()

    def get_result(self) -> Optional[TaskResult]:
        """獲取導航結果（線程安全）"""
        with self._lock:
            if self.navigator is None or not self._nav2_ready:
                return None
            try:
                return self.navigator.getResult()
            except RuntimeError as e:
                logger.debug(f"Get result failed: {e}")
                return None

    def is_stuck(self) -> bool:
        """檢查機器人是否卡住（導航失敗）"""
        if not self.is_task_complete():
            return False
        result = self.get_result()
        if result is None:
            return False
        return result == TaskResult.FAILED


# Global navigator manager
nav_manager = NavigatorManager()


def _handle_navigation_down():
    """導航停止或崩潰後的共用善後：重置 navigator、將進行中的送餐任務標記為卡住"""
    try:
        nav_manager.reset()
    except Exception as e:
        logger.warning(f"Navigator reset failed: {e}")
    task = delivery_manager.mark_stuck()
    if task is not None and task.status == DeliveryTaskStatus.STUCK:
        logger.warning("Delivery task marked STUCK because navigation went down")


# 健康監控偵測到導航進程崩潰時的善後（在監控執行緒的鎖外呼叫）
state.on_navigation_down = _handle_navigation_down


def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == '__main__':
    main()
