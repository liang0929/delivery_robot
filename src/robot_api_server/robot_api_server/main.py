import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math
import threading
import subprocess
import os
import signal
import logging
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

# --- Logging Setup ---
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)

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
            logging.getLogger(__name__).info("rclpy initialized successfully")
            return True
        except RuntimeError:
            # rclpy 已被初始化（可能在其他進程/線程中）
            _rclpy_initialized = True
            logging.getLogger(__name__).info("rclpy already initialized elsewhere")
            return True
        except Exception as e:
            logging.getLogger(__name__).error(f"Failed to initialize rclpy: {e}")
            return False
logger = logging.getLogger(__name__)

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
    MAPPING = "mapping"
    SAVING = "saving"

class NavStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"


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

    def __init__(self):
        self._lock = threading.Lock()
        self._slam_process: Optional[subprocess.Popen] = None
        self._slam_status = SlamStatus.IDLE
        self._nav_process: Optional[subprocess.Popen] = None
        self._nav_status = NavStatus.IDLE
        self._current_map: Optional[str] = None  # 當前導航使用的地圖
        self._robot_core_process: Optional[subprocess.Popen] = None
        self._robot_core_running = False
        # Health monitor
        self._health_thread: Optional[threading.Thread] = None
        self._health_stop_event = threading.Event()
        self._crash_info: dict = {}  # 記錄 crash 資訊

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
        """清理所有導航相關殘留進程"""
        patterns = ["nav2_", "autonomous_navigation.launch", "basic_navigator"]
        for pattern in patterns:
            try:
                subprocess.run(["pkill", "-f", pattern], capture_output=True)
            except Exception as e:
                logger.warning(f"Failed to kill processes matching '{pattern}': {e}")

        # 等待進程真正終止，避免競態條件
        if not self._wait_for_process_cleanup(patterns, timeout=3.0):
            logger.warning("Some navigation processes may still be running after cleanup")

    def _cleanup_slam_processes(self):
        """清理所有建圖相關殘留進程"""
        patterns = ["slam_toolbox", "mapping.launch", "map_relay"]
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
        with self._lock:
            if self._slam_status == SlamStatus.MAPPING:
                raise HTTPException(status_code=400, detail="Mapping is already running.")

            # 先清理可能殘留的導航進程
            self._cleanup_nav_processes()

            try:
                # 使用 DEVNULL 避免管道緩衝區滿導致死鎖
                self._slam_process = subprocess.Popen(
                    ["ros2", "launch", "nav2", "mapping.launch.py"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid
                )

                # 驗證進程確實啟動成功
                if not self._verify_process_started(self._slam_process, "SLAM"):
                    self._slam_process = None
                    raise HTTPException(status_code=500, detail="SLAM process failed to start.")

                self._slam_status = SlamStatus.MAPPING
                return {"message": "Mapping started.", "status": self._slam_status}
            except HTTPException:
                raise
            except Exception as e:
                self._slam_process = None
                raise HTTPException(status_code=500, detail=f"Failed to start mapping: {str(e)}")

    def stop_slam(self) -> dict:
        with self._lock:
            if self._slam_status != SlamStatus.MAPPING:
                raise HTTPException(status_code=400, detail="Mapping is not running.")

            try:
                if self._slam_process:
                    if not self._terminate_process_safely(self._slam_process, "SLAM"):
                        logger.error("SLAM process may still be running")
                    self._slam_process = None
                self._slam_status = SlamStatus.IDLE
                return {"message": "Mapping stopped.", "status": self._slam_status}
            except Exception as e:
                logger.error(f"Failed to stop mapping: {e}")
                raise HTTPException(status_code=500, detail=f"Failed to stop mapping: {str(e)}")

    # --- Navigation ---
    @property
    def nav_status(self) -> NavStatus:
        with self._lock:
            return self._nav_status

    def start_navigation(self, map_name: Optional[str] = None) -> dict:
        with self._lock:
            if self._nav_status == NavStatus.RUNNING:
                raise HTTPException(status_code=400, detail="Navigation is already running.")

            if self._slam_status == SlamStatus.MAPPING:
                raise HTTPException(status_code=400, detail="Cannot start navigation while mapping is running.")

            # 先清理可能殘留的建圖和導航進程
            self._cleanup_slam_processes()
            self._cleanup_nav_processes()

            try:
                if map_name:
                    map_yaml = os.path.join(MAP_SAVE_PATH, f"{map_name}.yaml")
                    if not os.path.exists(map_yaml):
                        raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found.")
                else:
                    map_yaml = os.path.join(MAP_SAVE_PATH, "map.yaml")

                # 使用 DEVNULL 避免管道緩衝區滿導致死鎖
                self._nav_process = subprocess.Popen(
                    ["ros2", "launch", "nav2", "autonomous_navigation.launch.py", f"map:={map_yaml}"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid
                )

                # 驗證進程確實啟動成功
                if not self._verify_process_started(self._nav_process, "Navigation"):
                    self._nav_process = None
                    raise HTTPException(status_code=500, detail="Navigation process failed to start.")

                self._nav_status = NavStatus.RUNNING
                # 記錄當前使用的地圖名稱
                self._current_map = map_name if map_name else "map"
                return {"message": f"Navigation started with map: {map_yaml}", "status": self._nav_status}
            except HTTPException:
                raise
            except Exception as e:
                self._nav_process = None
                raise HTTPException(status_code=500, detail=f"Failed to start navigation: {str(e)}")

    def stop_navigation(self) -> dict:
        with self._lock:
            if self._nav_status != NavStatus.RUNNING:
                raise HTTPException(status_code=400, detail="Navigation is not running.")

            try:
                if self._nav_process:
                    if not self._terminate_process_safely(self._nav_process, "Navigation"):
                        logger.error("Navigation process may still be running")
                    self._nav_process = None

                self._nav_status = NavStatus.IDLE
                return {"message": "Navigation stopped.", "status": self._nav_status}
            except Exception as e:
                logger.error(f"Failed to stop navigation: {e}")
                raise HTTPException(status_code=500, detail=f"Failed to stop navigation: {str(e)}")

    # --- Robot Core ---
    @property
    def robot_core_running(self) -> bool:
        with self._lock:
            return self._robot_core_running

    def start_robot_core(self) -> dict:
        with self._lock:
            if self._robot_core_running:
                raise HTTPException(status_code=400, detail="Robot core is already running.")

            try:
                # 使用 DEVNULL 避免管道緩衝區滿導致死鎖
                self._robot_core_process = subprocess.Popen(
                    ["ros2", "launch", "motor_control", "bringup.launch.py", "enable_web:=false"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    preexec_fn=os.setsid
                )

                # 驗證進程確實啟動成功
                if not self._verify_process_started(self._robot_core_process, "Robot Core"):
                    self._robot_core_process = None
                    raise HTTPException(status_code=500, detail="Robot core process failed to start.")

                self._robot_core_running = True
                return {"message": "Robot core started.", "is_running": True}
            except HTTPException:
                raise
            except Exception as e:
                self._robot_core_process = None
                raise HTTPException(status_code=500, detail=f"Failed to start robot core: {str(e)}")

    def stop_robot_core(self) -> dict:
        with self._lock:
            if not self._robot_core_running:
                raise HTTPException(status_code=400, detail="Robot core is not running.")

            try:
                if self._robot_core_process:
                    if not self._terminate_process_safely(self._robot_core_process, "Robot Core"):
                        logger.error("Robot Core process may still be running")
                    self._robot_core_process = None
                self._robot_core_running = False
                return {"message": "Robot core stopped.", "is_running": False}
            except Exception as e:
                logger.error(f"Failed to stop robot core: {e}")
                raise HTTPException(status_code=500, detail=f"Failed to stop robot core: {str(e)}")

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
            self._check_processes()
            self._health_stop_event.wait(self.HEALTH_CHECK_INTERVAL)

    def _check_processes(self):
        """檢查所有子程序是否存活"""
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
        with self._lock:
            for process, name in [
                (self._slam_process, "SLAM"),
                (self._nav_process, "Navigation"),
                (self._robot_core_process, "Robot Core")
            ]:
                if process:
                    self._terminate_process_safely(process, name, timeout=3)

            self._slam_process = None
            self._nav_process = None
            self._robot_core_process = None
            self._slam_status = SlamStatus.IDLE
            self._nav_status = NavStatus.IDLE
            self._robot_core_running = False
        logger.info("Cleanup completed")

    def get_status_snapshot(self) -> dict:
        """線程安全地獲取完整狀態快照"""
        with self._lock:
            return {
                "robot_core_running": self._robot_core_running,
                "slam_status": self._slam_status,
                "nav_status": self._nav_status,
                "crash_info": self._crash_info.copy(),
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
        """廣播訊息給所有連線"""
        async with self._lock:
            dead_connections = set()
            for conn in self._connections:
                try:
                    await conn.send_json(message)
                except (ConnectionError, RuntimeError) as e:
                    logger.debug(f"WebSocket connection lost: {e}")
                    dead_connections.add(conn)
            # 移除斷線的連線
            self._connections -= dead_connections

    @property
    def connection_count(self) -> int:
        return len(self._connections)


ws_manager = ConnectionManager()


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
        "crash_info": snapshot["crash_info"],
    }


# --- Lifespan for cleanup ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    state.start_health_monitor()
    # 啟動狀態廣播任務
    broadcast_task = asyncio.create_task(status_broadcast_loop())
    yield
    broadcast_task.cancel()
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
        # 連線時立即發送當前狀態
        await websocket.send_json({
            "type": "status_update",
            "data": get_full_status()
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

        # 解析 PGM 檔案獲取尺寸
        width, height = 0, 0
        if os.path.exists(pgm_path):
            with open(pgm_path, 'rb') as f:
                # 讀取 PGM header
                magic = f.readline().decode().strip()
                if magic in ['P5', 'P2']:
                    # 跳過註解
                    line = f.readline().decode().strip()
                    while line.startswith('#'):
                        line = f.readline().decode().strip()
                    # 讀取尺寸
                    parts = line.split()
                    width, height = int(parts[0]), int(parts[1])

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
        logger.error(f"Failed to read map metadata: {e}")
        raise HTTPException(status_code=500, detail=f"Failed to read map metadata: {str(e)}")

@app.post("/navigation/start")
async def start_navigation(request: NavigationStartRequest = None):
    """Start autonomous navigation by launching autonomous_navigation.launch.py."""
    map_name = request.map_name if request else None
    return await asyncio.to_thread(state.start_navigation, map_name)

@app.post("/navigation/stop")
async def stop_navigation():
    """Stop autonomous navigation."""
    return await asyncio.to_thread(state.stop_navigation)

@app.post("/navigation/set_initial_pose")
async def set_initial_pose(request: InitialPoseRequest):
    """Set the initial pose for AMCL localization."""
    def _set_pose():
        try:
            ensure_rclpy_initialized()
            node = rclpy.create_node('initial_pose_publisher')
            publisher = node.create_publisher(
                PoseWithCovarianceStamped,
                '/initialpose',
                10
            )

            # 等待訂閱者
            time.sleep(0.5)

            msg = PoseWithCovarianceStamped()
            msg.header.frame_id = 'map'
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.pose.position.x = request.x
            msg.pose.pose.position.y = request.y
            msg.pose.pose.position.z = 0.0

            # 從 yaw 計算四元數
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = math.sin(request.yaw / 2.0)
            msg.pose.pose.orientation.w = math.cos(request.yaw / 2.0)

            # 設置協方差（對角線元素）
            msg.pose.covariance[0] = 0.25  # x
            msg.pose.covariance[7] = 0.25  # y
            msg.pose.covariance[35] = 0.06853891945200942  # yaw

            publisher.publish(msg)
            logger.info(f"Published initial pose: x={request.x}, y={request.y}, yaw={request.yaw}")

            # 等待消息發送
            time.sleep(0.5)

            node.destroy_node()
            return {"message": "Initial pose set successfully", "x": request.x, "y": request.y, "yaw": request.yaw}
        except Exception as e:
            logger.error(f"Failed to set initial pose: {e}")
            raise HTTPException(status_code=500, detail=f"Failed to set initial pose: {str(e)}")

    if state.nav_status != NavStatus.RUNNING:
        raise HTTPException(status_code=400, detail="Navigation is not running. Start navigation first.")

    return await asyncio.to_thread(_set_pose)


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
        cmd = f"source /opt/ros/humble/setup.bash && source {WORKSPACE_ROOT}/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f {map_path} -t /map_saver --ros-args -p map_subscribe_transient_local:=true"
        return subprocess.run(
            ["bash", "-c", cmd],
            capture_output=True,
            text=True,
            timeout=30
        )

    try:
        state.slam_status = SlamStatus.SAVING
        result = await asyncio.to_thread(_save_map)

        if result.returncode != 0:
            state.slam_status = SlamStatus.MAPPING
            raise HTTPException(status_code=500, detail=f"Map save failed: {result.stderr}")

        state.slam_status = SlamStatus.MAPPING
        return {
            "message": "Map saved successfully.",
            "map_path": map_path,
            "files": [f"{map_path}.pgm", f"{map_path}.yaml"]
        }
    except subprocess.TimeoutExpired:
        state.slam_status = SlamStatus.MAPPING
        raise HTTPException(status_code=500, detail="Map save timed out.")
    except HTTPException:
        raise
    except Exception as e:
        state.slam_status = SlamStatus.MAPPING
        raise HTTPException(status_code=500, detail=f"Failed to save map: {str(e)}")

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
    # 驗證地圖存在
    map_yaml = os.path.join(MAP_SAVE_PATH, f"{map_name}.yaml")
    if not os.path.exists(map_yaml):
        raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found.")
    return load_waypoints(map_name)


@app.post("/maps/{map_name}/waypoints", response_model=Waypoint)
async def create_waypoint(map_name: str, waypoint: WaypointCreate):
    """Create a new waypoint for a specific map."""
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
    tables = load_tables(map_name)
    return tables


@app.post("/maps/{map_name}/tables", response_model=Table)
async def create_table(map_name: str, table: TableCreate):
    """Create a new table."""
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

    @property
    def current_task(self) -> Optional[DeliveryTask]:
        with self._lock:
            return self._current_task

    def start_delivery(self, map_name: str, table_ids: List[str], start_position: Position) -> DeliveryTask:
        """開始送餐任務"""
        with self._lock:
            if self._current_task is not None and self._current_task.status != DeliveryTaskStatus.IDLE:
                raise HTTPException(status_code=400, detail="A delivery task is already in progress.")

            # 載入桌位資訊
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

            return task

    def get_current_stop_table(self) -> Optional[Table]:
        """取得當前站點的桌位資訊"""
        with self._lock:
            if self._current_task is None or self._current_map is None:
                return None
            if self._current_task.currentStopIndex >= len(self._current_task.stops):
                return None

            stop = self._current_task.stops[self._current_task.currentStopIndex]
            tables = load_tables(self._current_map)
            for t in tables:
                if t.id == stop.tableId:
                    return t
            return None

    def mark_arrived(self) -> Optional[DeliveryTask]:
        """標記已到達當前桌位"""
        with self._lock:
            if self._current_task is None:
                return None
            if self._current_task.status != DeliveryTaskStatus.DELIVERING:
                return self._current_task

            stop = self._current_task.stops[self._current_task.currentStopIndex]
            stop.status = DeliveryStopStatus.ARRIVED
            self._current_task.status = DeliveryTaskStatus.AT_TABLE

            return self._current_task

    def confirm_arrival(self) -> Optional[DeliveryTask]:
        """確認到達並前往下一桌"""
        with self._lock:
            if self._current_task is None:
                return None
            if self._current_task.status != DeliveryTaskStatus.AT_TABLE:
                return self._current_task

            # 標記當前站點完成
            stop = self._current_task.stops[self._current_task.currentStopIndex]
            stop.status = DeliveryStopStatus.COMPLETED

            # 前往下一桌
            return self._advance_to_next()

    def skip_table(self) -> Optional[DeliveryTask]:
        """跳過當前桌位"""
        with self._lock:
            if self._current_task is None:
                return None
            if self._current_task.status != DeliveryTaskStatus.AT_TABLE:
                return self._current_task

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

        return self._current_task

    def complete_return(self) -> Optional[DeliveryTask]:
        """完成返回"""
        with self._lock:
            if self._current_task is None:
                return None

            self._current_task.status = DeliveryTaskStatus.IDLE
            task = self._current_task
            self._current_task = None
            self._current_map = None

            return task

    def cancel_delivery(self):
        """取消送餐任務"""
        with self._lock:
            self._current_task = None
            self._current_map = None


# 全局送餐管理器
delivery_manager = DeliveryManager()


# --- Delivery API ---
@app.post("/delivery/start", response_model=DeliveryTask)
async def start_delivery(request: DeliveryStartRequest):
    """Start a delivery task."""
    # 決定使用哪個地圖
    if request.mapName:
        map_name = request.mapName
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
            # 等待導航系統完全啟動
            logger.info("Waiting for navigation system to initialize...")
            await asyncio.sleep(5)  # 給 Nav2 一些啟動時間
            nav_just_started = True
        except HTTPException as e:
            raise HTTPException(status_code=500, detail=f"Failed to auto-start navigation: {e.detail}")
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"Failed to auto-start navigation: {str(e)}")

    # 如果導航剛啟動，使用前端傳來的起點位置設定初始位置
    if nav_just_started:
        logger.info(f"Setting initial pose to start position: x={request.startPosition.x}, y={request.startPosition.y}, yaw={request.startPosition.yaw}")
        try:
            def _set_initial_pose():
                ensure_rclpy_initialized()
                node = rclpy.create_node('delivery_initial_pose_publisher')
                publisher = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
                time.sleep(0.5)

                msg = PoseWithCovarianceStamped()
                msg.header.frame_id = 'map'
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.pose.position.x = request.startPosition.x
                msg.pose.pose.position.y = request.startPosition.y
                msg.pose.pose.position.z = 0.0
                msg.pose.pose.orientation.z = math.sin(request.startPosition.yaw / 2.0)
                msg.pose.pose.orientation.w = math.cos(request.startPosition.yaw / 2.0)
                # 較小的協方差 = 更確定的位置
                msg.pose.covariance[0] = 0.1   # x
                msg.pose.covariance[7] = 0.1   # y
                msg.pose.covariance[35] = 0.05  # yaw

                publisher.publish(msg)
                time.sleep(0.3)
                node.destroy_node()

            await asyncio.to_thread(_set_initial_pose)
            logger.info("Initial pose set successfully")
            await asyncio.sleep(1)  # 給 AMCL 時間處理
        except Exception as e:
            logger.warning(f"Failed to set initial pose: {e}")

    task = delivery_manager.start_delivery(map_name, request.tableIds, request.startPosition)

    # 導航到第一個桌位
    first_table = delivery_manager.get_current_stop_table()
    if first_table:
        is_ready = await asyncio.to_thread(nav_manager.ensure_nav2_ready)
        if is_ready:
            await asyncio.to_thread(nav_manager.send_goal, first_table.x, first_table.y, first_table.yaw_deg)
            logger.info(f"Starting delivery to table {first_table.number}")

    return task


@app.post("/delivery/confirm", response_model=DeliveryTask)
async def confirm_delivery_arrival():
    """Confirm arrival at current table and proceed to next."""
    task = delivery_manager.confirm_arrival()
    if task is None:
        raise HTTPException(status_code=400, detail="No active delivery task.")

    # 導航到下一個桌位或返回出發點
    if task.status == DeliveryTaskStatus.DELIVERING:
        next_table = delivery_manager.get_current_stop_table()
        if next_table:
            is_ready = await asyncio.to_thread(nav_manager.ensure_nav2_ready)
            if is_ready:
                await asyncio.to_thread(nav_manager.send_goal, next_table.x, next_table.y, next_table.yaw_deg)
                logger.info(f"Proceeding to table {next_table.number}")
    elif task.status == DeliveryTaskStatus.RETURNING:
        # 返回出發點
        is_ready = await asyncio.to_thread(nav_manager.ensure_nav2_ready)
        if is_ready:
            await asyncio.to_thread(nav_manager.send_goal, task.startPosition.x, task.startPosition.y, task.startPosition.yaw)
            logger.info("Returning to start position")

    return task


@app.post("/delivery/skip", response_model=DeliveryTask)
async def skip_delivery_table():
    """Skip current table and proceed to next."""
    task = delivery_manager.skip_table()
    if task is None:
        raise HTTPException(status_code=400, detail="No active delivery task.")

    # 導航到下一個桌位或返回出發點
    if task.status == DeliveryTaskStatus.DELIVERING:
        next_table = delivery_manager.get_current_stop_table()
        if next_table:
            is_ready = await asyncio.to_thread(nav_manager.ensure_nav2_ready)
            if is_ready:
                await asyncio.to_thread(nav_manager.send_goal, next_table.x, next_table.y, next_table.yaw_deg)
                logger.info(f"Proceeding to table {next_table.number}")
    elif task.status == DeliveryTaskStatus.RETURNING:
        # 返回出發點
        is_ready = await asyncio.to_thread(nav_manager.ensure_nav2_ready)
        if is_ready:
            await asyncio.to_thread(nav_manager.send_goal, task.startPosition.x, task.startPosition.y, task.startPosition.yaw)
            logger.info("Returning to start position")

    return task


@app.post("/delivery/cancel")
async def cancel_delivery():
    """Cancel the current delivery task."""
    delivery_manager.cancel_delivery()
    nav_manager.cancel_task()
    logger.info("Delivery cancelled")
    return {"message": "Delivery cancelled."}


@app.get("/delivery/status")
async def get_delivery_status():
    """Get current delivery task status."""
    task = delivery_manager.current_task

    distance_remaining = None
    is_stuck = False

    if task and task.status in [DeliveryTaskStatus.DELIVERING, DeliveryTaskStatus.RETURNING]:
        feedback = nav_manager.get_feedback()
        if feedback and hasattr(feedback, 'distance_remaining'):
            distance_remaining = feedback.distance_remaining

        # 檢查是否卡住
        if nav_manager.is_stuck():
            is_stuck = True
            task.status = DeliveryTaskStatus.STUCK
            logger.warning(f"Robot is stuck! Task status changed to STUCK")
        # 檢查是否到達
        elif nav_manager.is_task_complete():
            if task.status == DeliveryTaskStatus.DELIVERING:
                delivery_manager.mark_arrived()
                task = delivery_manager.current_task
            elif task.status == DeliveryTaskStatus.RETURNING:
                delivery_manager.complete_return()
                task = None

    # 如果已經是卡住狀態，繼續回報
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

    def ensure_nav2_ready(self) -> bool:
        """確保 Nav2 已準備好"""
        with self._lock:
            if self._nav2_ready and self.navigator is not None:
                return True

            try:
                if self.navigator is None:
                    # 使用全局線程安全的 rclpy 初始化
                    if not ensure_rclpy_initialized():
                        logger.error("Failed to initialize rclpy")
                        return False

                    logger.info("Creating BasicNavigator...")
                    self.navigator = BasicNavigator()

                logger.info("Waiting for Nav2 to become active...")
                self.navigator.waitUntilNav2Active()
                self._nav2_ready = True
                logger.info("Nav2 is active.")
                return True
            except RuntimeError as e:
                logger.error(f"Failed to connect to Nav2: {e}")
                return False

    def reset(self):
        """重置 navigator"""
        with self._lock:
            self._nav2_ready = False
            if self.navigator is not None:
                try:
                    self.navigator.lifecycleShutdown()
                except RuntimeError as e:
                    logger.warning(f"Error during navigator shutdown: {e}")
                self.navigator = None

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
                logger.debug(f"Task complete check failed: {e}")
                return True

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
            if self.navigator is not None:
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


def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == '__main__':
    main()
