import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
import threading
import subprocess
import os
import signal
import logging
import time
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
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


# --- Configuration ---
# 從環境變數讀取配置，提供合理預設值
DEFAULT_MAP_PATH = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'map')
MAP_SAVE_PATH = os.environ.get('ROBOT_MAP_PATH', os.path.abspath(DEFAULT_MAP_PATH))

# 確保地圖目錄存在
os.makedirs(MAP_SAVE_PATH, exist_ok=True)
logger.info(f"Map save path: {MAP_SAVE_PATH}")


# --- Waypoint File Operations ---
def get_waypoints_file_path(map_name: str) -> str:
    """取得 waypoints 檔案路徑"""
    safe_name = "".join(c for c in map_name if c.isalnum() or c in ('-', '_'))
    if not safe_name:
        raise HTTPException(status_code=400, detail="Invalid map name.")
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
    """取得完整系統狀態"""
    is_complete = nav_manager.is_task_complete()
    feedback = nav_manager.get_feedback()
    distance_remaining = feedback.distance_remaining if feedback else None

    return {
        "robot_core": {
            "running": state.robot_core_running,
        },
        "slam": {
            "status": state.slam_status.value,
            "is_mapping": state.slam_status == SlamStatus.MAPPING,
        },
        "navigation": {
            "status": state.nav_status.value,
            "nav_running": state.nav_status == NavStatus.RUNNING,
            "is_complete": is_complete,
            "distance_remaining": distance_remaining,
        },
        "crash_info": state.crash_info,
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

logger.info(f"CORS allowed origins: {ALLOWED_ORIGINS}")

app.add_middleware(
    CORSMiddleware,
    allow_origins=ALLOWED_ORIGINS,
    allow_credentials=True,
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

@app.post("/navigation/start")
async def start_navigation(request: NavigationStartRequest = None):
    """Start autonomous navigation by launching autonomous_navigation.launch.py."""
    map_name = request.map_name if request else None
    return await asyncio.to_thread(state.start_navigation, map_name)

@app.post("/navigation/stop")
async def stop_navigation():
    """Stop autonomous navigation."""
    return await asyncio.to_thread(state.stop_navigation)


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
    """驗證並返回安全的地圖路徑，防止路徑注入攻擊"""
    # 清理檔名：只允許字母數字和 -_
    safe_name = "".join(c for c in map_name if c.isalnum() or c in ('-', '_'))

    if not safe_name:
        raise HTTPException(status_code=400, detail="Invalid map name after sanitization.")

    # 限制檔名長度
    if len(safe_name) > 64:
        raise HTTPException(status_code=400, detail="Map name too long (max 64 characters).")

    # 構建完整路徑並規範化
    full_path = os.path.normpath(os.path.join(base_path, safe_name))

    # 確保路徑在預期目錄內（防止目錄遍歷）
    base_path_normalized = os.path.normpath(base_path)
    if not full_path.startswith(base_path_normalized + os.sep) and full_path != base_path_normalized:
        raise HTTPException(status_code=400, detail="Invalid map path.")

    return full_path


@app.post("/slam/save_map")
async def save_map(request: MapSaveRequest):
    """Save the current map using nav2_map_server."""
    map_name = request.map_name.strip()
    if not map_name:
        raise HTTPException(status_code=400, detail="Map name is required.")

    # 使用安全的路徑驗證函數
    map_path = validate_map_path(map_name, MAP_SAVE_PATH)

    def _save_map():
        return subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path, "--ros-args", "-p", "map_topic:=/map"],
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
        """發送導航目標（非阻塞）"""
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
        if self.navigator is None or not self._nav2_ready:
            return True
        try:
            return self.navigator.isTaskComplete()
        except RuntimeError as e:
            logger.debug(f"Task complete check failed: {e}")
            return True

    def get_feedback(self):
        if self.navigator is None or not self._nav2_ready:
            return None
        try:
            return self.navigator.getFeedback()
        except RuntimeError as e:
            logger.debug(f"Get feedback failed: {e}")
            return None

    def cancel_task(self):
        if self.navigator is not None:
            self.navigator.cancelTask()


# Global navigator manager
nav_manager = NavigatorManager()


def main():
    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == '__main__':
    main()
