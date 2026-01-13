import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
import threading
import subprocess
import os
import signal
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn
from typing import Optional
from enum import Enum
from contextlib import asynccontextmanager

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


# --- Thread-safe State Manager ---
class RobotStateManager:
    """Thread-safe manager for robot state and processes."""

    MAP_SAVE_PATH = "/home/jetson/base_dev/src/map/"
    HEALTH_CHECK_INTERVAL = 2.0  # 每 2 秒檢查一次

    def __init__(self):
        self._lock = threading.Lock()
        self._ros_node: Optional[Node] = None
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

    # --- ROS Node ---
    @property
    def ros_node(self):
        with self._lock:
            return self._ros_node

    @ros_node.setter
    def ros_node(self, value):
        with self._lock:
            self._ros_node = value

    # --- SLAM ---
    @property
    def slam_status(self) -> SlamStatus:
        with self._lock:
            return self._slam_status

    @slam_status.setter
    def slam_status(self, value: SlamStatus):
        with self._lock:
            self._slam_status = value

    def start_slam(self) -> dict:
        with self._lock:
            if self._slam_status == SlamStatus.MAPPING:
                raise HTTPException(status_code=400, detail="Mapping is already running.")

            try:
                self._slam_process = subprocess.Popen(
                    ["ros2", "launch", "nav2", "mapping.launch.py"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                self._slam_status = SlamStatus.MAPPING
                return {"message": "Mapping started.", "status": self._slam_status}
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to start mapping: {str(e)}")

    def stop_slam(self) -> dict:
        with self._lock:
            if self._slam_status != SlamStatus.MAPPING:
                raise HTTPException(status_code=400, detail="Mapping is not running.")

            try:
                if self._slam_process:
                    os.killpg(os.getpgid(self._slam_process.pid), signal.SIGTERM)
                    self._slam_process.wait(timeout=10)
                    self._slam_process = None
                self._slam_status = SlamStatus.IDLE
                return {"message": "Mapping stopped.", "status": self._slam_status}
            except Exception as e:
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

            try:
                if map_name:
                    map_yaml = os.path.join(self.MAP_SAVE_PATH, f"{map_name}.yaml")
                    if not os.path.exists(map_yaml):
                        raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found.")
                else:
                    map_yaml = os.path.join(self.MAP_SAVE_PATH, "map.yaml")

                self._nav_process = subprocess.Popen(
                    ["ros2", "launch", "nav2", "autonomous_navigation.launch.py", f"map:={map_yaml}"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                self._nav_status = NavStatus.RUNNING
                return {"message": f"Navigation started with map: {map_yaml}", "status": self._nav_status}
            except HTTPException:
                raise
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to start navigation: {str(e)}")

    def stop_navigation(self) -> dict:
        with self._lock:
            if self._nav_status != NavStatus.RUNNING:
                raise HTTPException(status_code=400, detail="Navigation is not running.")

            try:
                if self._nav_process:
                    os.killpg(os.getpgid(self._nav_process.pid), signal.SIGTERM)
                    self._nav_process.wait(timeout=10)
                    self._nav_process = None
                self._nav_status = NavStatus.IDLE
                return {"message": "Navigation stopped.", "status": self._nav_status}
            except Exception as e:
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
                self._robot_core_process = subprocess.Popen(
                    ["ros2", "launch", "motor_control", "full_system.launch.py"],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid
                )
                self._robot_core_running = True
                return {"message": "Robot core started.", "is_running": True}
            except Exception as e:
                raise HTTPException(status_code=500, detail=f"Failed to start robot core: {str(e)}")

    def stop_robot_core(self) -> dict:
        with self._lock:
            if not self._robot_core_running:
                raise HTTPException(status_code=400, detail="Robot core is not running.")

            try:
                if self._robot_core_process:
                    os.killpg(os.getpgid(self._robot_core_process.pid), signal.SIGTERM)
                    self._robot_core_process.wait(timeout=10)
                    self._robot_core_process = None
                self._robot_core_running = False
                return {"message": "Robot core stopped.", "is_running": False}
            except Exception as e:
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
        self.stop_health_monitor()
        with self._lock:
            for process, name in [
                (self._slam_process, "SLAM"),
                (self._nav_process, "Navigation"),
                (self._robot_core_process, "Robot Core")
            ]:
                if process:
                    try:
                        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
                        process.wait(timeout=5)
                    except Exception:
                        pass  # Best effort cleanup

            self._slam_process = None
            self._nav_process = None
            self._robot_core_process = None
            self._slam_status = SlamStatus.IDLE
            self._nav_status = NavStatus.IDLE
            self._robot_core_running = False


# --- Global State Manager Instance ---
state = RobotStateManager()


# --- Lifespan for cleanup ---
@asynccontextmanager
async def lifespan(app: FastAPI):
    state.start_health_monitor()
    yield
    state.cleanup()


# --- FastAPI App ---
app = FastAPI(title="Robot Control API", lifespan=lifespan)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# --- Robot Core Endpoints ---
@app.post("/robot/start")
async def start_robot_core():
    """Start robot core system (motor controller, LiDAR, IMU, EKF)."""
    return state.start_robot_core()

@app.post("/robot/stop")
async def stop_robot_core():
    """Stop robot core system."""
    return state.stop_robot_core()

@app.post("/robot/start_lidar")
async def start_lidar_motor():
    """Start LiDAR motor by calling /start_motor service."""
    try:
        result = subprocess.run(
            ["ros2", "service", "call", "/start_motor", "std_srvs/srv/Empty"],
            capture_output=True,
            text=True,
            timeout=10
        )
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
    ros_node = state.ros_node
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS2 node is not ready.")

    try:
        ros_node.get_logger().info(f"Received goal: x={goal.x}, y={goal.y}, yaw={goal.yaw_deg}")
        thread = threading.Thread(target=ros_node.send_goal_to_nav2, args=(goal.x, goal.y, goal.yaw_deg))
        thread.start()
        return {"message": "Goal received, navigation started."}
    except Exception as e:
        ros_node.get_logger().error(f"Failed to send goal: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/navigation/cancel")
async def cancel_navigation():
    """Cancel current navigation goal."""
    ros_node = state.ros_node
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS2 node is not ready.")

    try:
        ros_node.navigator.cancelTask()
        return {"message": "Navigation cancelled."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/navigation/status")
async def get_navigation_status():
    """Get current navigation status."""
    try:
        is_complete = True
        distance_remaining = None

        ros_node = state.ros_node
        if ros_node is not None:
            is_complete = ros_node.navigator.isTaskComplete()
            feedback = ros_node.navigator.getFeedback()
            distance_remaining = feedback.distance_remaining if feedback else None

        return {
            "is_complete": is_complete,
            "distance_remaining": distance_remaining,
            "nav_running": state.nav_status == NavStatus.RUNNING,
        }
    except Exception:
        return {"is_complete": True, "distance_remaining": None, "nav_running": state.nav_status == NavStatus.RUNNING}

@app.get("/maps/list")
async def list_maps():
    """List all available maps in the map directory."""
    try:
        maps = []
        if os.path.exists(state.MAP_SAVE_PATH):
            for file in os.listdir(state.MAP_SAVE_PATH):
                if file.endswith('.yaml'):
                    map_name = file[:-5]
                    yaml_path = os.path.join(state.MAP_SAVE_PATH, file)
                    pgm_path = os.path.join(state.MAP_SAVE_PATH, f"{map_name}.pgm")
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
    return state.start_navigation(map_name)

@app.post("/navigation/stop")
async def stop_navigation():
    """Stop autonomous navigation."""
    return state.stop_navigation()


# --- SLAM Endpoints ---
@app.post("/slam/start")
async def start_mapping():
    """Start SLAM mapping by launching mapping.launch.py."""
    return state.start_slam()

@app.post("/slam/stop")
async def stop_mapping():
    """Stop SLAM mapping."""
    return state.stop_slam()

@app.post("/slam/save_map")
async def save_map(request: MapSaveRequest):
    """Save the current map using nav2_map_server."""
    map_name = request.map_name.strip()
    if not map_name:
        raise HTTPException(status_code=400, detail="Map name is required.")

    map_name = "".join(c for c in map_name if c.isalnum() or c in ('-', '_'))
    map_path = os.path.join(state.MAP_SAVE_PATH, map_name)

    try:
        state.slam_status = SlamStatus.SAVING

        result = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path],
            capture_output=True,
            text=True,
            timeout=30
        )

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


# --- ROS2 Node ---
class ApiNavigatorNode(Node):
    def __init__(self):
        super().__init__('api_navigator_node')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active. API Navigator Node is ready.')

    def send_goal_to_nav2(self, x: float, y: float, yaw_deg: float):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y

        yaw_rad = math.radians(yaw_deg)
        goal_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self.get_logger().info(f"Sending goal to Nav2: Pose(x={x}, y={y}, yaw={yaw_deg})")
        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(f'Distance remaining: {feedback.distance_remaining:.2f} m')

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Goal failed!')


def run_ros_node():
    rclpy.init()
    node = ApiNavigatorNode()
    state.ros_node = node
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main():
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.daemon = True
    ros_thread.start()

    uvicorn.run(app, host="0.0.0.0", port=8000)


if __name__ == '__main__':
    main()
