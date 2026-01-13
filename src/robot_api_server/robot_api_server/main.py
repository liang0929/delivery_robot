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

# --- Pydantic Models ---
class Goal(BaseModel):
    x: float
    y: float
    yaw_deg: float

class MapSaveRequest(BaseModel):
    map_name: str

class NavigationStartRequest(BaseModel):
    map_name: Optional[str] = None  # 如果為 None，使用預設地圖

class SlamStatus(str, Enum):
    IDLE = "idle"
    MAPPING = "mapping"
    SAVING = "saving"

class NavStatus(str, Enum):
    IDLE = "idle"
    RUNNING = "running"

# --- FastAPI App ---
app = FastAPI(title="Robot Control API")

# Enable CORS for web frontend
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global state
ros_node = None
slam_process: Optional[subprocess.Popen] = None
slam_status = SlamStatus.IDLE
nav_process: Optional[subprocess.Popen] = None
nav_status = NavStatus.IDLE
robot_core_process: Optional[subprocess.Popen] = None
robot_core_running = False
MAP_SAVE_PATH = "/home/jetson/base_dev/src/map/"

# --- Robot Core Endpoints ---
@app.post("/robot/start")
async def start_robot_core():
    """Start robot core system (motor controller, LiDAR, IMU, EKF)."""
    global robot_core_process, robot_core_running

    if robot_core_running:
        raise HTTPException(status_code=400, detail="Robot core is already running.")

    try:
        robot_core_process = subprocess.Popen(
            ["ros2", "launch", "motor_control", "full_system.launch.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        robot_core_running = True
        return {"message": "Robot core started.", "is_running": True}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to start robot core: {str(e)}")

@app.post("/robot/stop")
async def stop_robot_core():
    """Stop robot core system."""
    global robot_core_process, robot_core_running

    if not robot_core_running:
        raise HTTPException(status_code=400, detail="Robot core is not running.")

    try:
        if robot_core_process:
            os.killpg(os.getpgid(robot_core_process.pid), signal.SIGTERM)
            robot_core_process.wait(timeout=10)
            robot_core_process = None
        robot_core_running = False
        return {"message": "Robot core stopped.", "is_running": False}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to stop robot core: {str(e)}")

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
    return {
        "is_running": robot_core_running,
    }

# --- Navigation Endpoints ---
@app.post("/navigate_to_goal")
async def navigate_to_goal(goal: Goal):
    """Send navigation goal to Nav2 stack."""
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

        if ros_node is not None:
            is_complete = ros_node.navigator.isTaskComplete()
            feedback = ros_node.navigator.getFeedback()
            distance_remaining = feedback.distance_remaining if feedback else None

        return {
            "is_complete": is_complete,
            "distance_remaining": distance_remaining,
            "nav_running": nav_status == NavStatus.RUNNING,
        }
    except Exception as e:
        return {"is_complete": True, "distance_remaining": None, "nav_running": nav_status == NavStatus.RUNNING}

@app.get("/maps/list")
async def list_maps():
    """List all available maps in the map directory."""
    try:
        maps = []
        if os.path.exists(MAP_SAVE_PATH):
            for file in os.listdir(MAP_SAVE_PATH):
                if file.endswith('.yaml'):
                    map_name = file[:-5]  # 移除 .yaml 副檔名
                    yaml_path = os.path.join(MAP_SAVE_PATH, file)
                    pgm_path = os.path.join(MAP_SAVE_PATH, f"{map_name}.pgm")
                    # 只有 yaml 和 pgm 都存在才算有效地圖
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
    global nav_process, nav_status

    if nav_status == NavStatus.RUNNING:
        raise HTTPException(status_code=400, detail="Navigation is already running.")

    if slam_status == SlamStatus.MAPPING:
        raise HTTPException(status_code=400, detail="Cannot start navigation while mapping is running. Stop mapping first.")

    try:
        # 決定地圖路徑
        if request and request.map_name:
            map_yaml = os.path.join(MAP_SAVE_PATH, f"{request.map_name}.yaml")
            if not os.path.exists(map_yaml):
                raise HTTPException(status_code=404, detail=f"Map '{request.map_name}' not found.")
        else:
            map_yaml = os.path.join(MAP_SAVE_PATH, "map.yaml")

        nav_process = subprocess.Popen(
            ["ros2", "launch", "nav2", "autonomous_navigation.launch.py", f"map:={map_yaml}"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        nav_status = NavStatus.RUNNING
        return {"message": f"Navigation started with map: {map_yaml}", "status": nav_status}
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to start navigation: {str(e)}")

@app.post("/navigation/stop")
async def stop_navigation():
    """Stop autonomous navigation."""
    global nav_process, nav_status

    if nav_status != NavStatus.RUNNING:
        raise HTTPException(status_code=400, detail="Navigation is not running.")

    try:
        if nav_process:
            os.killpg(os.getpgid(nav_process.pid), signal.SIGTERM)
            nav_process.wait(timeout=10)
            nav_process = None
        nav_status = NavStatus.IDLE
        return {"message": "Navigation stopped.", "status": nav_status}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to stop navigation: {str(e)}")

# --- SLAM Endpoints ---
@app.post("/slam/start")
async def start_mapping():
    """Start SLAM mapping by launching mapping.launch.py."""
    global slam_process, slam_status

    if slam_status == SlamStatus.MAPPING:
        raise HTTPException(status_code=400, detail="Mapping is already running.")

    try:
        # Launch the mapping launch file
        slam_process = subprocess.Popen(
            ["ros2", "launch", "nav2", "mapping.launch.py"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid
        )
        slam_status = SlamStatus.MAPPING
        return {"message": "Mapping started.", "status": slam_status}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to start mapping: {str(e)}")

@app.post("/slam/stop")
async def stop_mapping():
    """Stop SLAM mapping."""
    global slam_process, slam_status

    if slam_status != SlamStatus.MAPPING:
        raise HTTPException(status_code=400, detail="Mapping is not running.")

    try:
        if slam_process:
            os.killpg(os.getpgid(slam_process.pid), signal.SIGTERM)
            slam_process.wait(timeout=10)
            slam_process = None
        slam_status = SlamStatus.IDLE
        return {"message": "Mapping stopped.", "status": slam_status}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to stop mapping: {str(e)}")

@app.post("/slam/save_map")
async def save_map(request: MapSaveRequest):
    """Save the current map using nav2_map_server."""
    global slam_status

    map_name = request.map_name.strip()
    if not map_name:
        raise HTTPException(status_code=400, detail="Map name is required.")

    map_name = "".join(c for c in map_name if c.isalnum() or c in ('-', '_'))
    map_path = os.path.join(MAP_SAVE_PATH, map_name)

    try:
        slam_status = SlamStatus.SAVING

        result = subprocess.run(
            ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", map_path],
            capture_output=True,
            text=True,
            timeout=30
        )

        if result.returncode != 0:
            slam_status = SlamStatus.MAPPING
            raise HTTPException(status_code=500, detail=f"Map save failed: {result.stderr}")

        slam_status = SlamStatus.MAPPING
        return {
            "message": "Map saved successfully.",
            "map_path": map_path,
            "files": [f"{map_path}.pgm", f"{map_path}.yaml"]
        }
    except subprocess.TimeoutExpired:
        slam_status = SlamStatus.MAPPING
        raise HTTPException(status_code=500, detail="Map save timed out.")
    except Exception as e:
        slam_status = SlamStatus.MAPPING
        raise HTTPException(status_code=500, detail=f"Failed to save map: {str(e)}")

@app.get("/slam/status")
async def get_slam_status():
    """Get current SLAM status."""
    return {
        "status": slam_status,
        "is_mapping": slam_status == SlamStatus.MAPPING,
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
    global ros_node
    rclpy.init()
    ros_node = ApiNavigatorNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

def main():
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.daemon = True
    ros_thread.start()

    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == '__main__':
    main()
