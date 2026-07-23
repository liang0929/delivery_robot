"""rclpy 全域初始化與 ROS 套件的集中降級 import。

拆分自 ``ros_bridge.py``：``process_manager.py`` / ``navigator.py`` /
``bridge_node.py`` 都需要「ROS 是否可用」與 rclpy 初始化這兩件事，集中在
這裡避免各模組各自重複一份 try/except 降級邏輯、也避免各模組對同一批
ROS 套件重複 import。

所有 ROS 型別在 import 失敗時一律保留為 ``None``（而非完全不存在於本
模組命名空間），讓其他模組可以無條件 ``from .ros_common import X``，
不因為執行環境沒有 ROS 而在 import 期就整條鏈炸掉——degraded 模式的
「優雅降級」精神與原本 ``ros_bridge.py`` 逐字相同，只是把「未定義」
改成「顯式為 None」以支援跨模組 import。
"""

import threading

from .logging_config import get_logger

logger = get_logger(__name__)


# --- ROS 相依：不可用時優雅降級 ---
ROS_AVAILABLE = True
ROS_IMPORT_ERROR = None

rclpy = None
PoseStamped = None
PoseWithCovarianceStamped = None
Twist = None
OccupancyGrid = None
SingleThreadedExecutor = None
DurabilityPolicy = None
QoSProfile = None
ReliabilityPolicy = None
Bool = None
Float32 = None

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
BasicNavigator = None
TaskResult = None
try:
    if ROS_AVAILABLE:
        from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
except Exception as e:  # pragma: no cover
    NAV2_AVAILABLE = False
    logger.error(f"nav2_simple_commander unavailable: {e}")


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


__all__ = [
    'ROS_AVAILABLE', 'ROS_IMPORT_ERROR', 'NAV2_AVAILABLE',
    'rclpy', 'PoseStamped', 'PoseWithCovarianceStamped', 'Twist', 'OccupancyGrid',
    'SingleThreadedExecutor', 'DurabilityPolicy', 'QoSProfile', 'ReliabilityPolicy',
    'Bool', 'Float32', 'BasicNavigator', 'TaskResult',
    'ensure_rclpy_initialized',
]
