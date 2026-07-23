"""真機 (HSMotorController) 與 mock (MockMotorController) 馬達節點共用基底。

只收兩邊本就完全等價的邏輯：
    - cmd_vel 的 NaN/Inf 驗證（含警告訊息文字）
    - 標準 QoS profile 建構（cmd_vel/odom 等使用的 RELIABLE/VOLATILE profile）
    - E-Stop 訂閱建立（含其 QoS：TRANSIENT_LOCAL，確保收到 latched 狀態）
    - cmd_vel watchdog 的純時間判斷（是否逾時，不含逾時後的動作）

刻意 **不** 共用的部分（兩邊行為本就不同，抽出來會造成行為偏移）：
    - E-Stop 狀態機（真機需以 state_lock 保護、mock 不需要）
    - cmd_vel 逾時後的歸零動作（真機歸零 target_rpm 並持鎖、
      mock 直接歸零 current_linear_x/current_angular_z）
    - RPM 死區、量化等真機/mock 各自特有的驅動器模擬邏輯
"""

import math

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


class BaseMotorNode(Node):
    """馬達控制節點共用基底類別。"""

    @staticmethod
    def _make_reliable_qos(depth: int = 10) -> QoSProfile:
        """建立 RELIABLE / VOLATILE 的標準 QoS profile。

        與抽取前 HSMotorController / MockMotorController 各自內嵌的
        ``QoSProfile(depth=10, reliability=RELIABLE, durability=VOLATILE)``
        完全相同。
        """
        return QoSProfile(
            depth=depth,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _setup_e_stop_subscription(self) -> None:
        """建立 ``/e_stop`` 訂閱並初始化 ``e_stop_active = False``。

        QoS 為 TRANSIENT_LOCAL，確保收到 latched 狀態，與抽取前兩邊
        內嵌的程式碼完全相同。callback 由子類別各自的
        ``self.e_stop_callback`` 提供（兩邊狀態機不同，不共用）。
        """
        e_stop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.e_stop_sub = self.create_subscription(
            Bool, '/e_stop', self.e_stop_callback, e_stop_qos)
        self.e_stop_active = False

    def _validate_cmd_vel(self, msg: Twist) -> bool:
        """驗證 cmd_vel 是否包含 NaN/Inf。

        無效時記警告 log（訊息文字與抽取前逐字相同）並回傳 False；
        有效回傳 True。呼叫端據此決定是否 return（行為與抽取前相同）。
        """
        if math.isnan(msg.linear.x) or math.isinf(msg.linear.x):
            self.get_logger().warning('Invalid linear.x value (NaN/Inf), ignoring command')
            return False
        if math.isnan(msg.angular.z) or math.isinf(msg.angular.z):
            self.get_logger().warning('Invalid angular.z value (NaN/Inf), ignoring command')
            return False
        return True

    def _is_cmd_vel_stale(self, timeout_sec: float = 1.0) -> bool:
        """cmd_vel watchdog 的純時間判斷：距上次有效命令是否已逾時。

        只回傳布林值，逾時後的歸零動作（是否持鎖、歸零哪些狀態）留給
        子類別各自的 safety_check 實作。
        """
        time_since_cmd = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        return time_since_cmd > timeout_sec
