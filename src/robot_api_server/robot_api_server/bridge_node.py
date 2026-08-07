"""單一常駐 ROS 節點：電壓 / e-stop / 即時地圖訂閱、cmd_vel 與 initialpose 發布。

拆分自 ``ros_bridge.py``，逐字搬移。唯一的行為調整（依重構任務指示做的
低風險依賴注入）：``_on_e_stop`` 原本直接寫入模組全域單例
``state.e_stop_active``；現在改為建構子注入的 ``on_e_stop_changed``
callback，由 composition root（``ros_facade.py``）在組裝單例時綁定
``lambda active: setattr(state, 'e_stop_active', active)``。callback
在收到 ``/e_stop`` 訊息時同步呼叫，時機與行為完全不變，只是「怎麼通知
RobotStateManager」從讀寫模組全域變成呼叫注入的 callable——避免本模組
對 ``process_manager.py`` 產生 import 期的硬相依。
"""

import io
import threading
import time
from typing import Callable, Optional, Tuple
from uuid import uuid4

from .config import (
    BATTERY_MAX_V, BATTERY_MIN_V, BATTERY_STATE_TIMEOUT_SEC, MANUAL_ANGULAR_SPEED,
    MANUAL_LINEAR_SPEED, MANUAL_PUBLISH_HZ, NAV2_READY_TIMEOUT_SEC,
)
from .conversions import (
    m_to_cm, quaternion_to_yaw, voltage_to_battery, yaw_to_deg, yaw_to_quaternion,
)
from .imaging import Image, PIL_AVAILABLE
from .logging_config import get_logger
from .models import BatteryState, Direction, Location
from .ros_common import (
    ROS_AVAILABLE, Bool, DiagnosticStatus, DurabilityPolicy, Float32, OccupancyGrid,
    PoseWithCovarianceStamped, QoSProfile, ReliabilityPolicy, SingleThreadedExecutor,
    Twist, ensure_rclpy_initialized, rclpy,
)

logger = get_logger(__name__)

#: battery_guard 的 ``state`` KeyValue（motor_control.battery_policy 的
#: ``STATE_*`` 常數，大寫）→ API 欄位值。上游若新增狀態，這裡對不到就退回
#: UNKNOWN——寧可顯示「未知」也不要猜成 ok 而讓操作者以為電池沒事。
_BATTERY_STATE_MAP = {
    'OK': BatteryState.OK,
    'WARNING': BatteryState.WARNING,
    'SHUTDOWN': BatteryState.SHUTDOWN,
    'UNKNOWN': BatteryState.UNKNOWN,
}

#: ``state`` KeyValue 缺席時的退路：DiagnosticStatus.level（OK/WARN/ERROR/STALE
#: = 0/1/2/3）→ API 欄位值。用字面數字而非 ``DiagnosticStatus.OK``，因為
#: degraded 模式下 ``DiagnosticStatus`` 是 None。
_LEVEL_TO_BATTERY_STATE = {
    0: BatteryState.OK,
    1: BatteryState.WARNING,
    2: BatteryState.SHUTDOWN,
    3: BatteryState.UNKNOWN,
}


class RosBridge:
    """單一常駐 ROS 節點：電壓 / e-stop / 即時地圖訂閱、cmd_vel 與 initialpose 發布。

    以獨立的 SingleThreadedExecutor 在背景執行緒 spin，
    避免與 BasicNavigator 的 spin 衝突。
    """

    NODE_NAME = 'robot_api_bridge'

    def __init__(
        self,
        on_e_stop_changed: Optional[Callable[[bool], None]] = None,
        battery_state_timeout_sec: float = BATTERY_STATE_TIMEOUT_SEC,
        clock: Callable[[], float] = time.monotonic,
    ):
        # 過期判定用單調時鐘：量的是「距上次收訊多久」，不是牆上時間。
        # 用 time.monotonic() 而非 ROS clock 的理由見 _battery_snapshot。
        # clock 可注入純粹是為了測試能瞬間跳過 timeout，不必真的 sleep。
        self._clock = clock
        self._battery_state_timeout_sec = float(battery_state_timeout_sec)

        self._lock = threading.Lock()
        self._node = None
        self._executor = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

        self._voltage: Optional[float] = None
        self._e_stop = False
        # 低電壓保護狀態（/battery/state）。沒收到訊息＝沒有 battery_guard，
        # 語意是 UNKNOWN 而不是 OK。
        self._battery_state: BatteryState = BatteryState.UNKNOWN
        self._battery_stop_latched = False
        self._battery_voltage: Optional[float] = None
        # 上次收到 /battery/state 的單調時戳；None＝從未收過。
        self._battery_rx_at: Optional[float] = None
        # 是否已進入「曾收訊但已過期」狀態。純粹是 log 節流用的閂：讀取端
        # 每次呼叫都會重新判定過期，若不記住已印過，持續失聯期間每一次
        # /robot/info 都會噴一則 warning。不影響任何對外欄位。
        self._battery_stale_logged = False
        self._latest_map = None
        self._scan_count = 0  # 就緒探測用：確認 /scan 確實在發布
        self._latest_pose: Optional[Tuple[float, float, float]] = None  # (x_m, y_m, yaw_rad)

        self._cmd_vel_pub = None
        self._initialpose_pub = None
        self._tf_buffer = None
        self._tf_listener = None
        self._manual_twist = (0.0, 0.0)  # (linear, angular)

        # 建構子注入：e-stop 狀態變化時的善後 callback（由 composition root
        # 綁定 RobotStateManager.e_stop_active 的 setter）
        self.on_e_stop_changed: Optional[Callable[[bool], None]] = on_e_stop_changed

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
            # /battery/state 由 battery_guard 以 latched 發布；QoS 必須完全對齊
            # （RELIABLE + TRANSIENT_LOCAL depth 1），否則 API server 晚於
            # battery_guard 啟動時收不到「已經鎖存」的現況，UI 會誤顯示 unknown。
            node.create_subscription(
                DiagnosticStatus, '/battery/state', self._on_battery_state, latched_qos
            )

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
    # msg 型別以字串註記（forward reference）：ROS 訊息類別視執行環境而定，
    # 部分（如 LaserScan）只在 _spin_loop 內局部 import，模組層級不一定存在。
    def _on_voltage(self, msg: "Float32") -> None:
        with self._lock:
            self._voltage = float(msg.data)

    def _on_e_stop(self, msg: "Bool") -> None:
        with self._lock:
            self._e_stop = bool(msg.data)
        if self.on_e_stop_changed is not None:
            self.on_e_stop_changed(bool(msg.data))

    def _on_battery_state(self, msg: "DiagnosticStatus") -> None:
        """解析 battery_guard 的診斷訊息。

        以 ``state`` KeyValue 為準（那是 battery_policy 的一手判定結果）；
        沒有這一欄時才退回 ``level``。兩者其實同源，但 level 只有 4 個等級、
        語意較粗，能拿到字串就不猜。
        """
        values = {}
        for kv in (getattr(msg, 'values', None) or []):
            values[kv.key] = kv.value

        raw_state = values.get('state')
        if raw_state is not None:
            state = _BATTERY_STATE_MAP.get(raw_state.strip().upper(), BatteryState.UNKNOWN)
        else:
            state = _LEVEL_TO_BATTERY_STATE.get(int(getattr(msg, 'level', 3)),
                                                BatteryState.UNKNOWN)

        # 明確比對 "true"：任何其他值（含空字串、缺欄位）都當作沒有鎖存，
        # 但 SHUTDOWN 本身就蘊含鎖存，補上以免上游漏填時 UI 少一半資訊。
        latched = values.get('stop_latched', '').strip().lower() == 'true'
        if state == BatteryState.SHUTDOWN:
            latched = True

        voltage = None
        raw_voltage = values.get('voltage', '').strip()
        if raw_voltage:
            try:
                voltage = float(raw_voltage)
            except ValueError:
                logger.debug(f"Unparsable battery voltage in /battery/state: {raw_voltage!r}")

        rx_at = self._clock()
        with self._lock:
            # 恢復在「收到訊息」這一刻就成立，不必等下一次讀取——訊息本身
            # 就是上游回來的證據。判定與清閂都在鎖內，並發收訊只會印一則。
            outage_sec = (
                rx_at - self._battery_rx_at
                if self._battery_stale_logged and self._battery_rx_at is not None
                else None
            )
            self._battery_stale_logged = False
            self._battery_state = state
            self._battery_stop_latched = latched
            self._battery_voltage = voltage
            self._battery_rx_at = rx_at
        # 出鎖才寫 log：logging handler 可能阻塞（檔案 / journal），不該把
        # ROS callback 執行緒的鎖持有時間交給 I/O。
        if outage_sec is not None:
            logger.info(
                f"/battery/state recovered after {outage_sec:.1f}s without messages; "
                f"battery protection state is live again (state={state.value})"
            )

    def _on_map(self, msg: "OccupancyGrid") -> None:
        with self._lock:
            self._latest_map = msg

    def _on_scan(self, msg: "LaserScan") -> None:
        # 只計數，不保留內容——就緒探測只需要知道雷射有沒有在發布
        with self._lock:
            self._scan_count += 1

    def _on_amcl_pose(self, msg: "PoseWithCovarianceStamped") -> None:
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

    def _battery_snapshot(self) -> Tuple[BatteryState, bool, Optional[float]]:
        """取電池保護狀態，逾時未收訊就退回「未知」。

        battery_guard 是**週期**發布（``publish_rate_hz: 2.0``），不是變化時
        才發，所以「久未收訊」＝上游不在了，是可靠訊號。此時必須退回
        UNKNOWN：保留最後一次的 ``ok`` 會讓操作者以為低電壓保護還在線，而
        監視鏈「靜默失效時顯示得比實際樂觀」比誤報嚴重。

        從未收過訊息時（``_battery_rx_at is None``）維持上一輪的語意——同樣
        是 UNKNOWN，只是不經過過期判定，行為與時鐘完全無關。

        時鐘用 ``time.monotonic()`` 而非 ROS clock：
        (1) 量的是經過時間，不能被 NTP 校時或系統時間調整拉歪；
        (2) 這三個 getter 由 FastAPI 執行緒呼叫，degraded 模式下根本沒有
            node，拿 ``node.get_clock()`` 會壞；
        (3) api_server.launch.py 沒有宣告 ``use_sim_time``，bridge 節點永遠
            走系統時間，ROS clock 在此並不會帶來 sim 相容性，只多一層耦合。
        真要在模擬環境跑，收訊時戳與判定時鐘同源即可（兩邊都改），不影響
        本判定的形狀。

        過期是靜默退化（對外只看到 unknown），維運端無從得知 bridge 何時
        失聯，因此在 fresh→stale 轉換印一則 warning；恢復的 info 在
        ``_on_battery_state``。只在轉換點印＝節流，持續失聯不會洗版。
        """
        timeout = self._battery_state_timeout_sec
        stale_since_sec = None
        with self._lock:
            rx_at = self._battery_rx_at
            # timeout <= 0＝停用過期判定（現場緊急關閉用），退回舊行為
            fresh = rx_at is not None and (
                timeout <= 0 or (self._clock() - rx_at) <= timeout)
            if fresh:
                snapshot = (
                    self._battery_state, self._battery_stop_latched, self._battery_voltage)
            else:
                snapshot = (BatteryState.UNKNOWN, False, None)
                # rx_at is None＝從未收過（沒裝 battery_guard），那是常態不是
                # 事件，不印；閂在鎖內翻轉，多執行緒同時讀也只有一個印得到。
                if rx_at is not None and not self._battery_stale_logged:
                    self._battery_stale_logged = True
                    stale_since_sec = self._clock() - rx_at
        if stale_since_sec is not None:
            logger.warning(
                f"/battery/state stale: no message for {stale_since_sec:.1f}s "
                f"(timeout {timeout:.1f}s); battery protection state reported as unknown "
                f"until battery_guard resumes"
            )
        return snapshot

    def battery_state(self) -> BatteryState:
        """低電壓保護狀態；沒有 battery_guard（或它已失聯）時為 UNKNOWN。"""
        return self._battery_snapshot()[0]

    def battery_stop_latched(self) -> bool:
        """battery_guard 是否已鎖存停機（充電後重啟才會解除）。"""
        return self._battery_snapshot()[1]

    def battery_guard_voltage(self) -> Optional[float]:
        """battery_guard 仲裁後的電壓。

        與 ``voltage()``（原始 ``/motor/voltage``）刻意分開：前者已濾波並取
        多來源最小值，是保護判定的依據；後者是 UI 一直在顯示的即時讀值。
        目前只作為診斷用，不進 robot_info。

        與 state / stop_latched 一起過期：三者同源，只讓其中兩個退回未知會
        變成「狀態未知但電壓還在」的自相矛盾讀數。
        """
        return self._battery_snapshot()[2]

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


__all__ = ['RosBridge']
