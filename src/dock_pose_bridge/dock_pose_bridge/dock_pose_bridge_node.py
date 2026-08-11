"""dock_pose_bridge — 把 AprilTag 的 TF 轉成 opennav_docking 吃的 PoseStamped。

  apriltag_ros ──/tf──> (camera_optical_frame → tag36h11:<id>)
                            │  lookup_transform（最新可用）
                            ↓
            detected_dock_pose (geometry_msgs/PoseStamped)
                            │
                            ↓
            opennav_docking / SimpleChargingDock
              （use_external_detection_pose: true 時才訂閱）

存在的理由：偵測端只發 `/tf`（`detections` 訊息內沒有 pose 欄位），
接收端只認硬編碼的 `detected_dock_pose`，Humble 沒有現成的橋接節點可用。
完整比較見調研報告 §4.1／§4.3。

## 🔴 時間戳

發出去的 `header.stamp` 一律是**該 transform 自己的時間戳**，不是 `now()`。
理由見 ``pose_bridge`` 模組說明——`external_detection_timeout` 與 TF 內插
兩邊都直接吃這個值。

## tag 不見了怎麼辦

**不發布**，並限流 warn（比照 pico_sensor_hub 的節點慣例）。判定有兩道：

1. `lookup_transform` 直接失敗（tag 從沒出現過、TF 鏈斷掉）。
2. 查得到、但 transform 已經比 ``max_transform_age_sec`` 舊——
   tf buffer 有 10 秒快取，tag 離開畫面後查詢仍會成功並一直回同一筆舊值，
   只靠第 1 道會安靜地餵十秒鐘的過期位姿。

同一筆 transform 也不會重發：發布頻率與相機幀率不一定整除，重送同一個
stamp 對下游沒有任何新資訊（timeout 判定看的就是 stamp）。

## 相依

只有 rclpy / tf2_ros / geometry_msgs。**不依賴 opennav_docking 的任何訊息**，
所以 opennav_docking 還沒安裝時本 package 一樣 build 得起來、測得過。
"""

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from dock_pose_bridge.pose_bridge import (
    NO_STAMP,
    is_stale,
    same_stamp,
    tag_frame_name,
    transform_age_sec,
    transform_to_pose_stamped,
)


class DockPoseBridge(Node):
    """訂 TF、發 `detected_dock_pose`。"""

    # 參數名與載入後的屬性名一致（比照 pico_sensor_hub 的 PARAMS 表）。
    PARAMS = {
        # 相機的**光學** frame（Z 朝前、X 朝右的那個），也就是 apriltag_ros
        # 收到的影像 header.frame_id——tag 的 TF 就掛在它底下。
        # 預設值與 config/camera_apriltag.yaml 的 camera_frame_id 一致。
        'camera_optical_frame': 'dock_camera_optical_frame',
        # tag family 與 ID。apriltag_ros 預設把 frame 命名成 "<family>:<id>"。
        'tag_family': 'tag36h11',
        'tag_id': 0,
        # 非空時直接當 frame 名用，family/id 不參與命名。對應 apriltag_ros
        # yaml 裡的 tag.frames（有設就不是 "<family>:<id>" 了）。
        'tag_frame': '',
        # 發布 topic。opennav_docking 訂的是相對名稱 "detected_dock_pose"，
        # namespace 為空時解析成 /detected_dock_pose，不需要 remap。
        'output_topic': 'detected_dock_pose',
        # 查詢頻率。相機是 YUY2 1280x720 ~10fps，查更快只會反覆查到同一筆
        # （同 stamp 不重發），查更慢則平白增加偵測延遲。
        'publish_rate_hz': 10.0,
        # 超過這個歲數的 transform 視同「tag 不見了」。0 或負值＝停用，
        # 只信 lookup 成敗（不建議，理由見模組說明）。
        # 0.5 s 在 10fps 下等於連掉 5 幀，不會被單幀漏偵打斷；
        # 同時遠小於 external_detection_timeout(2.0)，下游還來得及自己判逾時。
        'max_transform_age_sec': 0.5,
        'log_throttle_sec': 5.0,
    }

    def __init__(self, tf_buffer=None, **kwargs):
        """``tf_buffer`` 只給測試注入用；``kwargs`` 直接轉給 `Node`。

        給了 buffer 就不建 TransformListener——測試灌自己的 transform，
        不該（也不能）受車上真實 `/tf` 影響。正式執行一律走預設值。
        ``kwargs`` 讓測試能用 ``parameter_overrides`` 改參數，
        不必為此起一個 launch。
        """
        super().__init__('dock_pose_bridge', **kwargs)

        for name, default in self.PARAMS.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)

        if self.publish_rate_hz <= 0:
            raise ValueError(
                f'publish_rate_hz must be positive, got {self.publish_rate_hz}')

        self._tag_frame = tag_frame_name(
            self.tag_family, self.tag_id, self.tag_frame)

        if tf_buffer is None:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        else:
            self.tf_buffer = tf_buffer
            self.tf_listener = None

        self.pose_pub = self.create_publisher(
            PoseStamped, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._tick)

        #: 上一筆已發布的 transform 時戳，用來擋重複幀。
        self._last_stamp = NO_STAMP
        #: 目前是否處於「看得到 tag」狀態，只用來決定要不要打邊緣 log。
        self._detected = False

        self.get_logger().info('dock_pose_bridge 啟動')
        self.get_logger().info(
            f'  來源 TF: {self.camera_optical_frame} → {self._tag_frame}')
        self.get_logger().info(
            f'  發布: {self.output_topic} @ {self.publish_rate_hz:.1f}Hz'
            f'（stamp 沿用 transform 時戳，非 now()）')
        self.get_logger().info(
            f'  transform 超過 {self.max_transform_age_sec:.2f}s 未更新即視為 tag 消失')

    # ------------------------------------------------------------------
    # 週期查詢
    # ------------------------------------------------------------------

    def _tick(self):
        try:
            # Time() ＝ 最新可用的 transform。不指定確切時間點：偵測本來就是
            # 有一幀算一幀，指定時間只會在幀間隔上外插失敗。
            transform = self.tf_buffer.lookup_transform(
                self.camera_optical_frame, self._tag_frame, Time())
        except TransformException as exc:
            self._on_tag_absent(f'查不到 TF：{exc}')
            return

        now = self.get_clock().now().to_msg()
        stamp = transform.header.stamp

        if is_stale(stamp, now, self.max_transform_age_sec):
            age = transform_age_sec(stamp, now)
            self._on_tag_absent(f'最新 transform 已 {age:.2f}s 未更新')
            return

        if same_stamp(stamp, self._last_stamp):
            # 同一幀被查到第二次：偵測端還沒送新的，沒有新資訊可發。
            # 這仍算「看得到 tag」，不觸發消失判定。
            return

        self.pose_pub.publish(transform_to_pose_stamped(transform))
        self._last_stamp = stamp
        self._on_tag_present()

    # ------------------------------------------------------------------
    # 狀態邊緣 log（穩態不洗版）
    # ------------------------------------------------------------------

    def _on_tag_present(self):
        if self._detected:
            return
        self._detected = True
        self.get_logger().info(
            f'偵測到 {self._tag_frame}，{self.output_topic} 開始發布')

    def _on_tag_absent(self, reason):
        """tag 不在時**不發布**；只在狀態翻轉時 warn 一次，之後限流重複提醒。"""
        if self._detected:
            self._detected = False
            self.get_logger().warning(
                f'{self._tag_frame} 消失（{reason}），'
                f'{self.output_topic} 停止發布')
            return
        self.get_logger().warning(
            f'等不到 {self._tag_frame}（{reason}）',
            throttle_duration_sec=self.log_throttle_sec)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DockPoseBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
