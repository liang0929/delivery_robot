"""節點層行為測試：灌 TF 進去，看 `detected_dock_pose` 出來什麼。

## 為什麼 fake broadcaster 是「直接寫 buffer」而不是真的發 /tf

`TransformListener` 收到 `/tf` 之後做的事就是 `Buffer.set_transform()`，
所以直接寫 buffer 與真的廣播在被測程式眼中完全等價，但少了兩個麻煩：

1. **不依賴 DDS 通訊**——colcon test 不會因為網路/domain 設定而變成 flaky。
2. **不會被車上真實的 `/tf` 污染**——這台機器平常就跑著 robot-core 在發 TF，
   走真 topic 的測試會收到不屬於自己的資料。

節點本身支援注入 buffer（`DockPoseBridge(tf_buffer=...)`），注入時就不建
TransformListener，測試環境因此完全封閉。

發布端同理：用一個記錄用的假 publisher 換掉真的，驗的是「發了什麼」，
不是 DDS 有沒有把它送到。
"""

import time

import pytest
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from tf2_ros.buffer import Buffer

from dock_pose_bridge.dock_pose_bridge_node import DockPoseBridge

CAMERA_FRAME = 'dock_camera_optical_frame'
TAG_FRAME = 'tag36h11:0'


class RecordingPublisher:
    """假 publisher：把發出去的訊息留下來給測試檢查。"""

    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


@pytest.fixture(scope='module', autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def make_bridge(**params):
    """建一顆封閉的 bridge：乾淨 buffer（不訂閱真 /tf）＋假 publisher。"""
    overrides = [
        rclpy.parameter.Parameter(name, value=value)
        for name, value in params.items()
    ]
    node = DockPoseBridge(tf_buffer=Buffer(), parameter_overrides=overrides)
    node.pose_pub = RecordingPublisher()
    return node


@pytest.fixture
def bridge_factory():
    """要改參數的測試用這個，建出來的節點統一在收尾時銷毀。"""
    created = []

    def factory(**params):
        node = make_bridge(**params)
        created.append(node)
        return node

    yield factory
    for node in created:
        node.destroy_node()


@pytest.fixture
def bridge(bridge_factory):
    return bridge_factory()


def feed_tag(node, age_sec=0.0, x=0.1, y=-0.2, z=1.5):
    """模擬 apriltag_ros 廣播一筆 tag TF，時戳為「現在往前推 age_sec」。

    回傳灌進去的 stamp，讓測試可以逐位元比對發出來的那筆。
    """
    detected_at = node.get_clock().now() - Duration(seconds=age_sec)
    stamp = detected_at.to_msg()

    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = CAMERA_FRAME
    t.child_frame_id = TAG_FRAME
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    t.transform.rotation.w = 1.0
    node.tf_buffer.set_transform(t, 'test_apriltag')
    return stamp


def stamp_of(msg):
    return (msg.header.stamp.sec, msg.header.stamp.nanosec)


def as_tuple(stamp: TimeMsg):
    return (stamp.sec, stamp.nanosec)


# --------------------------------------------------------------------------
# 🔴 stamp 語意：發出去的時戳＝transform 的時戳，不是 now()
# --------------------------------------------------------------------------

def test_published_stamp_equals_transform_stamp(bridge):
    stamp = feed_tag(bridge, age_sec=0.2)
    bridge._tick()

    assert len(bridge.pose_pub.messages) == 1
    assert stamp_of(bridge.pose_pub.messages[0]) == as_tuple(stamp)


def test_published_stamp_is_older_than_now(bridge):
    """反面確認：now() 會等於當下，transform 時戳必須落在它之前。"""
    feed_tag(bridge, age_sec=0.2)
    bridge._tick()

    published = bridge.pose_pub.messages[0].header.stamp
    now = bridge.get_clock().now().to_msg()
    published_sec = published.sec + published.nanosec * 1e-9
    now_sec = now.sec + now.nanosec * 1e-9
    assert published_sec < now_sec
    # 而且差距就是我們灌進去的那 0.2 秒（放寬到 0.15~0.45 容忍測試機負載）
    assert 0.15 < now_sec - published_sec < 0.45


def test_published_frame_is_camera_optical_frame(bridge):
    feed_tag(bridge)
    bridge._tick()

    assert bridge.pose_pub.messages[0].header.frame_id == CAMERA_FRAME


def test_published_pose_matches_transform(bridge):
    feed_tag(bridge, x=0.3, y=0.4, z=2.0)
    bridge._tick()

    pose = bridge.pose_pub.messages[0].pose
    assert pose.position.x == pytest.approx(0.3)
    assert pose.position.y == pytest.approx(0.4)
    assert pose.position.z == pytest.approx(2.0)
    assert pose.orientation.w == pytest.approx(1.0)


# --------------------------------------------------------------------------
# tag 缺席時不發布
# --------------------------------------------------------------------------

def test_no_publish_when_tag_never_seen(bridge):
    """TF 樹裡根本沒有這個 tag：lookup 失敗，不得發布。"""
    bridge._tick()
    assert bridge.pose_pub.messages == []


def test_no_publish_when_transform_is_stale(bridge):
    """tag 離開畫面後 lookup 仍會成功（buffer 快取 10 秒），必須靠歲數判掉。"""
    feed_tag(bridge, age_sec=3.0)   # 遠大於 max_transform_age_sec 預設 0.5
    bridge._tick()
    assert bridge.pose_pub.messages == []


def test_stops_publishing_after_tag_disappears(bridge_factory):
    """看得到 → 消失：先發一筆，之後靜默（buffer 裡的舊值不再被送出）。

    真的等它過期，不是灌一筆假的舊 transform——`lookup_transform(..., Time())`
    查的是「時戳最新的那筆」，灌舊的根本不會被選中，測不到這條路徑。
    """
    node = bridge_factory(max_transform_age_sec=0.2)
    feed_tag(node)
    node._tick()
    assert len(node.pose_pub.messages) == 1

    # 相機不再送新幀，buffer 裡那筆就這樣放到過期
    time.sleep(0.4)
    node._last_stamp = None         # 排除「重複幀」這條路徑的干擾
    node._tick()
    assert len(node.pose_pub.messages) == 1
    assert node._detected is False   # 狀態要翻回「看不到」


def test_recovers_after_tag_reappears(bridge):
    bridge._tick()                  # 一開始查不到
    assert bridge.pose_pub.messages == []

    feed_tag(bridge)
    bridge._tick()
    assert len(bridge.pose_pub.messages) == 1


# --------------------------------------------------------------------------
# 同一幀不重發
# --------------------------------------------------------------------------

def test_same_transform_is_published_once(bridge):
    """查詢頻率高於相機幀率時，同一筆 transform 會被查到很多次。"""
    feed_tag(bridge)
    bridge._tick()
    bridge._tick()
    bridge._tick()
    assert len(bridge.pose_pub.messages) == 1


def test_new_frame_is_published_again(bridge):
    first = feed_tag(bridge)
    bridge._tick()
    second = feed_tag(bridge)       # 新的一幀，時戳不同
    bridge._tick()

    assert len(bridge.pose_pub.messages) == 2
    assert stamp_of(bridge.pose_pub.messages[0]) == as_tuple(first)
    assert stamp_of(bridge.pose_pub.messages[1]) == as_tuple(second)


# --------------------------------------------------------------------------
# 設定
# --------------------------------------------------------------------------

def test_defaults_match_apriltag_naming(bridge):
    """預設查的就是 apriltag_ros 預設會發的那個 frame。"""
    assert bridge._tag_frame == TAG_FRAME
    assert bridge.camera_optical_frame == CAMERA_FRAME
    assert bridge.output_topic == 'detected_dock_pose'


def test_custom_tag_frame_is_used(bridge_factory):
    """yaml 給了 tag.frames 時，bridge 要改查那個名字。"""
    node = bridge_factory(tag_frame='dock_tag')
    assert node._tag_frame == 'dock_tag'


def test_tag_id_changes_the_frame_name(bridge_factory):
    node = bridge_factory(tag_id=7)
    assert node._tag_frame == 'tag36h11:7'


def test_zero_max_age_publishes_even_old_transforms(bridge_factory):
    """停用歲數判定後，只剩 lookup 成敗決定發不發。"""
    node = bridge_factory(max_transform_age_sec=0.0)
    feed_tag(node, age_sec=5.0)
    node._tick()
    assert len(node.pose_pub.messages) == 1


def test_rejects_non_positive_rate():
    """除以零會在 create_timer 之前就炸開，這裡要求它明確擋下。"""
    with pytest.raises(ValueError):
        make_bridge(publish_rate_hz=0.0).destroy_node()
