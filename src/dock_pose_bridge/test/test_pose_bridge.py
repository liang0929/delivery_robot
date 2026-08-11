"""TF → PoseStamped 轉換規則的離線測試（不起節點、不 rclpy.init）。

盯的是三件會讓 docking 安靜地走歪的事：

1. **時間戳保真**。stamp 被換成 now() 的話，
   `external_detection_timeout` 永遠不會觸發、TF 內插點也會錯位——
   而且畫面上一切看起來都正常。
2. **frame 帶對**。PoseStamped 的 frame_id 必須是相機光學 frame
   （transform 的 parent），填成 tag frame 會讓下游把座標轉錯邊。
3. **「tag 消失」的判定**。tf buffer 有 10 秒快取，tag 離開畫面後
   lookup 仍然成功並回同一筆舊值，所以「舊到什麼程度算消失」這條線
   本身就是功能的一部分。
"""

from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped

import pytest

from dock_pose_bridge.pose_bridge import (
    NO_STAMP,
    is_stale,
    same_stamp,
    stamp_to_seconds,
    tag_frame_name,
    transform_age_sec,
    transform_to_pose_stamped,
)

CAMERA_FRAME = 'dock_camera_optical_frame'
TAG_FRAME = 'tag36h11:0'


def stamp(sec, nanosec=0):
    msg = TimeMsg()
    msg.sec = sec
    msg.nanosec = nanosec
    return msg


def make_transform(sec=1234, nanosec=567890123,
                   frame_id=CAMERA_FRAME, child_frame_id=TAG_FRAME):
    """一筆典型的 apriltag_ros TF：相機光學 frame 底下掛著 tag。"""
    t = TransformStamped()
    t.header.stamp = stamp(sec, nanosec)
    t.header.frame_id = frame_id
    t.child_frame_id = child_frame_id
    t.transform.translation.x = 0.1
    t.transform.translation.y = -0.2
    t.transform.translation.z = 1.5
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.7071067811865476
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 0.7071067811865476
    return t


# --------------------------------------------------------------------------
# 🔴 時間戳保真（最高風險路徑）
# --------------------------------------------------------------------------

def test_stamp_comes_from_transform_not_now():
    """PoseStamped 的 stamp 必須逐位元等於 transform 的 stamp。"""
    t = make_transform(sec=1234, nanosec=567890123)
    pose = transform_to_pose_stamped(t)
    assert pose.header.stamp.sec == 1234
    assert pose.header.stamp.nanosec == 567890123


def test_stamp_nanosec_not_truncated():
    """奈秒不得被 float 秒的往返運算磨掉（1e9 量級只剩 ~0.1µs 解析度）。"""
    t = make_transform(sec=1786441200, nanosec=999999999)
    pose = transform_to_pose_stamped(t)
    assert pose.header.stamp.sec == 1786441200
    assert pose.header.stamp.nanosec == 999999999


def test_zero_stamp_is_passed_through():
    """stamp 為 0 也照樣帶過去，不可被當成「沒有時間」而改填 now()。"""
    pose = transform_to_pose_stamped(make_transform(sec=0, nanosec=0))
    assert pose.header.stamp.sec == 0
    assert pose.header.stamp.nanosec == 0


def test_stamp_is_not_aliased_to_source():
    """複製而非共用：改到來源不該回頭污染已發出的訊息。"""
    t = make_transform(sec=100, nanosec=0)
    pose = transform_to_pose_stamped(t)
    t.header.stamp.sec = 999
    assert pose.header.stamp.sec == 100


# --------------------------------------------------------------------------
# frame 與位姿欄位
# --------------------------------------------------------------------------

def test_frame_id_is_the_transform_parent():
    """frame_id 取 parent（相機光學 frame），不是 child（tag）。"""
    pose = transform_to_pose_stamped(make_transform())
    assert pose.header.frame_id == CAMERA_FRAME
    assert pose.header.frame_id != TAG_FRAME


def test_translation_and_rotation_are_copied():
    t = make_transform()
    pose = transform_to_pose_stamped(t)
    assert pose.pose.position.x == pytest.approx(0.1)
    assert pose.pose.position.y == pytest.approx(-0.2)
    assert pose.pose.position.z == pytest.approx(1.5)
    assert pose.pose.orientation.x == pytest.approx(0.0)
    assert pose.pose.orientation.y == pytest.approx(0.7071067811865476)
    assert pose.pose.orientation.z == pytest.approx(0.0)
    assert pose.pose.orientation.w == pytest.approx(0.7071067811865476)


def test_pose_fields_are_not_aliased_to_source():
    t = make_transform()
    pose = transform_to_pose_stamped(t)
    t.transform.translation.x = 42.0
    t.transform.rotation.w = 0.0
    assert pose.pose.position.x == pytest.approx(0.1)
    assert pose.pose.orientation.w == pytest.approx(0.7071067811865476)


# --------------------------------------------------------------------------
# frame 命名
# --------------------------------------------------------------------------

def test_default_tag_frame_name():
    """apriltag_ros 沒設 tag.frames 時的命名。"""
    assert tag_frame_name('tag36h11', 0) == 'tag36h11:0'


def test_tag_frame_override_wins():
    """yaml 設了 tag.frames 就以它為準，family/id 不參與命名。"""
    assert tag_frame_name('tag36h11', 0, 'dock_tag') == 'dock_tag'


def test_empty_override_falls_back_to_default():
    assert tag_frame_name('tag36h11', 7, '') == 'tag36h11:7'


# --------------------------------------------------------------------------
# 「tag 消失」判定
# --------------------------------------------------------------------------

def test_fresh_transform_is_not_stale():
    assert not is_stale(stamp(100, 0), stamp(100, 200000000), 0.5)


def test_old_transform_is_stale():
    """tag 離開畫面後 lookup 仍會回舊值，靠這條線才判得出消失。"""
    assert is_stale(stamp(100, 0), stamp(101, 0), 0.5)


def test_exactly_at_threshold_is_not_stale():
    """邊界不算過期：> 才算，避免剛好卡在門檻上時反覆抖動。"""
    assert not is_stale(stamp(100, 0), stamp(100, 500000000), 0.5)


def test_future_stamp_is_not_stale():
    """未來時戳是時鐘抖動，不是「tag 不見了」。"""
    assert not is_stale(stamp(101, 0), stamp(100, 0), 0.5)


def test_zero_max_age_disables_staleness():
    assert not is_stale(stamp(0, 0), stamp(9999, 0), 0.0)


def test_negative_max_age_disables_staleness():
    assert not is_stale(stamp(0, 0), stamp(9999, 0), -1.0)


def test_transform_age_crosses_second_boundary():
    age = transform_age_sec(stamp(99, 900000000), stamp(100, 100000000))
    assert age == pytest.approx(0.2)


# --------------------------------------------------------------------------
# 重複幀判定
# --------------------------------------------------------------------------

def test_same_stamp_is_detected():
    assert same_stamp(stamp(5, 5), stamp(5, 5))


def test_nanosec_difference_is_a_new_frame():
    assert not same_stamp(stamp(5, 5), stamp(5, 6))


def test_no_previous_stamp_is_never_the_same():
    """啟動後的第一筆一定要發得出去。"""
    assert not same_stamp(stamp(5, 5), NO_STAMP)
    assert not same_stamp(NO_STAMP, stamp(5, 5))


def test_stamp_to_seconds():
    assert stamp_to_seconds(stamp(2, 500000000)) == pytest.approx(2.5)
