"""TF → PoseStamped 的純轉換邏輯（不碰 rclpy、不需執行期）。

節點層（``dock_pose_bridge_node``）只負責「什麼時候查 TF、查不到怎麼辦」，
真正決定訊息內容的規則全在這裡，所以這一層可以離線測到底。

## 為什麼要有這個模組

`apriltag_ros`（christianrauch 版）只把 tag 位姿發到 `/tf`，
`detections` 訊息裡**沒有 pose 欄位**；而 `opennav_docking` 的
`SimpleChargingDock` 只訂閱硬編碼的 `detected_dock_pose`
（`geometry_msgs/PoseStamped`）。中間這段沒有現成的 Humble 套件可用
（`image_proc/TrackMarkerNode` 只存在於 rolling，且是 ArUco 不是 AprilTag），
所以自己接。出處見調研報告 §4.1／§4.3。

## 🔴 stamp 語意

`PoseStamped.header.stamp` **必須**沿用 transform 自己的時間戳，
不可以填 `now()`。下游有兩個地方直接吃這個值：

1. `external_detection_timeout`——用 pose 的 stamp 判斷偵測是否過期。
   填 now() 會讓「相機早就看不到 tag」偽裝成永遠新鮮，dock 會照著一張
   過期的位姿一路撞上去。
2. TF 內插——docking server 要把這個 pose 轉到自己的工作 frame，
   內插點就是這個 stamp。填 now() 會查到「未來」而外插失敗或歪掉。

出處：調研報告 §4.3。
"""

from geometry_msgs.msg import PoseStamped

#: 沒有 tag 時的哨兵值，讓「還沒發過任何一筆」與「發過」可以分辨。
NO_STAMP = None


def tag_frame_name(family, tag_id, override=''):
    """組出 apriltag_ros 會發出的 TF child frame 名。

    預設命名是 ``<family>:<id>``（例如 ``tag36h11:0``），這是
    christianrauch 版 apriltag_ros 沒有設定 ``tag.frames`` 時的行為。
    yaml 裡若替 tag 指定了 ``frames``，frame 名就變成那個自訂字串——
    此時用 ``override`` 直接指定，family/id 就不參與命名。
    """
    if override:
        return override
    return f'{family}:{tag_id}'


def stamp_to_seconds(stamp):
    """`builtin_interfaces/Time` → float 秒。

    只用於「新舊比較」，不當成對外時間值使用：float 秒在 1e9 量級只剩
    約 0.1 µs 解析度，拿去回填 msg 會損失精度。對外一律傳原始 stamp 物件。
    """
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def same_stamp(a, b):
    """兩個 stamp 是否為同一瞬間（整數比較，不經 float）。"""
    if a is NO_STAMP or b is NO_STAMP:
        return False
    return a.sec == b.sec and a.nanosec == b.nanosec


def transform_age_sec(stamp, now):
    """transform 的時間戳距 ``now`` 幾秒（可能為負：時戳來自未來）。"""
    return stamp_to_seconds(now) - stamp_to_seconds(stamp)


def is_stale(stamp, now, max_age_sec):
    """這筆 transform 是否已經舊到該當成「tag 不見了」。

    為什麼需要這一關：`Buffer.lookup_transform(..., Time())` 查的是
    「最新可用」，而 tf buffer 有 10 秒快取——**tag 從畫面消失後，
    這個查詢在快取期內仍然會成功，並且一直回同一筆舊 transform**。
    只靠「lookup 失敗才算消失」的話，實際上要等十秒才會安靜下來。

    ``max_age_sec <= 0`` 代表停用這道判定（只信 lookup 成敗）。

    未來時戳（age < 0）不算 stale：那是時鐘抖動或 transform 剛進 buffer，
    不是「tag 不見了」，交給下游的 timeout 去管。
    """
    if max_age_sec <= 0:
        return False
    return transform_age_sec(stamp, now) > max_age_sec


def transform_to_pose_stamped(transform_stamped):
    """`TransformStamped` → `PoseStamped`，時間戳與 frame 原封不動帶過去。

    ``lookup_transform(camera_optical_frame, tag_frame, ...)`` 回傳的是
    「tag 在相機光學 frame 中的位姿」，所以 header.frame_id 沿用
    transform 的 ``header.frame_id``（＝相機光學 frame），
    child_frame_id（tag）在 PoseStamped 裡沒有對應欄位，自然被丟掉。

    欄位逐一複製而不是直接指派子訊息物件：直接指派會讓兩則訊息共用同一個
    `Vector3` / `Quaternion` 實例，之後任一邊被改到都會互相污染。
    """
    pose = PoseStamped()

    # 🔴 這三行是本模組的重點，理由見模組說明。
    # sec / nanosec 逐欄複製（而非指派整個 Time 物件），原因同下方位姿欄位。
    pose.header.stamp.sec = transform_stamped.header.stamp.sec
    pose.header.stamp.nanosec = transform_stamped.header.stamp.nanosec
    pose.header.frame_id = transform_stamped.header.frame_id

    translation = transform_stamped.transform.translation
    pose.pose.position.x = float(translation.x)
    pose.pose.position.y = float(translation.y)
    pose.pose.position.z = float(translation.z)

    rotation = transform_stamped.transform.rotation
    pose.pose.orientation.x = float(rotation.x)
    pose.pose.orientation.y = float(rotation.y)
    pose.pose.orientation.z = float(rotation.z)
    pose.pose.orientation.w = float(rotation.w)

    return pose
