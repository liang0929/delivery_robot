"""把「車已對接好」的當下位姿換算成 dock pose，並寫進 opennav_docking 的
dock database（``src/dock_pose_bridge/config/dock_database.yaml``）。

本模組刻意**不依賴 rclpy / tf2**：TF 查詢由 ``bridge_node.RosBridge`` 負責，
這裡只吃 ``(x_m, y_m, yaw_rad)`` 三個數字，因此換算與寫檔可以完全離線測試。

## 🔴 yaw 換算式（出處：opennav_docking 0.0.2，commit c332c11）

實裝版本是 ``ros-humble-opennav-docking 0.0.2-4``。對應的上游原始碼是
humble 分支的 ``c332c11``（"adding humble ignored file and bumping for
release"，package.xml version 0.0.2）——比對方式：``/opt/ros/humble/include/
opennav_docking/*.hpp`` 全部 8 個 header 的 blob hash 與該 commit 完全一致。

### dock pose 的 yaw 指向哪裡

``SimpleChargingDock::getStagingPose()``（src/simple_charging_dock.cpp:141-167）：

    const double yaw = tf2::getYaw(pose.orientation);          // :153
    staging_pose.pose.position.x += cos(yaw) * staging_x_offset_;  // :158
    staging_pose.pose.position.y += sin(yaw) * staging_x_offset_;  // :159

staging pose ＝ dock pose 沿 **dock 自己的 +x 軸** 平移 ``staging_x_offset``。
本專案的 ``staging_x_offset = -0.7``（config/docking_server.yaml），是負值，
所以 staging 落在 dock 的 **-x 側**。staging 必定在 dock 外側（車還沒進來的
那一側），因此：

    **dock pose 的 +x 軸指向 dock 內部（充電座背面／牆），
      也就是機器人接近 dock 時的行進方向。**

同一結論在 ``DockingServer::approachDock()`` 得到交叉驗證
（src/docking_server.cpp:415-418）：控制目標會沿 dock 的 +x 再往前推
``backward_projection = 0.25``，註解寫明是推到「dock 之後」，避免控制律在
接觸前就收斂。往 dock 之後 ＝ +x 方向，與上面一致。

### 車尾對接（dock_backwards=true）時 base_link 與 dock 的關係

本車充電刷塊在車尾，``dock_backwards: true``。對接完成的那一刻，車尾壓在
dock 接觸面上，車頭背對 dock 內部——也就是 base_link 的 +x 與 dock 的 +x
**反向**：

    dock_yaw = wrap_to_pi(base_link_yaw + π)

這是幾何事實（車尾朝 dock），與控制律的實作缺陷無關；見下面的 ⚠️。

### ⚠️ 0.0.2 的已知上游缺陷（記錄值刻意**不**為它補償）

修復版的 ``approachDock()`` 在 ``dock_backwards_`` 為真時，會先把控制目標的
yaw 轉 π 再交給控制律（上游 commit 5a3883d "Fix dock orientation while
moving backwards" (#51)，2024-07-22，只加了 7 行）。**這個修復不在 0.0.2
裡**（0.0.2 是 2024-06-05），所以本機這版的 approach 控制律會收斂到「dock
在車正後方、且車與 dock 同向」，與上面的幾何相矛盾。

處理原則：dock database 存的是**物理真值**，不是用來抵銷上游 bug 的補償值。
理由有二——

1. database 的 pose 主要被 staging pose 推算與 pre-staging 距離判定使用
   （src/docking_server.cpp:240-244）；為了 approach 而把 yaw 記反，會讓
   staging pose 落到 dock **內側**（牆裡），連帶讓 pre-staging 判定失準。
2. ``use_external_detection_pose: true`` 時，approach 階段真正吃的 dock pose
   來自 AprilTag 偵測（``getRefinedPose``），database 值只是初始估計。

缺陷本身要靠升級 opennav_docking 解決，不在本模組的職責內。

## 位置的取法與其誤差

記錄的位置是 **base_link 原點**（車體中心）在 ``frame`` 中的位置，可再沿
dock 的 +x 方向平移 ``contact_offset``（公尺，預設 0.0）。

預設 0.0 的理由：dock pose 嚴格說應該落在刷塊接觸面上，而 base_link 在車體
中心，兩者差「車體中心 → 車尾刷塊」的距離 L。**L 沒有實測值，本模組不猜**。
維持 0.0 的後果是 staging pose 比設計值近了 L（0.7 → 0.7 − L）；量到 L 之後
用 ``contact_offset`` 帶進來即可，不必改程式。
"""

import math
import os
import tempfile
from datetime import datetime
from typing import Dict, Optional, Tuple

import yaml

from .logging_config import get_logger

logger = get_logger(__name__)

#: dock database 的預設 dock id。與 config/dock_database.yaml 既有條目同名。
DEFAULT_DOCK_ID = 'home_dock'

#: 沒有既有條目可沿用時要寫進去的 type。必須等於 docking_server.yaml 的
#: ``dock_plugins`` 實例名，否則 on_configure 會找不到 plugin。
DEFAULT_DOCK_TYPE = 'charging_dock'

#: 正式記錄唯一可接受的 frame。其餘一律視為測試值（見 ``is_test_frame``）。
PRODUCTION_FRAME = 'map'


class DockDatabaseError(Exception):
    """dock database 讀取或寫入失敗。"""


def wrap_to_pi(angle_rad: float) -> float:
    """把角度正規化到 (-π, π]。"""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def is_test_frame(frame: str) -> bool:
    """非 ``map`` 的 frame 一律是測試值。

    odom 在每次重開機都會歸零，記進 database 的值下次開機就沒有意義；
    這個判定是 ``test_only`` 標記與 UI 警示的唯一依據。
    """
    return frame != PRODUCTION_FRAME


def base_link_to_dock_pose(
    x_m: float,
    y_m: float,
    yaw_rad: float,
    contact_offset_m: float = 0.0,
) -> Tuple[float, float, float]:
    """已對接的 base_link 位姿 →  dock pose ``(x, y, yaw)``。

    換算式與完整出處見模組說明：

        dock_yaw = wrap_to_pi(base_link_yaw + π)      # 車尾朝 dock
        dock_x   = x + cos(dock_yaw) * contact_offset # 沿 dock +x（往 dock 內）
        dock_y   = y + sin(dock_yaw) * contact_offset
    """
    dock_yaw = wrap_to_pi(yaw_rad + math.pi)
    return (
        x_m + math.cos(dock_yaw) * contact_offset_m,
        y_m + math.sin(dock_yaw) * contact_offset_m,
        dock_yaw,
    )


def load_docks(path: str) -> Dict[str, dict]:
    """讀出 database 的 ``docks`` 區塊；檔案不存在時回空 dict。

    解析失敗會拋 :class:`DockDatabaseError` 而不是靜默回空——把壞掉的檔案
    當成「沒有 dock」再覆寫過去，等於無聲吃掉使用者手寫的其他 dock 條目。
    """
    if not os.path.exists(path):
        return {}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as e:
        raise DockDatabaseError(f"無法解析 {path}: {e}") from e

    if data is None:
        return {}
    if not isinstance(data, dict) or not isinstance(data.get('docks'), dict):
        raise DockDatabaseError(f"{path} 缺少 docks 對應表")
    return {k: (v if isinstance(v, dict) else {}) for k, v in data['docks'].items()}


def _format_dock_block(dock_id: str, entry: dict) -> str:
    """單一 dock 條目的 yaml 片段（含記錄註記）。"""
    frame = entry.get('frame', PRODUCTION_FRAME)
    pose = entry.get('pose') or [0.0, 0.0, 0.0]
    lines = [f"  {dock_id}:", f"    type: \"{entry.get('type', DEFAULT_DOCK_TYPE)}\""]

    recorded_at = entry.get('_recorded_at')
    if recorded_at:
        lines.append('')
        if is_test_frame(frame):
            lines += [
                f"    # 🔴 測試值，**不是正式座標**：記錄於 {frame} frame。",
                "    # odom 在每次重開機歸零，這組數字下次開機就對不上實體 dock。",
                "    # 正式記錄必須在 Nav2 起來、map→base_link 存在之後重做一次。",
            ]
        else:
            lines.append("    # 由 POST /v1/robot/dock/record_pose 記錄（車已對接時的實測值）。")
        base = entry.get('_base_link')
        if base:
            lines.append(
                f"    # 記錄時間 {recorded_at}；當下 base_link＝"
                f"({base[0]:.3f}, {base[1]:.3f}, {math.degrees(base[2]):.1f}°)，"
                f"dock yaw ＝ base_link yaw + 180°。"
            )
        else:
            lines.append(f"    # 記錄時間 {recorded_at}。")
        offset = entry.get('_contact_offset')
        if offset:
            lines.append(f"    # 已沿 dock +x 補 contact_offset {offset:.3f} m。")
        else:
            lines.append(
                "    # contact_offset 為 0：位置是 base_link 原點（車體中心），"
                "未扣車體中心→刷塊接觸面的距離。"
            )
    else:
        lines += [
            '',
            "    # 🔴 佔位值，尚未記錄。把車開到與充電座完全對接的位置後，",
            "    # 用網頁的「記錄充電座位置」或 POST /v1/robot/dock/record_pose 寫入。",
        ]

    lines += [
        f"    frame: \"{frame}\"",
        f"    pose: [{float(pose[0]):.4f}, {float(pose[1]):.4f}, {float(pose[2]):.4f}]",
    ]
    return '\n'.join(lines)


def render_database(docks: Dict[str, dict]) -> str:
    """把 dock 對應表重新產生成帶說明註解的 yaml 全文。

    不用 ``yaml.dump``：這個檔案要給現場的人讀，格式說明與「這組數字是什麼、
    什麼時候記的」的註記比機器序列化重要，而 PyYAML 無法保留註解。既有條目
    一律沿用（只有被記錄的那個會換值），手寫的其他 dock 不會消失。
    """
    header = '\n'.join([
        "# dock 資料庫（opennav_docking 的 dock_database 參數指向這裡）",
        "#",
        "# ⚠️ 本檔會被 POST /v1/robot/dock/record_pose 整份重新產生。",
        "#    手寫的 dock 條目會保留（type / frame / pose 照抄），但自行加的",
        "#    註解不會——要留說明請寫進 docs/ 或 launch 註解。",
        "#",
        "# 格式（src:utils.hpp parseDockFile / opennav_docking 0.0.2）：",
        "#   type  必填。必須等於 docking_server.yaml 裡 dock_plugins 的某個實例名。",
        "#   frame 選填，預設 \"map\"。",
        "#   pose  必填，[x, y, theta]，x/y 單位 m、theta 單位 rad。",
        "#         **長度不是 3 會直接報錯。**",
        "#",
        "# 🔴 pose 的 theta 是 dock 的朝向，+x 軸指向 dock 內部（牆），也就是",
        "#    機器人接近 dock 的行進方向——staging pose 由此沿 -x 退",
        "#    staging_x_offset(0.7m) 算出（src:simple_charging_dock.cpp:158-159）。",
        "#    本車車尾對接，所以 dock theta ＝ 對接時 base_link yaw + 180°。",
        "",
        "docks:",
    ])
    blocks = [_format_dock_block(k, v) for k, v in docks.items()]
    return header + '\n' + '\n\n'.join(blocks) + '\n'


def write_atomic(path: str, text: str) -> None:
    """同目錄 temp file + ``os.replace``，避免 docking_server 讀到寫一半的檔。

    temp file 刻意建在目標同一個目錄：``os.replace`` 只在同一檔案系統上才是
    原子操作，用 ``/tmp`` 會退化成跨裝置複製。
    """
    directory = os.path.dirname(path) or '.'
    fd, tmp = tempfile.mkstemp(dir=directory, prefix='.dock_database.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            f.write(text)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, path)
    except Exception:
        try:
            os.unlink(tmp)
        except OSError:
            pass
        raise


def record_dock_pose(
    path: str,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    frame: str = PRODUCTION_FRAME,
    dock_id: str = DEFAULT_DOCK_ID,
    contact_offset_m: float = 0.0,
    now: Optional[datetime] = None,
) -> dict:
    """換算 → 併回既有條目 → 原子寫檔，回傳這次記到的內容。

    ``now`` 只給測試注入，正式呼叫一律用當下時間。
    """
    dock_x, dock_y, dock_yaw = base_link_to_dock_pose(x_m, y_m, yaw_rad, contact_offset_m)
    stamp = (now or datetime.now().astimezone()).isoformat(timespec='seconds')

    docks = load_docks(path)
    entry = dict(docks.get(dock_id) or {})
    entry.update({
        'type': entry.get('type') or DEFAULT_DOCK_TYPE,
        'frame': frame,
        'pose': [dock_x, dock_y, dock_yaw],
        '_recorded_at': stamp,
        '_base_link': [x_m, y_m, yaw_rad],
        '_contact_offset': contact_offset_m,
    })
    docks[dock_id] = entry

    try:
        write_atomic(path, render_database(docks))
    except OSError as e:
        raise DockDatabaseError(f"寫入 {path} 失敗: {e}") from e

    logger.info(
        f"dock '{dock_id}' 位置已記錄到 {path}: frame={frame} "
        f"pose=[{dock_x:.3f}, {dock_y:.3f}, {dock_yaw:.3f}]"
        f"{'（測試值）' if is_test_frame(frame) else ''}"
    )
    return {
        'dock_id': dock_id,
        'frame': frame,
        'pose': {
            'x_m': dock_x,
            'y_m': dock_y,
            'yaw_rad': dock_yaw,
            'yaw_deg': math.degrees(dock_yaw),
        },
        'base_link': {
            'x_m': x_m,
            'y_m': y_m,
            'yaw_rad': yaw_rad,
            'yaw_deg': math.degrees(yaw_rad),
        },
        'contact_offset_m': contact_offset_m,
        'test_only': is_test_frame(frame),
        'recorded_at': stamp,
        'database_path': path,
    }


__all__ = [
    'DEFAULT_DOCK_ID', 'DEFAULT_DOCK_TYPE', 'PRODUCTION_FRAME',
    'DockDatabaseError', 'base_link_to_dock_pose', 'is_test_frame',
    'load_docks', 'record_dock_pose', 'render_database', 'wrap_to_pi',
    'write_atomic',
]
