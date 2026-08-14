"""充電對接的感知鏈：相機 → rectify → AprilTag → dock_pose_bridge。

    usb_cam ──/image_raw──> rectify ──/image_rect──> apriltag_node
       └───────/camera_info─────┴──────────────────────┘
                                                    │ /tf (dock_camera_optical_frame
                                                    │      → tag36h11:0)
                                                    ↓
                                          dock_pose_bridge
                                                    │
                                                    ↓ /detected_dock_pose
                                       （opennav_docking，待安裝）

## 各段狀態

| 段 | 節點 | 狀態 |
|---|---|---|
| A | `dock_pose_bridge` | ✅ 可跑（本 package，只需 rclpy/tf2_ros） |
| B | `usb_cam` / `rectify_node` / `apriltag_node` | ✅ 真相機＋真 tag 端到端驗過 |
| C | `docking_server` + `lifecycle_manager` | ✅ 套件已裝；仍以註解放在檔尾，理由見該處 |

套件（ros-humble-usb-cam / image-proc / apriltag-ros / opennav-docking）
都已安裝，段 B 的 executable 名稱已對照 `ros2 pkg executables` 逐一驗過。

整條 B+A：

    ros2 launch dock_pose_bridge dock_perception.launch.py

⚠️ `usb_cam` 開不了裝置時會**退出**，不像舊的 v4l2_camera 印完錯誤還繼續跑
（實測，兩種失敗長得不一樣）：
  - 裝置不存在 → `Device specified is not available or is not a vaild V4L2
    device: /dev/dock_camera`，接著印出目前可用的 /dev/videoN 清單，
    然後正常關閉（exit 0）。
  - 裝置被別的 process 佔用 → `terminate called after throwing an instance
    of 'char*'`，SIGABRT（exit 250）。跑之前先 `fuser -v /dev/dock_camera`
    看有沒有前一輪沒收乾淨的節點。
其餘三個節點照常運作，沒有影像就沒有 tag TF，`dock_pose_bridge` 會每 5 秒
限流 warn 一次「等不到 tag36h11:0」。USB 拔插後 udev 會重建 symlink，
但 usb_cam 節點已經退出、不會自己回來，要重起——日後 systemd 化時
這段要設 `Restart=always`。

⚠️ 若 `apriltag` 一直印 `Topics '/image_rect' and '/camera_info' do not
appear to be synchronized` 且 `Synchronized pairs: 0`，整條鏈就是完全不偵測
（只有 warn，沒有 error，很容易漏看）。實測 8 次啟動中出現 1 次，觸發條件
還沒定位（當次的前一輪有個 usb_cam 是 SIGABRT 死的，懷疑與殘留的 DDS 端點
有關，但刻意「殺完馬上重起」重現 2 次都沒中）。它不會自己恢復，把四個節點
全殺乾淨、等幾秒再起即可。
每次起鏈後請自檢：`ros2 topic hz /detections` 有沒有在跑
（正常應該貼近相機幀率，約 13~15 Hz）。

只起段 A：

    ros2 launch dock_pose_bridge dock_perception.launch.py bridge_only:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('dock_pose_bridge')
    bridge_config = os.path.join(share, 'config', 'dock_pose_bridge.yaml')
    camera_config = os.path.join(share, 'config', 'camera_apriltag.yaml')

    bridge_only_arg = DeclareLaunchArgument(
        'bridge_only',
        default_value='false',
        description='只起 dock_pose_bridge，不起相機/rectify/apriltag')

    camera_tf_arg = DeclareLaunchArgument(
        'publish_camera_tf',
        default_value='false',
        description='發布 base_link → 相機光學 frame 的佔位 static TF。'
                    '預設關閉：實際安裝位置還沒量，發出去只會是錯的 TF。'
                    '量到之後正確做法是寫進 robot_description 的 URDF，'
                    '不是長期留在這裡。')

    bridge_only = LaunchConfiguration('bridge_only')

    # ======================================================================
    # 段 B：感知鏈（bridge_only:=true 時整段跳過）
    # ======================================================================
    # executable 名稱已對照實裝核對過（`ros2 pkg executables <pkg>`）：
    #   usb_cam      → usb_cam_node_exe   ✅（另有 show_image.py）
    #   image_proc   → rectify_node       ✅
    #   apriltag_ros → apriltag_node      ✅（該 package 只有這一個）
    # 三個節點也都有對應的 composable 版本，改走 container 時用這些 plugin：
    #   usb_cam::UsbCamNode / image_proc::RectifyNode / AprilTagNode
    #   （apriltag_ros 的 plugin 沒有 namespace 前綴，別寫成 apriltag_ros::）
    #
    # 🔴 name 必須是 'usb_cam'：camera_apriltag.yaml 的參數掛在 `usb_cam:`
    # 這個 key 底下，節點名一改就整份參數讀不到（會安靜地跑預設值
    # /dev/video0 + 640x480 + 30fps，很難察覺）。
    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        output='screen',
        parameters=[camera_config],
        condition=UnlessCondition(bridge_only),
    )

    # AprilTag 的位姿估計要求輸入已 rectify（它只讀 camera_info 的 P 矩陣，
    # 不讀 D）。humble 的 rectify 不限制 encoding 且沿用輸入 encoding，
    # 所以 rgb8 進、rgb8 出；rgb8→mono8 由 apriltag_node 端的 cv_bridge
    # toCvShare(msg, "mono8") 處理（usb_cam 沒有可用的灰階輸出路徑，
    # 理由見 config/camera_apriltag.yaml 取像段）。
    rectify = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify_dock_camera',
        output='screen',
        remappings=[
            ('image', '/image_raw'),
            ('camera_info', '/camera_info'),
            ('image_rect', '/image_rect'),
        ],
        condition=UnlessCondition(bridge_only),
    )

    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[camera_config],
        remappings=[
            ('image_rect', '/image_rect'),
            ('camera_info', '/camera_info'),
        ],
        condition=UnlessCondition(bridge_only),
    )

    # 佔位 TF：相機裝在車尾朝後，實際位置待量測（預設不啟用，見上面的說明）。
    # 值的意義：x 往後 0.3m、z 高 0.2m，再從 base_link 的 REP-103 軸向轉到
    # 光學 frame 的軸向（Z 朝前、X 朝右）——這裡的 yaw/pitch/roll 是
    # 「朝車尾看」的光學 frame，量到真實安裝位置前不要當真。
    camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='dock_camera_tf',
        output='screen',
        arguments=[
            '--x', '-0.30', '--y', '0.0', '--z', '0.20',
            '--yaw', '1.5708', '--pitch', '0.0', '--roll', '-1.5708',
            '--frame-id', 'base_link',
            '--child-frame-id', 'dock_camera_optical_frame',
        ],
        condition=IfCondition(LaunchConfiguration('publish_camera_tf')),
    )

    # ======================================================================
    # 段 A：橋接（✅ 現在就可跑）
    # ======================================================================
    bridge = Node(
        package='dock_pose_bridge',
        executable='dock_pose_bridge_node',
        name='dock_pose_bridge',
        output='screen',
        parameters=[bridge_config],
    )

    return LaunchDescription([
        bridge_only_arg,
        camera_tf_arg,
        camera,
        rectify,
        apriltag,
        camera_tf,
        bridge,
    ])


# ==========================================================================
# 段 C：docking_server（套件已裝，仍**刻意保持註解**）
# ==========================================================================
# opennav_docking 4 包已安裝，下面的 executable 名稱也已核對：
#   opennav_docking        → opennav_docking   ✅
#   nav2_lifecycle_manager → lifecycle_manager ✅
# config/docking_server.yaml 已實跑過 on_configure，plugin 載入、
# dock_database 解析都通過。
#
# 那為什麼還是註解？因為 **active 的 docking_server 會發 /cmd_vel**
# （configure 就建好 publisher，active 後收到 goal 即輸出速度），
# 而實裝的 0.0.2-4 缺 rotate_to_dock、也沒有任何碰撞偵測——
# 詳見 config/docking_server.yaml 開頭的版本落差說明。
# 在那個缺口收斂、且相機與 INA226 都上線之前，這段不該自動啟動。
# 取消註解前請先確認：有人在旁邊看著、急停在手上。
#
# 另外實務上建議段 C 與感知鏈分開起：感知鏈可以先單獨驗軸向與偵測距離。
#
# docking_server 是 **lifecycle node**：只 Node(...) 起來它會停在
# unconfigured，什麼都不做，必須有 lifecycle_manager 把它推到 active。
#
#     docking_server = Node(
#         package='opennav_docking',
#         executable='opennav_docking',
#         name='docking_server',
#         output='screen',
#         parameters=[os.path.join(share, 'config', 'docking_server.yaml')],
#     )
#
#     # dock_database 參數在 yaml 裡是寫死的絕對路徑，用這個覆寫比較穩：
#     #   parameters=[cfg, {'dock_database': os.path.join(
#     #       share, 'config', 'dock_database.yaml')}]
#
#     lifecycle_manager = Node(
#         package='nav2_lifecycle_manager',
#         executable='lifecycle_manager',
#         name='lifecycle_manager_docking',
#         output='screen',
#         parameters=[{
#             'autostart': True,
#             'node_names': ['docking_server'],
#         }],
#     )
#
# 起來之後的動作（Nav2 尚未啟動，所以 staging 要手動開過去）：
#
#     ros2 action send_goal /dock_robot opennav_docking_msgs/action/DockRobot \
#       "{use_dock_id: true, dock_id: 'home_dock',
#         navigate_to_staging_pose: false}"
#
# navigate_to_staging_pose 必須是 false：預設 true 會去呼叫 Nav2 的
# NavigateToPose，而車上沒有 Nav2 在跑，會停在 FAILED_TO_STAGE(903)。
