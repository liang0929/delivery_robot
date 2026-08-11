"""充電對接的感知鏈：相機 → rectify → AprilTag → dock_pose_bridge。

    v4l2_camera ──/image_raw──> rectify ──/image_rect──> apriltag_node
         └────────/camera_info──────┴──────────────────────┘
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
| B | `v4l2_camera` / `rectify_node` / `apriltag_node` | ✅ 套件已裝、名稱已核對；`v4l2_camera` 待相機到貨 |
| C | `docking_server` + `lifecycle_manager` | ✅ 套件已裝；仍以註解放在檔尾，理由見該處 |

套件（ros-humble-v4l2-camera / image-proc / apriltag-ros / opennav-docking）
都已安裝，段 B 的 executable 名稱已對照 `ros2 pkg executables` 逐一驗過。

整條 B+A：

    ros2 launch dock_pose_bridge dock_perception.launch.py

⚠️ 相機尚未接上（沒有 /dev/video*）時，`v4l2_camera` 會印
`Failed opening device /dev/video0` 但**不會退出**，其餘節點照常運作；
沒有影像就沒有 tag TF，`dock_pose_bridge` 會每 5 秒限流 warn 一次
「等不到 tag36h11:0」——這兩個都是預期行為。

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
    #   v4l2_camera  → v4l2_camera_node   ✅（另有 v4l2_camera_compose_test）
    #   image_proc   → rectify_node       ✅
    #   apriltag_ros → apriltag_node      ✅（該 package 只有這一個）
    # 三個節點也都有對應的 composable 版本，改走 container 時用這些 plugin：
    #   v4l2_camera::V4L2Camera / image_proc::RectifyNode / AprilTagNode
    #   （apriltag_ros 的 plugin 沒有 namespace 前綴，別寫成 apriltag_ros::）
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[camera_config],
        condition=UnlessCondition(bridge_only),
    )

    # AprilTag 的位姿估計要求輸入已 rectify（它只讀 camera_info 的 P 矩陣，
    # 不讀 D）。humble 的 rectify 不限制 encoding 且沿用輸入 encoding，
    # 所以 mono8 進、mono8 出，不會偷偷轉成 RGB。
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
