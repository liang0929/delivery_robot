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

## 現在可跑 vs 待安裝

| 段 | 節點 | 狀態 |
|---|---|---|
| A | `dock_pose_bridge` | ✅ **現在可跑**（本 package，只需 rclpy/tf2_ros） |
| B | `v4l2_camera` / `rectify_node` / `apriltag_node` | ⏳ 待 apt 安裝感知套件 |
| C | `docking_server` + `lifecycle_manager` | ⏳ 待 apt 安裝 opennav_docking（見檔尾註解） |

套件都還沒裝的現在，只有段 A 起得來：

    ros2 launch dock_pose_bridge dock_perception.launch.py bridge_only:=true

（沒有 tag TF 可查，節點會每 5 秒限流 warn 一次「等不到 tag36h11:0」，這是預期行為。）

感知套件裝好之後就用預設值起整條 B+A：

    ros2 launch dock_pose_bridge dock_perception.launch.py

段 C 不在這支 launch 裡執行，只以註解形式放在檔尾——**它需要的套件尚未安裝，
寫成可執行的節點會讓整支 launch 在現在直接失敗**。
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
        description='只起 dock_pose_bridge，不起相機/rectify/apriltag'
                    '（感知套件尚未安裝時的唯一可用組合）')

    camera_tf_arg = DeclareLaunchArgument(
        'publish_camera_tf',
        default_value='false',
        description='發布 base_link → 相機光學 frame 的佔位 static TF。'
                    '預設關閉：實際安裝位置還沒量，發出去只會是錯的 TF。'
                    '量到之後正確做法是寫進 robot_description 的 URDF，'
                    '不是長期留在這裡。')

    bridge_only = LaunchConfiguration('bridge_only')

    # ======================================================================
    # 段 B：感知鏈（⏳ 待安裝 ros-humble-v4l2-camera / image-proc /
    #               apriltag-ros；bridge_only:=true 時整段跳過）
    # ======================================================================
    # ⚠️ executable 名稱以安裝後 `ros2 pkg executables <pkg>` 為準再核對一次。
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
# 段 C：docking_server（⏳ 待安裝，**現在不要取消註解**）
# ==========================================================================
# 需要先 apt 安裝 opennav_docking 的 4 個包（待許可）。裝好之後把下面這段
# 搬進 generate_launch_description() 的回傳清單，並自行決定要不要跟感知鏈
# 放同一支 launch——實務上建議分開起，感知鏈可以先單獨驗軸向與偵測距離。
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
