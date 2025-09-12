import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
# 用來直接給定目標點座標
def send_goal(x, y, yaw_deg):
    rclpy.init()
    navigator = BasicNavigator()

    # 等待 Nav2 啟動
    navigator.waitUntilNav2Active()

    # 建立目標 Pose
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y

    # 將角度轉成 quaternion (只旋轉 z 軸)
    yaw_rad = math.radians(yaw_deg)
    goal_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    goal_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

    # 發送目標
    navigator.goToPose(goal_pose)

    # 等待完成
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback:
            print(f"距離目標還有 {feedback.distance_remaining:.2f} m")

    # 結果
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print("到達目標")
    elif result == TaskResult.CANCELED:
        print("任務被取消")
    elif result == TaskResult.FAILED:
        print("任務失敗")

    rclpy.shutdown()


if __name__ == "__main__":
    # 單位公尺（x,y,theta）
    send_goal(2.0, 1.0, 90)
