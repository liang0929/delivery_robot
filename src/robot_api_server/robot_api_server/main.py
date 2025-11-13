import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped
import math
import threading
from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
import uvicorn

# --- Pydantic Model for API Input ---
class Goal(BaseModel):
    x: float
    y: float
    yaw_deg: float

# --- FastAPI App ---
app = FastAPI()

# Global variable to hold the ROS2 node
ros_node = None

@app.post("/navigate_to_goal")
async def navigate_to_goal(goal: Goal):
    """
    Receives a navigation goal and sends it to the Nav2 stack.
    """
    if ros_node is None:
        raise HTTPException(status_code=503, detail="ROS2 node is not ready.")
    
    try:
        ros_node.get_logger().info(f"Received goal: x={goal.x}, y={goal.y}, yaw={goal.yaw_deg}")
        # Run the navigation task in a separate thread to avoid blocking the API
        thread = threading.Thread(target=ros_node.send_goal_to_nav2, args=(goal.x, goal.y, goal.yaw_deg))
        thread.start()
        return {"message": "Goal received, navigation started."}
    except Exception as e:
        ros_node.get_logger().error(f"Failed to send goal: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# --- ROS2 Node ---
class ApiNavigatorNode(Node):
    def __init__(self):
        super().__init__('api_navigator_node')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active. API Navigator Node is ready.')

    def send_goal_to_nav2(self, x: float, y: float, yaw_deg: float):
        """
        Sends a goal to the Nav2 stack.
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y

        yaw_rad = math.radians(yaw_deg)
        goal_pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

        self.get_logger().info(f"Sending goal to Nav2: Pose(x={x}, y={y}, yaw={yaw_deg})")
        self.navigator.goToPose(goal_pose)

        # --- Feedback and Result Handling ---
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f'Distance remaining: {feedback.distance_remaining:.2f} m'
                )

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Goal was canceled!')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Goal failed!')
        else:
            self.get_logger().info(f'Goal has an invalid return status: {result}')

def run_ros_node():
    """Initialize and run the ROS2 node."""
    global ros_node
    rclpy.init()
    ros_node = ApiNavigatorNode()
    # Keep the node running. rclpy.spin() is not used here to avoid blocking the main thread.
    # Instead, we rely on the FastAPI server to keep the process alive.
    # We need to handle shutdown gracefully.
    try:
        # We don't spin here, just let the node exist.
        # The main thread will be running the uvicorn server.
        while rclpy.ok():
            rclpy.spin_once(ros_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()

def main():
    """Main entry point."""
    # Run the ROS2 node in a background thread
    ros_thread = threading.Thread(target=run_ros_node)
    ros_thread.daemon = True
    ros_thread.start()

    # Start the FastAPI server
    uvicorn.run(app, host="192.168.0.100", port=8000)

if __name__ == '__main__':
    main()
