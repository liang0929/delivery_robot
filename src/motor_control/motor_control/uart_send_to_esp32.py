import json
import math
import time
import threading
from typing import Dict, Any

import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point, Vector3
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


class ESP32MotorController(Node):
    """ESP32馬達控制節點"""

    def __init__(self):
        super().__init__('esp32_motor_controller')
        # 宣告參數
        self.declare_parameter('uart_port', '/dev/ttyTHS1')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_separation', 0.381)  # 輪距 (m)
        self.declare_parameter('wheel_radius', 0.065)      # 輪半徑 (m)
        self.declare_parameter('max_linear_vel', 1.11)      # 最大線速度 (m/s)
        self.declare_parameter('max_angular_vel', 2.0)     # 最大角速度 (rad/s)
        
        # 獲取參數
        self.uart_port = self.get_parameter('uart_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # 初始化串口
        self.serial_conn = None
        self.connect_serial()

        # ROS2 發布者和訂閱者
        qos = QoSProfile(depth=10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, qos)
        self.odom_pub = self.create_publisher(Odometry, 'odom_raw', qos)
        
        # 初始化TF廣播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 狀態變數
        self.last_cmd_time = time.time()
        self.odom_data = {
            'x': 0.0, 'y': 0.0, 'theta': 0.0,
            'vx': 0.0, 'vy': 0.0, 'vth': 0.0
        }
        
        # 啟動接收線程
        self.running = True
        self.receive_thread = threading.Thread(target=self.receive_data_thread)
        self.receive_thread.daemon = True
        self.receive_thread.start()

        #數據起始與結束符號
        self.start_word = "::"
        self.end_word = "::"

        # 定時器 - 安全停止檢查
        self.safety_timer = self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info(f'ESP32 Motor Controller initialized on {self.uart_port}')

    def connect_serial(self):
        """連接串口"""
        try:
            self.serial_conn = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate
            )
            time.sleep(2)  # 等待ESP32重啟完成
            self.get_logger().info(f'Connected to ESP32 on {self.uart_port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.uart_port}: {e}')
            self.serial_conn = None

    def cmd_vel_callback(self, msg: Twist):
        """速度命令回調函數"""
        self.last_cmd_time = time.time()
        
        # 限制速度範圍
        linear_x = max(min(msg.linear.x, self.max_linear_vel), -self.max_linear_vel)
        angular_z = max(min(msg.angular.z, self.max_angular_vel), -self.max_angular_vel)
        
        # 差動驅動運算
        left_vel, right_vel = self.diff_drive_kinematics(linear_x, angular_z)
        
        # 發送到ESP32
        self.send_motor_command(left_vel, right_vel)
        
        self.get_logger().debug(
            f'Cmd: linear={linear_x:.3f}, angular={angular_z:.3f} '
            f'-> left={left_vel:.3f}, right={right_vel:.3f}'
        )

    def diff_drive_kinematics(self, linear_x: float, angular_z: float) -> tuple:
        """差動驅動運動學計算"""
        # 計算左右輪速度 (m/s)
        left_vel = linear_x - (angular_z * self.wheel_separation / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_separation / 2.0)
        
        return left_vel, right_vel

    def send_motor_command(self, left_vel: float, right_vel: float):
        """發送馬達控制命令到ESP32"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        
        try:
            cmd = {
                'type': 'motor_cmd',
                'left_vel': round(left_vel, 3),
                'right_vel': round(right_vel, 3),
                'timestamp': time.time()
            }
            message = json.dumps(cmd) + '\n'
            self.serial_conn.write(message.encode())
            self.get_logger().debug(f'Sent: {message.strip()}')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')

    def receive_data_thread(self):
        """接收ESP32數據的線程"""
        while self.running and rclpy.ok():
            if not self.serial_conn or not self.serial_conn.is_open:
                time.sleep(1)
                continue
            try:
                while self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.readline().decode('utf-8',errors='replace')
                    self.process_received_data(data)

            except serial.SerialException as e:
                self.get_logger().error(f'Serial read error: {e}')
                time.sleep(1)   

            except Exception as e:
                self.get_logger().error(f'Data processing error: {e}')

    def process_received_data(self, data: str):
        """處理從ESP32接收的數據"""
        start_idx = data.find(self.start_word)
        end_idx = data.find(self.end_word, start_idx + 2)
        payload = data[start_idx + len(self.start_word):end_idx]
        msg = json.loads(payload) #string to json
        self.get_logger().debug(f'Received: {msg}')
        
        try:
            if msg.get('type') == 'odometry':
                self.update_odometry(msg)
            elif msg.get('type') == 'sensor':
                self.process_sensor_data(msg)
            elif msg.get('type') == 'status':
                self.get_logger().info(f"ESP32 Status: {msg.get('message')}")
                
        except json.JSONDecodeError:
            # 可能是普通的debug訊息
            if data.startswith('ESP32:'):
                self.get_logger().debug(data)

    def update_odometry(self, odom_msg: Dict[str, Any]):
        """更新里程計數據並發布"""
        try:
            # 更新內部狀態
            self.odom_data.update({
                'x': float(odom_msg.get('x', 0.0)),
                'y': float(odom_msg.get('y', 0.0)),
                'theta': float(odom_msg.get('theta', 0.0)),
                'vx': float(odom_msg.get('vx', 0.0)),
                'vy': float(odom_msg.get('vy', 0.0)),
                'vth': float(odom_msg.get('vth', 0.0))
            })

            
            # 發布ROS2里程計訊息和TF
            self.publish_odometry()
            self.publish_tf()
        
        except Exception as e:
            self.get_logger().error(f"Odometry update error: {e}")
                
    def publish_tf(self):
        """發布TF轉換"""
        now = self.get_clock().now()
        
        # 發布 odom -> base_footprint 轉換
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        # 設置位置
        t.transform.translation.x = self.odom_data['x']
        t.transform.translation.y = self.odom_data['y']
        t.transform.translation.z = 0.0
        
        # 設置方向（四元數）
        theta = self.odom_data['theta']
        t.transform.rotation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(theta/2.0),
            w=math.cos(theta/2.0)
        )
        
        # 發布轉換
        self.tf_broadcaster.sendTransform(t)
    
    def publish_odometry(self):
        """發布里程計訊息"""
        odom = Odometry()
        
        # Header
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # 位置
        odom.pose.pose.position = Point(
            x=self.odom_data['x'],
            y=self.odom_data['y'],
            z=0.0
        )
        
        # 方向 (四元數)
        theta = self.odom_data['theta']
        odom.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(theta/2.0),
            w=math.cos(theta/2.0)
        )
        
        # 速度
        odom.twist.twist.linear = Vector3(
            x=self.odom_data['vx'],
            y=self.odom_data['vy'],
            z=0.0
        )
        odom.twist.twist.angular = Vector3(
            x=0.0, y=0.0,
            z=self.odom_data['vth']
        )
        
        # 協方差矩陣 (簡化)
        odom.pose.covariance[0] = 0.1   # x
        odom.pose.covariance[7] = 0.1   # y
        odom.pose.covariance[35] = 0.1  # theta
        
        self.odom_pub.publish(odom)

    def process_sensor_data(self, sensor_msg: Dict[str, Any]):
        """處理感測器數據"""
        # 可以在這裡處理來自ESP32的其他感測器數據
        # 例如: IMU, 編碼器原始數據, 電池電壓等
        sensor_type = sensor_msg.get('sensor_type')
        
        if sensor_type == 'battery':
            voltage = sensor_msg.get('voltage', 0.0)
            self.get_logger().info(f'Battery voltage: {voltage:.2f}V')
        elif sensor_type == 'encoder':
            left_count = sensor_msg.get('left_encoder')
            right_count = sensor_msg.get('right_encoder')
            self.get_logger().debug(f'Encoders - L: {left_count}, R: {right_count}')

    def safety_check(self):
        """安全檢查 - 如果長時間沒收到命令就停止馬達"""
        current_time = time.time()
        if current_time - self.last_cmd_time > 1.0:  # 1秒無命令就停止
            self.send_motor_command(0.0, 0.0)

    def destroy_node(self):
        """節點銷毀時的清理工作"""
        self.running = False
        if hasattr(self, 'receive_thread'):
            self.receive_thread.join(timeout=1.0)
        
        # 停止馬達
        if self.serial_conn and self.serial_conn.is_open:
            self.send_motor_command(0.0, 0.0)
            time.sleep(0.1)
            self.serial_conn.close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = ESP32MotorController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.connect_serial().close()
        pass
    finally:
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()