"""
Mock LiDAR 節點 - 用於模擬測試

模擬 2D LiDAR 掃描數據，可設定簡單的虛擬環境。
支援多種場景：空曠、方形房間、走廊等。
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


class MockLidar(Node):
    """Mock LiDAR 節點 - 模擬 2D 雷射掃描"""

    def __init__(self):
        super().__init__('mock_lidar')

        # 宣告參數
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('scan_frequency', 10.0)
        self.declare_parameter('angle_min', -math.pi)
        self.declare_parameter('angle_max', math.pi)
        self.declare_parameter('angle_increment', math.pi / 180.0)  # 1 度
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('scene', 'room')  # 場景: empty, room, corridor

        # 獲取參數
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_frequency = self.get_parameter('scan_frequency').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.scene = self.get_parameter('scene').value

        # 計算射線數量
        self.num_readings = int(
            (self.angle_max - self.angle_min) / self.angle_increment)

        # ROS2 發布者
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.scan_pub = self.create_publisher(LaserScan, 'scan', qos)

        # 定時器
        scan_period = 1.0 / self.scan_frequency
        self.scan_timer = self.create_timer(scan_period, self.publish_scan)

        self.get_logger().info(
            f'Mock LiDAR initialized (simulation mode)'
        )
        self.get_logger().info(
            f'  scene: {self.scene}, readings: {self.num_readings}'
        )

    def get_simulated_ranges(self) -> list:
        """根據場景生成模擬距離數據"""
        ranges = []

        for i in range(self.num_readings):
            angle = self.angle_min + i * self.angle_increment

            if self.scene == 'empty':
                # 空曠場景 - 全部最大距離
                distance = self.range_max
            elif self.scene == 'room':
                # 方形房間 (5m x 5m)
                distance = self._room_distance(angle, 5.0, 5.0)
            elif self.scene == 'corridor':
                # 走廊 (寬 2m，長 10m)
                distance = self._corridor_distance(angle, 2.0, 10.0)
            else:
                distance = self.range_max

            # 添加一點噪聲 (模擬真實 LiDAR)
            import random
            noise = random.gauss(0, 0.01)
            distance = max(self.range_min, min(distance + noise, self.range_max))

            ranges.append(distance)

        return ranges

    def _room_distance(self, angle: float, width: float, height: float) -> float:
        """計算到方形房間牆壁的距離"""
        # 機器人在房間中央
        half_w = width / 2.0
        half_h = height / 2.0

        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        # 避免除以零
        if abs(cos_a) < 1e-6:
            cos_a = 1e-6
        if abs(sin_a) < 1e-6:
            sin_a = 1e-6

        # 計算到四面牆的距離
        distances = []

        # 右牆 (x = half_w)
        if cos_a > 0:
            d = half_w / cos_a
            y = d * sin_a
            if abs(y) <= half_h:
                distances.append(d)

        # 左牆 (x = -half_w)
        if cos_a < 0:
            d = -half_w / cos_a
            y = d * sin_a
            if abs(y) <= half_h:
                distances.append(d)

        # 前牆 (y = half_h)
        if sin_a > 0:
            d = half_h / sin_a
            x = d * cos_a
            if abs(x) <= half_w:
                distances.append(d)

        # 後牆 (y = -half_h)
        if sin_a < 0:
            d = -half_h / sin_a
            x = d * cos_a
            if abs(x) <= half_w:
                distances.append(d)

        return min(distances) if distances else self.range_max

    def _corridor_distance(self, angle: float, width: float, length: float) -> float:
        """計算到走廊牆壁的距離"""
        # 走廊沿 x 軸方向
        half_w = width / 2.0
        half_l = length / 2.0

        cos_a = math.cos(angle)
        sin_a = math.sin(angle)

        if abs(cos_a) < 1e-6:
            cos_a = 1e-6
        if abs(sin_a) < 1e-6:
            sin_a = 1e-6

        distances = []

        # 側牆 (y = ±half_w)
        if sin_a > 0:
            d = half_w / sin_a
            x = d * cos_a
            if abs(x) <= half_l:
                distances.append(d)
        if sin_a < 0:
            d = -half_w / sin_a
            x = d * cos_a
            if abs(x) <= half_l:
                distances.append(d)

        # 前後牆 (x = ±half_l)
        if cos_a > 0:
            d = half_l / cos_a
            y = d * sin_a
            if abs(y) <= half_w:
                distances.append(d)
        if cos_a < 0:
            d = -half_l / cos_a
            y = d * sin_a
            if abs(y) <= half_w:
                distances.append(d)

        return min(distances) if distances else self.range_max

    def publish_scan(self):
        """發布 LaserScan 訊息"""
        scan = LaserScan()

        # Header
        scan.header = Header()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id

        # 掃描參數
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = (1.0 / self.scan_frequency) / self.num_readings
        scan.scan_time = 1.0 / self.scan_frequency
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        # 距離數據
        scan.ranges = self.get_simulated_ranges()
        scan.intensities = [100.0] * len(scan.ranges)  # 模擬強度

        self.scan_pub.publish(scan)


def main(args=None):
    rclpy.init(args=args)

    try:
        lidar = MockLidar()
        rclpy.spin(lidar)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
