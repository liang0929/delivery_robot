#!/usr/bin/env python3
"""
Map Relay Node - 解決 slam_toolbox 與 rosbridge QoS 不相容問題

slam_toolbox 發布 /map 使用 TRANSIENT_LOCAL + RELIABLE
rosbridge 訂閱使用 VOLATILE + BEST_EFFORT
此節點轉發地圖到相容的 QoS 設定
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid


class MapRelayNode(Node):
    def __init__(self):
        super().__init__('map_relay')

        # 訂閱 /map (使用與 slam_toolbox 相容的 QoS)
        sub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 發布 /map_relay (使用與 rosbridge 相容的 QoS)
        pub_qos_rosbridge = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 發布 /map_saver (使用與 map_saver 相容的 QoS)
        pub_qos_saver = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            sub_qos
        )

        # 給 rosbridge/前端用
        self.publisher_relay = self.create_publisher(
            OccupancyGrid,
            '/map_relay',
            pub_qos_rosbridge
        )

        # 給 map_saver 用
        self.publisher_saver = self.create_publisher(
            OccupancyGrid,
            '/map_saver',
            pub_qos_saver
        )

        self.get_logger().info('Map relay node started: /map -> /map_relay, /map_saver')

    def map_callback(self, msg: OccupancyGrid):
        self.publisher_relay.publish(msg)
        self.publisher_saver.publish(msg)


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = MapRelayNode()
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()
