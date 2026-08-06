"""
真節點與 mock 節點共用的發布層。

抽出來的唯一理由是**保證兩者的 topic 名稱、訊息型別、QoS、欄位填法完全一致**。
mock 存在的意義就是讓下游在沒有硬體時能照常開發；只要有一處對不上，
mock 就從「替身」變成「另一套 API」，那還不如不要。
"""

import math

from rclpy.qos import QoSProfile
from sensor_msgs.msg import Range
from std_msgs.msg import Float32

from pico_sensor_hub.protocol import CHANNEL_SLUGS

# HC-SR04 規格（韌體 README 6.2）：最遠約 4 m，最近約 2 cm，波束約 15°。
DEFAULT_FIELD_OF_VIEW = 0.26   # rad，約 15°
DEFAULT_MIN_RANGE = 0.02       # m
DEFAULT_MAX_RANGE = 4.0        # m

ULTRASONIC_TOPIC_FMT = 'pico/ultrasonic/{slug}'
VOLTAGE_TOPIC = 'pico/voltage'
CURRENT_TOPIC = 'pico/current'
POWER_TOPIC = 'pico/power'


class PicoPublishers:
    """8 通道 Range + 電源三路 Float32。"""

    def __init__(self, node, frame_prefix='ultrasonic_',
                 field_of_view=DEFAULT_FIELD_OF_VIEW,
                 min_range=DEFAULT_MIN_RANGE,
                 max_range=DEFAULT_MAX_RANGE):
        self.node = node
        self.frame_prefix = frame_prefix
        self.field_of_view = float(field_of_view)
        self.min_range = float(min_range)
        self.max_range = float(max_range)

        qos = QoSProfile(depth=10)

        self.range_pubs = [
            node.create_publisher(Range, ULTRASONIC_TOPIC_FMT.format(slug=slug), qos)
            for slug in CHANNEL_SLUGS
        ]
        # 電壓刻意與 motor/voltage 同型別（Float32、單位 V），
        # 讓下游能用同一套程式吃兩個獨立的電壓來源做交叉比對。
        self.voltage_pub = node.create_publisher(Float32, VOLTAGE_TOPIC, qos)
        self.current_pub = node.create_publisher(Float32, CURRENT_TOPIC, qos)
        self.power_pub = node.create_publisher(Float32, POWER_TOPIC, qos)

    def publish_ranges(self, ranges_m, stamp=None):
        """發布 8 通道距離。ranges_m 允許含 inf / nan（見 protocol 的映射表）。"""
        if stamp is None:
            stamp = self.node.get_clock().now().to_msg()
        for i, value in enumerate(ranges_m):
            msg = Range()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_prefix + CHANNEL_SLUGS[i]
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = self.field_of_view
            msg.min_range = self.min_range
            msg.max_range = self.max_range
            # 這裡直接送 protocol 算好的值。不要在這層再對 inf/nan 做任何
            # 「順手正規化」——把它們變成 0 就是誤急停的來源。
            msg.range = float(value)
            self.range_pubs[i].publish(msg)

    def publish_power(self, bus_v, current_a, power_w):
        """發布電源三路。任一為 None 就整組不發（見節點內的 ok=0 處理說明）。"""
        if bus_v is None or current_a is None or power_w is None:
            return False
        self.voltage_pub.publish(Float32(data=float(bus_v)))
        self.current_pub.publish(Float32(data=float(current_a)))
        self.power_pub.publish(Float32(data=float(power_w)))
        return True


def all_topic_names():
    """本 package 對外的完整 topic 清單，log 與文件用。"""
    return ([ULTRASONIC_TOPIC_FMT.format(slug=s) for s in CHANNEL_SLUGS]
            + [VOLTAGE_TOPIC, CURRENT_TOPIC, POWER_TOPIC])


def is_valid_range(value):
    """是不是一筆真的量到東西的距離（inf/nan 都不是）。"""
    return not (math.isinf(value) or math.isnan(value))
