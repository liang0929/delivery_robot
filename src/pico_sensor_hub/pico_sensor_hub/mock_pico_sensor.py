"""
Mock Pico 感測器集線板節點 - 用於模擬測試

不接硬體也能跑，發布與真節點**完全相同**的 topics（同名、同型別、同 QoS），
讓下游避障／電源監控在沒有 Pico 的情況下照常開發。

刻意保留兩個無效通道（預設：通道 3 逾時、通道 6 故障），
因為下游最容易寫錯的就是無效值處理——mock 全部給漂亮數字的話，
真的上車才發現沒處理 +Inf / NaN 就太晚了。
"""

import math
import random

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pico_sensor_hub.protocol import CHANNEL_SLUGS
from pico_sensor_hub.publishers import (
    DEFAULT_FIELD_OF_VIEW,
    DEFAULT_MAX_RANGE,
    DEFAULT_MIN_RANGE,
    PicoPublishers,
    all_topic_names,
)


class MockPicoSensor(Node):
    """Mock Pico 集線板 - 模擬 8 通道超音波與 INA226 電源"""

    def __init__(self):
        super().__init__('mock_pico_sensor_hub')

        # 宣告參數
        self.declare_parameter('frame_prefix', 'ultrasonic_')
        self.declare_parameter('publish_frequency', 10.0)   # 與韌體的 10 Hz 一致
        self.declare_parameter('field_of_view', DEFAULT_FIELD_OF_VIEW)
        self.declare_parameter('min_range', DEFAULT_MIN_RANGE)
        self.declare_parameter('max_range', DEFAULT_MAX_RANGE)
        # 模擬「沒接感測器」與「通道故障」的通道索引（0–7）
        self.declare_parameter('timeout_channels', [3])
        self.declare_parameter('fault_channels', [6])
        self.declare_parameter('battery_voltage', 25.2)     # 7S 充電上限 28.0V，這裡取中段
        self.declare_parameter('power_invalid', False)      # True 時模擬 INA226 讀取無效

        # 獲取參數
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.timeout_channels = set(self.get_parameter('timeout_channels').value)
        self.fault_channels = set(self.get_parameter('fault_channels').value)
        self.battery_voltage = self.get_parameter('battery_voltage').value
        self.power_invalid = self.get_parameter('power_invalid').value

        self.pubs = PicoPublishers(
            self,
            frame_prefix=self.get_parameter('frame_prefix').value,
            field_of_view=self.get_parameter('field_of_view').value,
            min_range=self.get_parameter('min_range').value,
            max_range=self.get_parameter('max_range').value,
        )

        # 每個通道給不同相位，八個數字才不會整齊劃一地一起動
        self.phase = [i * math.pi / 4.0 for i in range(len(CHANNEL_SLUGS))]
        self.tick = 0

        # 定時器
        publish_period = 1.0 / self.publish_frequency
        self.timer = self.create_timer(publish_period, self.publish_all)

        self.get_logger().info('Mock Pico 集線板啟動 (simulation mode)')
        self.get_logger().info(
            f'  frequency: {self.publish_frequency}Hz,'
            f' 逾時通道: {sorted(self.timeout_channels)},'
            f' 故障通道: {sorted(self.fault_channels)}')
        self.get_logger().info(f'  發布 topics: {", ".join(all_topic_names())}')

    def publish_all(self):
        """發布一輪模擬資料"""
        self.tick += 1
        t = self.tick / self.publish_frequency

        ranges = []
        for i in range(len(CHANNEL_SLUGS)):
            if i in self.fault_channels:
                # 通道故障：量測本身壞了，什麼都不知道 → NaN
                ranges.append(math.nan)
            elif i in self.timeout_channels:
                # 逾時無回波（含未接線）：範圍內沒東西 → +Inf
                ranges.append(math.inf)
            else:
                # 0.3–2.0 m 之間緩慢擺動，外加一點量測噪聲
                base = 1.15 + 0.85 * math.sin(0.5 * t + self.phase[i])
                ranges.append(max(0.05, base + random.gauss(0, 0.005)))
        self.pubs.publish_ranges(ranges)

        if self.power_invalid:
            # 對應真節點的 $PW ok=0：整組不發布
            return

        # 電池電壓緩慢下降（每分鐘約 0.06V），電流隨行走負載起伏
        voltage = self.battery_voltage - 0.001 * t + random.gauss(0, 0.01)
        current = 2.5 + 1.5 * math.sin(0.2 * t) + random.gauss(0, 0.05)
        self.pubs.publish_power(voltage, current, voltage * current)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MockPicoSensor()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
