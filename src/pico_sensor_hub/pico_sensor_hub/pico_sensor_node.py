"""
pico_sensor_hub 節點 - 把 Pico 集線板的序列輸出送進 ROS 2

讀 `/dev/pico_sensor_hub`（udev 固定節點，見 firmware/pico_sensor_hub/README.md 3.3），
逐行做 XOR 校驗，把 $US 發成 8 個 sensor_msgs/Range，$PW 發成三路 Float32。

可靠性要求與韌體同源：**不可以卡死、不可以回報假的近距離**。
因此序列讀取跑在背景執行緒，任何 I/O 例外都只讓連線重來，不讓節點死掉；
距離哨兵值一律走 protocol.distance_mm_to_range_m 映射成 +Inf / NaN，絕不為 0。
"""

import threading
import time

import serial
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from pico_sensor_hub import protocol
from pico_sensor_hub.publishers import (
    DEFAULT_FIELD_OF_VIEW,
    DEFAULT_MAX_RANGE,
    DEFAULT_MIN_RANGE,
    PicoPublishers,
    all_topic_names,
)


class PicoSensorNode(Node):
    """Pico 感測器集線板的序列埠橋接節點"""

    def __init__(self):
        super().__init__('pico_sensor_hub')

        # 宣告參數
        self.declare_parameter('port', '/dev/pico_sensor_hub')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_prefix', 'ultrasonic_')
        self.declare_parameter('field_of_view', DEFAULT_FIELD_OF_VIEW)
        self.declare_parameter('min_range', DEFAULT_MIN_RANGE)
        self.declare_parameter('max_range', DEFAULT_MAX_RANGE)
        self.declare_parameter('reconnect_period', 1.0)
        self.declare_parameter('data_timeout', 3.0)

        # 獲取參數
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.reconnect_period = self.get_parameter('reconnect_period').value
        self.data_timeout = self.get_parameter('data_timeout').value

        self.pubs = PicoPublishers(
            self,
            frame_prefix=self.get_parameter('frame_prefix').value,
            field_of_view=self.get_parameter('field_of_view').value,
            min_range=self.get_parameter('min_range').value,
            max_range=self.get_parameter('max_range').value,
        )

        # 執行期狀態
        self.serial_conn = None
        self.connected = False
        self.checksum_errors = 0
        self.us_frames = 0
        self.last_seq = None
        self.dropped_frames = 0
        self.last_data_time = None
        self.fw_version = None
        self.last_flags = None

        # 序列讀取跑背景執行緒：pyserial 的 read 會阻塞，放進 timer 會拖住
        # executor，連帶讓 log 與參數服務一起卡住。publisher 本身是 thread-safe。
        self._stop = threading.Event()
        self._reader = threading.Thread(
            target=self._reader_loop, name='pico_serial_reader', daemon=True)
        self._reader.start()

        # 連線健康檢查：拔線時 pyserial 不一定拋例外，也可能只是永遠讀不到東西，
        # 所以另外用「多久沒有有效資料」當作斷線判據。
        self.health_timer = self.create_timer(1.0, self._health_check)

        self.get_logger().info('pico_sensor_hub 節點啟動')
        self.get_logger().info(f'  序列埠: {self.port} @ {self.baudrate}')
        self.get_logger().info(f'  發布 topics: {", ".join(all_topic_names())}')

    # ------------------------------------------------------------------
    # 序列埠
    # ------------------------------------------------------------------

    def _open_serial(self):
        """開啟序列埠。失敗回 False，不拋例外——呼叫端會照 reconnect_period 重試。"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.2,          # 讀逾時：讓執行緒有機會看 stop 旗標
            )
            # 開啟瞬間緩衝區裡可能積著拔線前的殘行，丟掉避免解析出半行
            self.serial_conn.reset_input_buffer()
            self.connected = True
            self.last_data_time = time.monotonic()
            self.get_logger().info(f'序列埠已連線: {self.port}')
            return True
        except (serial.SerialException, OSError) as exc:
            self.serial_conn = None
            self.connected = False
            # 拔線期間會每秒重試一次，用 throttle 免得洗版
            self.get_logger().warning(
                f'序列埠開啟失敗 ({self.port}): {exc}，'
                f'{self.reconnect_period:.1f}s 後重試',
                throttle_duration_sec=10.0)
            return False

    def _close_serial(self, reason: str):
        if self.serial_conn is not None:
            try:
                self.serial_conn.close()
            except Exception:      # noqa: BLE001 - 關閉失敗不影響重連
                pass
        self.serial_conn = None
        if self.connected:
            self.get_logger().warning(f'序列埠斷線: {reason}，準備重連')
        self.connected = False

    def _reader_loop(self):
        """背景執行緒：連線 → 逐行讀 → 斷線就重連，直到節點結束。"""
        buf = b''
        while not self._stop.is_set():
            if self.serial_conn is None:
                if not self._open_serial():
                    self._stop.wait(self.reconnect_period)
                    continue
                buf = b''

            try:
                # read(1) 阻塞到「有資料」或 timeout，再一次把緩衝裡的其餘位元組
                # 取乾淨。若改成固定 read(256)，會一路等到 0.2s 逾時才返回，
                # 兩幀被綁在一起處理，時間戳失真（實測 topic hz 的 std dev 0.1s）。
                chunk = self.serial_conn.read(1)
                waiting = self.serial_conn.in_waiting
                if waiting:
                    chunk += self.serial_conn.read(waiting)
            except (serial.SerialException, OSError, TypeError) as exc:
                # 拔掉 USB 時 pyserial 多半在這裡拋；TypeError 是 close() 與
                # read() 撞在一起時 pyserial 內部的表現，一併當成斷線。
                self._close_serial(str(exc))
                buf = b''
                self._stop.wait(self.reconnect_period)
                continue

            if not chunk:
                continue

            buf += chunk
            # 資料太多代表沒有換行、對面不是我們認識的裝置，丟掉避免無限長大
            if len(buf) > 8192:
                self.get_logger().warning('輸入緩衝異常膨脹，丟棄後重新同步')
                buf = b''
                continue

            while b'\n' in buf:
                raw, buf = buf.split(b'\n', 1)
                self._handle_line(raw.decode('ascii', errors='replace'))

    # ------------------------------------------------------------------
    # 解析與發布
    # ------------------------------------------------------------------

    def _handle_line(self, raw: str):
        if not raw.strip():
            return
        try:
            kind, data = protocol.parse_message(raw)
        except protocol.ChecksumError as exc:
            self.checksum_errors += 1
            self.get_logger().warning(
                f'校驗失敗（累計 {self.checksum_errors}）: {exc}',
                throttle_duration_sec=5.0)
            return
        except ValueError as exc:
            # 欄位數不對：最可能是韌體版本與本節點不匹配（例如 $PW 還是 v0.1.x 的五欄）
            self.get_logger().error(
                f'協定欄位不符，請確認韌體版本 ≥ 0.2.0: {exc}',
                throttle_duration_sec=10.0)
            return

        self.last_data_time = time.monotonic()

        if kind == 'US':
            self._on_us(data)
        elif kind == 'PW':
            self._on_pw(data)
        elif kind == 'ST':
            self._on_st(data)
        elif kind == 'ID':
            self._on_id(data)

    def _on_us(self, data):
        self.us_frames += 1
        seq = data['seq']
        if self.last_seq is not None:
            gap = (seq - self.last_seq) & 0xFFFF
            if gap > 1:
                self.dropped_frames += gap - 1
        self.last_seq = seq
        self.pubs.publish_ranges(data['ranges_m'])

    def _on_pw(self, data):
        """ok=0 時整組不發布。

        取捨：Float32 沒有 header 也沒有狀態欄，發 NaN 的話下游只要少一個
        isnan 檢查就會把 NaN 帶進電量計算或門檻比較（NaN 比較永遠為 False，
        會安靜地讓低電壓保護失效）。改成不發布，下游用「topic 多久沒更新」
        判斷資料不可用，這是 ROS 既有且不會被忽略的機制。
        無效原因會在下面的 $ST flags log 出來，不會靜默消失。
        """
        if not self.pubs.publish_power(
                data['bus_v'], data['current_a'], data['power_w']):
            self.get_logger().warning(
                '$PW ok=0：INA226 讀取無效，本輪不發布電源資料',
                throttle_duration_sec=10.0)

    def _on_st(self, data):
        flags = data['flags']
        if flags != self.last_flags:
            # 只在旗標變化時 log，穩態不洗版
            self.get_logger().info(
                f'韌體狀態 flags=0x{flags:04X} ({protocol.format_flags(flags)})'
                f' uptime={data["uptime_s"]:.1f}s'
                f' 逾時累計={data["us_timeout_count"]}'
                f' I2C錯誤={data["i2c_err_count"]}')
            self.last_flags = flags

    def _on_id(self, data):
        version = data['fw_version']
        if version != self.fw_version:
            self.fw_version = version
            self.get_logger().info(
                f'韌體 v{version} (build {data["build_date"]}, {data["board"]})')
        if not version.startswith('0.2'):
            self.get_logger().warning(
                f'韌體版本 {version} 與本節點預期的 $PW 六欄格式（v0.2.0+）不符')

    # ------------------------------------------------------------------
    # 健康檢查
    # ------------------------------------------------------------------

    def _health_check(self):
        """連著但長時間沒資料 → 主動斷開，讓 reader 重新連。

        USB 被拔掉時核心可能只是讓 read 一直回空，不拋例外；沒有這層檢查
        節點會安靜地永遠不再發布，比直接 crash 更難查。
        """
        if not self.connected or self.last_data_time is None:
            return
        idle = time.monotonic() - self.last_data_time
        if idle > self.data_timeout:
            self.get_logger().warning(
                f'已 {idle:.1f}s 沒有收到有效資料（門檻 {self.data_timeout:.1f}s）')
            self._close_serial('資料停滯')

    def destroy_node(self):
        self._stop.set()
        if self._reader.is_alive():
            self._reader.join(timeout=2.0)
        self._close_serial('節點關閉')
        self.get_logger().info(
            f'統計：$US {self.us_frames} 幀、掉幀 {self.dropped_frames}、'
            f'校驗錯誤 {self.checksum_errors}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PicoSensorNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
