#!/usr/bin/env python3
"""
picoterm.py — pico_sensor_hub 韌體的主機端驗證工具

只用 Python 標準庫（termios + os），不需要 pyserial，也不需要 pip 裝任何東西。
若系統上剛好有 pyserial（apt install python3-serial），可用 --backend pyserial 切換。

功能：
  * 逐行讀取 /dev/ttyACM0
  * 驗證每一行的 XOR 校驗，統計錯誤率
  * 統計 seq 掉幀率
  * 統計實際更新頻率（$US 應為 10 Hz）
  * 以車體位置（前左／前右／…）即時顯示八個方位的距離
  * --duration N 跑 N 秒後印統計摘要

用法：
  ./picoterm.py                        # 即時表格，Ctrl-C 結束
  ./picoterm.py --duration 300         # 跑 5 分鐘後印摘要（驗收判準 7）
  ./picoterm.py --raw                  # 只印原始行，方便除錯
  ./picoterm.py --send 'CMD,PING'      # 送一條指令後繼續監看
  ./picoterm.py --duration 20 --no-table   # 純統計，不畫表格
"""

import argparse
import os
import select
import sys
import time

# 通道索引 → 車體位置。順序對應 $US 的 d1..d8，
# 來源是工單「呼叫鏈與資料流」那張腳位表。
CHANNEL_NAMES = ["前左", "前右", "右前", "右後", "後右", "後左", "左後", "左前"]

# $ST flags 的位元意義，與韌體 main.c 的定義一致
FLAG_BITS = [
    (1 << 0, "INA226故障"),
    (1 << 1, "看門狗重開"),
    (1 << 2, "超音波逾時"),
    (1 << 3, "USB輸出丟棄"),
    (1 << 4, "INA226校正失敗"),
    (1 << 5, "指令異常"),
    (1 << 6, "通道故障"),
]


# --------------------------------------------------------------------------
# 協定
# --------------------------------------------------------------------------

def xor_checksum(payload: str) -> int:
    """payload 每個位元組的 XOR。校驗範圍不含 '$' 與 '*'。"""
    x = 0
    for b in payload.encode("utf-8", errors="replace"):
        x ^= b
    return x


def parse_line(line: str):
    """
    回傳 (ok, payload)。
    ok=False 代表格式錯誤或校驗不符 —— 兩者都算校驗失敗。
    """
    line = line.strip()
    if not line.startswith("$"):
        return False, line
    star = line.rfind("*")
    if star < 0 or star + 3 != len(line):
        return False, line
    payload = line[1:star]
    try:
        want = int(line[star + 1:], 16)
    except ValueError:
        return False, line
    return xor_checksum(payload) == want, payload


def build_line(payload: str) -> bytes:
    return ("$%s*%02X\n" % (payload, xor_checksum(payload))).encode("ascii")


# --------------------------------------------------------------------------
# 序列埠後端
# --------------------------------------------------------------------------

class TermiosPort:
    """純標準庫的 CDC-ACM 讀寫。CDC 是虛擬序列埠，baudrate 實際上不影響傳輸。"""

    def __init__(self, dev: str):
        import termios

        self.termios = termios
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)

        attrs = termios.tcgetattr(self.fd)
        iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs

        # raw 模式：不做任何字元轉換，否則 \r\n 與 XOR 校驗會對不起來
        iflag &= ~(termios.IGNBRK | termios.BRKINT | termios.PARMRK |
                   termios.ISTRIP | termios.INLCR | termios.IGNCR |
                   termios.ICRNL | termios.IXON)
        oflag &= ~termios.OPOST
        lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON |
                   termios.ISIG | termios.IEXTEN)
        cflag &= ~(termios.CSIZE | termios.PARENB)
        cflag |= termios.CS8 | termios.CREAD | termios.CLOCAL

        cc[termios.VMIN] = 0
        cc[termios.VTIME] = 0

        termios.tcsetattr(
            self.fd, termios.TCSANOW,
            [iflag, oflag, cflag, lflag, ispeed, ospeed, cc])
        termios.tcflush(self.fd, termios.TCIFLUSH)

    def read(self, n: int = 65536) -> bytes:
        """
        阻塞最多 50ms 等資料，而不是「沒資料就 sleep 固定時間」。

        原本的輪詢寫法在主機負載高時會落後：程序被排程延遲，核心 tty 緩衝區
        累積，最後溢位丟行。select 讓核心一有資料就叫醒我們，把落後的機會壓到最低。
        """
        try:
            r, _, _ = select.select([self.fd], [], [], 0.05)
            if not r:
                return b""
            data = os.read(self.fd, n)
            return data if data else b""
        except BlockingIOError:
            return b""
        except InterruptedError:
            return b""
        except OSError as exc:
            raise IOError("讀取失敗：%s" % exc)

    def write(self, data: bytes) -> None:
        try:
            os.write(self.fd, data)
        except OSError as exc:
            print("寫入失敗：%s" % exc, file=sys.stderr)

    def close(self) -> None:
        try:
            os.close(self.fd)
        except OSError:
            pass


class PySerialPort:
    def __init__(self, dev: str):
        import serial
        # timeout 與 termios 後端的 select 逾時一致，避免忙碌輪詢
        self.ser = serial.Serial(dev, 115200, timeout=0.05)

    def read(self, n: int = 65536) -> bytes:
        # in_waiting 有多少就讀多少，避免一次只取一小塊而落後
        want = max(self.ser.in_waiting, 1)
        return self.ser.read(min(want, n))

    def write(self, data: bytes) -> None:
        self.ser.write(data)

    def close(self) -> None:
        self.ser.close()


def open_port(dev: str, backend: str):
    if backend == "pyserial":
        return PySerialPort(dev)
    if backend == "termios":
        return TermiosPort(dev)
    # auto：優先用不需要額外套件的 termios
    try:
        return TermiosPort(dev)
    except Exception as exc:            # noqa: BLE001 - 退回 pyserial 再試一次
        print("termios 後端開啟失敗（%s），改用 pyserial" % exc, file=sys.stderr)
        return PySerialPort(dev)


# --------------------------------------------------------------------------
# 統計
# --------------------------------------------------------------------------

class Stats:
    def __init__(self):
        self.lines_total = 0
        self.checksum_ok = 0
        self.checksum_bad = 0
        self.bad_samples = []

        self.msg_counts = {}          # "US" / "PW" / "ST" / "ID" / ...

        self.us_frames = 0
        self.us_dropped = 0           # 由 seq 跳號推算的掉幀數
        self.last_us_seq = None

        self.first_us_time = None
        self.last_us_time = None

        self.last_us = None           # 最後一筆八通道距離
        self.last_pw = None           # (ok, bus_mV, current_mA, power_mW)
        self.last_st = None           # (uptime_ms, flags, us_timeout, i2c_err)
        self.last_id = None
        self.other_lines = []         # ACK / NAK / JT / I2C 等

    # -- 記錄 --------------------------------------------------------------

    def feed(self, raw: str):
        self.lines_total += 1
        ok, payload = parse_line(raw)
        if not ok:
            self.checksum_bad += 1
            if len(self.bad_samples) < 10:
                self.bad_samples.append(raw.strip())
            return
        self.checksum_ok += 1

        fields = payload.split(",")
        kind = fields[0]
        self.msg_counts[kind] = self.msg_counts.get(kind, 0) + 1

        if kind == "US":
            self._feed_us(fields)
        elif kind == "PW":
            self._feed_pw(fields)
        elif kind == "ST":
            self._feed_st(fields)
        elif kind == "ID":
            self.last_id = payload
        else:
            self.other_lines.append(payload)
            if len(self.other_lines) > 40:
                self.other_lines.pop(0)

    def _feed_us(self, f):
        if len(f) != 10:
            return
        try:
            seq = int(f[1])
            dists = [int(x) for x in f[2:10]]
        except ValueError:
            return

        now = time.monotonic()
        if self.first_us_time is None:
            self.first_us_time = now
        self.last_us_time = now

        self.us_frames += 1
        self.last_us = dists

        if self.last_us_seq is not None:
            gap = (seq - self.last_us_seq) & 0xFFFF
            if gap > 1:
                self.us_dropped += gap - 1
        self.last_us_seq = seq

    def _feed_pw(self, f):
        # 韌體 v0.2.0 起：$PW,<seq>,<ok>,<mV>,<mA>,<mW> —— 六欄。
        # 舊的五欄格式（無 ok 欄）直接忽略，避免把 v0.1.x 的 mV 誤讀成 ok。
        if len(f) != 6:
            return
        try:
            self.last_pw = (int(f[2]), int(f[3]), int(f[4]), int(f[5]))
        except ValueError:
            pass

    def _feed_st(self, f):
        if len(f) != 5:
            return
        try:
            self.last_st = (int(f[1]), int(f[2], 16), int(f[3]), int(f[4]))
        except ValueError:
            pass

    # -- 衍生指標 ----------------------------------------------------------

    @property
    def rate_hz(self):
        if (self.first_us_time is None or self.last_us_time is None
                or self.us_frames < 2):
            return 0.0
        span = self.last_us_time - self.first_us_time
        if span <= 0:
            return 0.0
        return (self.us_frames - 1) / span

    @property
    def drop_rate(self):
        expected = self.us_frames + self.us_dropped
        if expected == 0:
            return 0.0
        return self.us_dropped / expected

    @property
    def checksum_err_rate(self):
        if self.lines_total == 0:
            return 0.0
        return self.checksum_bad / self.lines_total


# --------------------------------------------------------------------------
# 顯示
# --------------------------------------------------------------------------

def fmt_dist(mm):
    if mm == -1:
        return "  逾時"
    if mm == -2:
        return "  故障"
    return "%4d mm" % mm


def fmt_flags(flags):
    if flags == 0:
        return "無"
    names = [name for bit, name in FLAG_BITS if flags & bit]
    unknown = flags & ~sum(bit for bit, _ in FLAG_BITS)
    if unknown:
        names.append("未知位元 0x%X" % unknown)
    return " | ".join(names)


def render_table(st: Stats, elapsed: float, duration):
    lines = []
    lines.append("=" * 62)
    tail = "/%ds" % duration if duration else ""
    lines.append(" pico_sensor_hub 監看     已跑 %6.1fs%s" % (elapsed, tail))
    lines.append("=" * 62)

    if st.last_id:
        lines.append(" 韌體  %s" % st.last_id)

    lines.append("-" * 62)
    if st.last_us:
        # 依車體方位排成兩欄，讀起來比一長串數字快
        for i in range(0, 8, 2):
            left = "%s %s" % (CHANNEL_NAMES[i], fmt_dist(st.last_us[i]))
            right = "%s %s" % (CHANNEL_NAMES[i + 1], fmt_dist(st.last_us[i + 1]))
            lines.append("  %-26s %-26s" % (left, right))
    else:
        lines.append("  （尚未收到 $US）")

    lines.append("-" * 62)
    if st.last_pw:
        ok, bus, cur, pwr = st.last_pw
        if not ok:
            lines.append("  電源   讀取無效（ok=0，見狀態旗標）")
        else:
            lines.append("  電源   %.3f V   %+d mA   %d mW"
                         % (bus / 1000.0, cur, pwr))
    else:
        lines.append("  電源   （尚未收到 $PW）")

    if st.last_st:
        uptime, flags, us_to, i2c_err = st.last_st
        lines.append("  狀態   uptime %.1fs  逾時累計 %d  I2C錯誤 %d"
                     % (uptime / 1000.0, us_to, i2c_err))
        lines.append("  旗標   0x%04X  %s" % (flags, fmt_flags(flags)))

    lines.append("-" * 62)
    lines.append("  更新率 %.2f Hz    幀數 %d    掉幀 %d (%.3f%%)"
                 % (st.rate_hz, st.us_frames, st.us_dropped,
                    st.drop_rate * 100.0))
    lines.append("  校驗   OK %d / 錯 %d (%.4f%%)"
                 % (st.checksum_ok, st.checksum_bad,
                    st.checksum_err_rate * 100.0))

    if st.other_lines:
        lines.append("-" * 62)
        for entry in st.other_lines[-6:]:
            lines.append("  > %s" % entry)

    return "\n".join(lines)


def print_summary(st: Stats, elapsed: float):
    print()
    print("=" * 62)
    print(" 統計摘要（%.1f 秒）" % elapsed)
    print("=" * 62)
    print("  總行數          %d" % st.lines_total)
    print("  校驗通過        %d" % st.checksum_ok)
    print("  校驗錯誤        %d  (%.4f%%)"
          % (st.checksum_bad, st.checksum_err_rate * 100.0))
    print("  $US 幀數        %d" % st.us_frames)
    print("  掉幀            %d  (%.4f%%)"
          % (st.us_dropped, st.drop_rate * 100.0))
    print("  實測更新率      %.3f Hz   (目標 10 Hz ±10%% → 9.0–11.0)"
          % st.rate_hz)

    if st.msg_counts:
        kinds = ", ".join("%s=%d" % (k, v)
                          for k, v in sorted(st.msg_counts.items()))
        print("  訊息分佈        %s" % kinds)

    if st.last_st:
        uptime, flags, us_to, i2c_err = st.last_st
        print("  最後 uptime     %.1f s" % (uptime / 1000.0))
        print("  旗標            0x%04X  %s" % (flags, fmt_flags(flags)))
        print("  逾時累計        %d" % us_to)
        print("  I2C 錯誤        %d" % i2c_err)

    if st.last_us:
        print("  最後距離        %s"
              % "  ".join("%s=%s" % (CHANNEL_NAMES[i], fmt_dist(d).strip())
                          for i, d in enumerate(st.last_us)))

    if st.bad_samples:
        print("  校驗錯誤樣本：")
        for s in st.bad_samples:
            print("    %r" % s)

    # 判準對照，直接給出通過與否，省得人工換算
    print("-" * 62)
    ok_cksum = st.checksum_bad == 0
    ok_drop = st.drop_rate < 0.001
    ok_rate = 9.0 <= st.rate_hz <= 11.0
    print("  校驗錯誤 0 筆       %s" % ("PASS" if ok_cksum else "FAIL"))
    print("  掉幀率 < 0.1%%       %s (%.4f%%)"
          % ("PASS" if ok_drop else "FAIL", st.drop_rate * 100.0))
    print("  更新率 10Hz ±10%%    %s (%.3f Hz)"
          % ("PASS" if ok_rate else "FAIL", st.rate_hz))
    print("=" * 62)

    return ok_cksum and ok_drop and ok_rate


# --------------------------------------------------------------------------
# 主程式
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description="pico_sensor_hub 序列協定驗證工具")
    ap.add_argument("-d", "--device", default="/dev/ttyACM0")
    ap.add_argument("--duration", type=float, default=None,
                    metavar="N", help="跑 N 秒後印統計摘要並結束")
    ap.add_argument("--raw", action="store_true",
                    help="只印原始行，不畫表格")
    ap.add_argument("--no-table", action="store_true",
                    help="不畫即時表格（只在結束時印摘要）")
    ap.add_argument("--send", action="append", default=[], metavar="PAYLOAD",
                    help="開始時送出的指令 payload，例如 'CMD,PING'（可重複）")
    ap.add_argument("--send-raw", action="append", default=[], metavar="LINE",
                    help="原樣送出一行，不補校驗。用來驗證韌體面對畸形／"
                         "校驗錯誤的輸入時會回 $NAK 而不是當機（可重複）")
    ap.add_argument("--backend", choices=["auto", "termios", "pyserial"],
                    default="auto")
    ap.add_argument("--no-query-id", action="store_true",
                    help="不要在連上後主動查詢 $ID。韌體只在 USB 斷線→連線"
                         "轉換時自動送橫幅，重複連線（DTR 未真正斷開）時不會重送，"
                         "所以預設主動問一次，確保永遠知道跑的是哪一版。")
    ap.add_argument("--refresh", type=float, default=0.5,
                    help="表格更新間隔秒數（預設 0.5）")
    args = ap.parse_args()

    if not os.path.exists(args.device):
        print("找不到 %s —— Pico 沒插上、還在 BOOTSEL 模式，或韌體沒跑起來"
              % args.device, file=sys.stderr)
        return 2

    try:
        port = open_port(args.device, args.backend)
    except Exception as exc:            # noqa: BLE001
        print("開啟 %s 失敗：%s" % (args.device, exc), file=sys.stderr)
        return 2

    if not args.no_query_id:
        port.write(build_line("CMD,ID"))
        time.sleep(0.05)

    for payload in args.send:
        line = build_line(payload)
        port.write(line)
        print("送出：%s" % line.decode("ascii").rstrip())
        time.sleep(0.1)

    for raw in args.send_raw:
        data = (raw + "\n").encode("utf-8", errors="replace")
        port.write(data)
        print("原樣送出：%r" % raw)
        time.sleep(0.1)

    st = Stats()
    buf = b""
    start = time.monotonic()
    last_render = 0.0
    rc = 0

    try:
        while True:
            now = time.monotonic()
            elapsed = now - start

            if args.duration is not None and elapsed >= args.duration:
                break

            # read() 內部用 select 阻塞最多 50ms，不需要額外 sleep
            chunk = port.read()
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    text = raw.decode("utf-8", errors="replace")
                    if not text.strip():
                        continue
                    if args.raw:
                        print(text.strip())
                    st.feed(text)

            if not args.raw and not args.no_table:
                if now - last_render >= args.refresh:
                    # 清畫面用 ANSI，避免依賴 curses
                    sys.stdout.write("\033[2J\033[H")
                    sys.stdout.write(render_table(st, elapsed, args.duration))
                    sys.stdout.write("\n")
                    sys.stdout.flush()
                    last_render = now

    except KeyboardInterrupt:
        print("\n（中斷）")
    finally:
        elapsed = time.monotonic() - start
        port.close()
        passed = print_summary(st, elapsed)
        if args.duration is not None and not passed:
            rc = 1

    return rc


if __name__ == "__main__":
    sys.exit(main())
