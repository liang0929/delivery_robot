"""低電壓保護的純狀態機（不依賴 rclpy，可離線單元測試）。

抽出來的理由與 kinematics.py / pico_sensor_hub.protocol 相同：
判斷「要不要停機」是這個功能唯一有風險的部分，必須能在沒有 ROS、沒有硬體、
沒有真實時間的條件下逐條驗證。節點層只負責訂閱 / 發布 / 取時間，
所有門檻、遲滯、持續時間、雙源仲裁的決策都在這裡。

## 決策鏈（evaluate 的順序）

1. **收樣**：``submit()`` 每筆樣本先過有效值檢查（NaN/Inf 與
   [min_valid_voltage, max_valid_voltage] 之外一律丟棄）。驅動器封包解析
   異常時可能吐出 0.0，這種值若直接進門檻比較會在 shutdown_duration_sec
   後把機器人停死，所以擋在最前面。
2. **濾波**：每個來源各自對 filter_window_sec 內的樣本取算術平均。
   馬達加速瞬間的電壓 sag 是真的（電池內阻 × 電流），但它不代表電池電量，
   靠時間窗平均先削掉大部分。
3. **仲裁**：只取「新鮮」（last_stamp 在 source_timeout_sec 內）的來源，
   對它們的平均值取 **min**。取小值是刻意的保守選擇，見 _arbitrate。
4. **持續時間 + 遲滯**：仲裁值低於門檻要連續滿 N 秒才改狀態；
   回升必須超過「門檻 + hysteresis_voltage」才把計時歸零，
   避免在門檻上下抖動時計時器被反覆重置而永遠湊不滿 N 秒。
5. **鎖存**：SHUTDOWN 一旦成立就不再解除（``reset()`` 只給測試用）。
   負載一停電壓就會回彈，把回彈當成「電池復原」正是過放的典型死法。

## 狀態

UNKNOWN → 沒有任何新鮮來源，不知道電壓，**不觸發**停機（沒資料不等於低電壓）。
OK      → 仲裁電壓正常。
WARNING → 低於 warn_voltage 持續 warn_duration_sec；可因電壓回升自行解除。
SHUTDOWN→ 低於 shutdown_voltage 持續 shutdown_duration_sec；鎖存，不解除。
"""

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

# 狀態枚舉（字串而非 IntEnum：直接進 log 與 DiagnosticStatus 的 message 欄位，
# 讀 topic 的人不必再查對照表）
STATE_UNKNOWN = 'UNKNOWN'
STATE_OK = 'OK'
STATE_WARNING = 'WARNING'
STATE_SHUTDOWN = 'SHUTDOWN'

# 電壓來源代號
SOURCE_MOTOR = 'motor'   # /motor/voltage，AGV-BLD-2S 驅動器回報
SOURCE_PICO = 'pico'     # /pico/voltage，Pico 集線板 INA226 實測

DEFAULT_SOURCES = (SOURCE_MOTOR, SOURCE_PICO)


@dataclass
class BatteryPolicyConfig:
    """門檻與時間常數。預設值與 config/battery_guard.yaml 一致。

    7S 鋰電（3.7V × 7 = 25.9V 標稱）：
      - warn_voltage 22.4V ≈ 每串 3.2V，剩餘電量已不多，該回充電座。
      - shutdown_voltage 21.7V ≈ 每串 3.1V，再放下去進入陡降段，過放傷電池。
    """

    warn_voltage: float = 22.4
    shutdown_voltage: float = 21.7
    # 回升遲滯：要高過「門檻 + 此值」才算脫離該區間
    hysteresis_voltage: float = 0.3
    # 低於門檻必須連續維持多久才改變狀態（抗加速 sag 誤觸發的主力）
    warn_duration_sec: float = 2.0
    shutdown_duration_sec: float = 3.0
    # 每個來源的滑動平均時間窗
    filter_window_sec: float = 1.0
    # 超過這麼久沒收到樣本就視為該來源消失（pico ok=0 時整組不發布）
    source_timeout_sec: float = 2.0
    # 明顯不可能的讀值直接丟棄（例如解析異常的 0.0）
    min_valid_voltage: float = 5.0
    max_valid_voltage: float = 60.0

    def validate(self) -> None:
        """檢查設定自洽，不合理直接拋 ValueError（比默默用壞參數保護失效好）。"""
        errors = []
        if self.shutdown_voltage >= self.warn_voltage:
            errors.append(
                f'shutdown_voltage ({self.shutdown_voltage}) must be lower than '
                f'warn_voltage ({self.warn_voltage})')
        if self.hysteresis_voltage < 0:
            errors.append(
                f'hysteresis_voltage must be >= 0, got {self.hysteresis_voltage}')
        # 遲滯過大會讓 shutdown 的回升門檻蓋過 warn 門檻，狀態機語意變混亂
        if self.shutdown_voltage + self.hysteresis_voltage > self.warn_voltage:
            errors.append(
                f'shutdown_voltage + hysteresis_voltage '
                f'({self.shutdown_voltage + self.hysteresis_voltage}) must not exceed '
                f'warn_voltage ({self.warn_voltage})')
        for name in ('warn_duration_sec', 'shutdown_duration_sec',
                     'filter_window_sec', 'source_timeout_sec'):
            value = getattr(self, name)
            if value <= 0:
                errors.append(f'{name} must be positive, got {value}')
        if self.min_valid_voltage >= self.max_valid_voltage:
            errors.append(
                f'min_valid_voltage ({self.min_valid_voltage}) must be lower than '
                f'max_valid_voltage ({self.max_valid_voltage})')
        if self.min_valid_voltage > self.shutdown_voltage:
            # 否則真的放到 shutdown_voltage 以下時樣本全被當成無效值丟掉，保護失效
            errors.append(
                f'min_valid_voltage ({self.min_valid_voltage}) must not exceed '
                f'shutdown_voltage ({self.shutdown_voltage})')
        if errors:
            raise ValueError('Invalid battery policy config: ' + '; '.join(errors))


@dataclass
class BatteryDecision:
    """一次 evaluate 的完整結果，節點層照著填 topic 與 log。"""

    state: str
    # 仲裁後的電壓（已濾波）；沒有任何新鮮來源時為 None
    voltage: Optional[float]
    # 各來源濾波後的值，None 代表該來源不新鮮 / 沒資料
    source_voltages: Dict[str, Optional[float]] = field(default_factory=dict)
    # 實際參與仲裁的來源（依 DEFAULT_SOURCES 順序）
    sources_used: Tuple[str, ...] = ()
    # 停機鎖存旗標。與 state == STATE_SHUTDOWN 等價，另外給一欄是因為
    # 節點層要用它決定是否持續發布 /safety/stop，語意上是「命令」不是「狀態」。
    stop_latched: bool = False
    # 人看的原因字串，進 log 與 DiagnosticStatus.message
    reason: str = ''


class _VoltageSource:
    """單一電壓來源的樣本窗。"""

    def __init__(self, name: str):
        self.name = name
        self._samples = deque()      # [(stamp, value), ...]，時間遞增
        self.last_stamp: Optional[float] = None
        self.last_raw: Optional[float] = None
        self.accepted = 0
        self.rejected = 0

    def add(self, value: float, now: float) -> None:
        self._samples.append((now, value))
        self.last_stamp = now
        self.last_raw = value
        self.accepted += 1

    def is_fresh(self, now: float, timeout_sec: float) -> bool:
        if self.last_stamp is None:
            return False
        return (now - self.last_stamp) <= timeout_sec

    def average(self, now: float, window_sec: float) -> Optional[float]:
        """時間窗內樣本的算術平均。

        刻意**永遠保留最新一筆**：window (1s) 通常小於 timeout (2s)，
        若照時間窗把樣本清光，來源明明還算新鮮卻取不到值，
        會在 1~2 秒的空窗裡誤判成「來源消失」。
        """
        cutoff = now - window_sec
        while len(self._samples) > 1 and self._samples[0][0] < cutoff:
            self._samples.popleft()
        if not self._samples:
            return None
        return sum(v for _, v in self._samples) / len(self._samples)


class BatteryPolicy:
    """低電壓保護狀態機。所有時間都由呼叫端傳入（單調秒數），內部不取時鐘。"""

    def __init__(self, config: Optional[BatteryPolicyConfig] = None,
                 sources: Tuple[str, ...] = DEFAULT_SOURCES):
        self.config = config or BatteryPolicyConfig()
        self.config.validate()
        self._sources = {name: _VoltageSource(name) for name in sources}

        # 「低於門檻」的起算時間；None = 目前不在低壓區間（或已被遲滯清掉）
        self._below_warn_since: Optional[float] = None
        self._below_shutdown_since: Optional[float] = None

        self._warning_active = False
        self._stop_latched = False

    # ------------------------------------------------------------------
    # 收樣
    # ------------------------------------------------------------------

    def submit(self, source: str, value: float, now: float) -> bool:
        """餵入一筆電壓樣本。回傳是否被接受（False = 無效值，已丟棄）。"""
        if source not in self._sources:
            raise KeyError(f'unknown voltage source: {source}')
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            self._sources[source].rejected += 1
            return False
        if not (self.config.min_valid_voltage <= value <= self.config.max_valid_voltage):
            self._sources[source].rejected += 1
            return False
        self._sources[source].add(float(value), now)
        return True

    def rejected_count(self, source: str) -> int:
        return self._sources[source].rejected

    # ------------------------------------------------------------------
    # 仲裁與判定
    # ------------------------------------------------------------------

    def _arbitrate(self, now: float):
        """回傳 (仲裁值, {來源: 濾波值}, 參與仲裁的來源)。

        雙源都在 → 取 **min**。理由：
          - 兩個來源量的是同一組電池，正常情況差距只有線損與 ADC 誤差；
            取小值等於選擇「先保護、晚一點才停」的那一邊反過來，
            寧可早停也不要因為某一路讀得偏高而錯過過放。
          - 取 max 會讓任何一路讀高就掩蓋真實低電壓，保護等於形同虛設。
          - 取平均則在一路壞掉（讀成 30V）時把真實的 21V 拉高到 25V，同樣失效。
          - 取小值的代價是「誤停」，但誤停被第 4 步的持續時間 + 濾波擋掉，
            而且誤停的後果（機器人停住）遠小於漏停（電池報廢）。
        單源 → 直接用該源，不因為少一路就降低保護（pico ok=0 不發布是常態）。
        """
        per_source: Dict[str, Optional[float]] = {}
        used = []
        values = []
        # 依建構時的來源順序走（dict 保序），不寫死 DEFAULT_SOURCES，
        # 免得日後多接一路電壓源時仲裁悄悄漏掉它
        for name, src in self._sources.items():
            if not src.is_fresh(now, self.config.source_timeout_sec):
                per_source[name] = None
                continue
            avg = src.average(now, self.config.filter_window_sec)
            per_source[name] = avg
            if avg is not None:
                used.append(name)
                values.append(avg)
        arbitrated = min(values) if values else None
        return arbitrated, per_source, tuple(used)

    def _update_timer(self, since: Optional[float], voltage: float,
                      threshold: float, now: float) -> Optional[float]:
        """更新單一門檻的「持續低於」計時起點，含遲滯。

        - 低於門檻：沒在計時就開始計時，已在計時就維持原起點（繼續累積）。
        - 高於「門檻 + 遲滯」：脫離低壓區，計時歸零。
        - 落在遲滯帶 [門檻, 門檻+遲滯) 之間：**維持現狀**。
          這是遲滯的重點——電壓恰好在門檻附近抖動時不歸零，
          否則每次抖回門檻之上就重新計時，永遠湊不滿持續時間，保護不會生效。
        """
        if voltage < threshold:
            return now if since is None else since
        if voltage >= threshold + self.config.hysteresis_voltage:
            return None
        return since

    def evaluate(self, now: float) -> BatteryDecision:
        """依當前樣本與時間算出狀態。應由節點的定時器週期呼叫。"""
        cfg = self.config
        arbitrated, per_source, used = self._arbitrate(now)

        if arbitrated is None:
            # 沒有任何新鮮來源：不知道電壓就不改變低壓判定。
            # 計時器歸零，避免把「盲區時間」算進持續條件而在資料回來的瞬間誤停。
            self._below_warn_since = None
            self._below_shutdown_since = None
            state = STATE_SHUTDOWN if self._stop_latched else STATE_UNKNOWN
            reason = ('停機已鎖存（電壓來源目前無資料）' if self._stop_latched
                      else '無可用電壓來源（/motor/voltage 與 /pico/voltage 皆逾時）')
            return BatteryDecision(
                state=state, voltage=None, source_voltages=per_source,
                sources_used=used, stop_latched=self._stop_latched, reason=reason)

        self._below_shutdown_since = self._update_timer(
            self._below_shutdown_since, arbitrated, cfg.shutdown_voltage, now)
        self._below_warn_since = self._update_timer(
            self._below_warn_since, arbitrated, cfg.warn_voltage, now)

        # 停機：滿足持續時間即鎖存，之後任何電壓都不再解除
        if (not self._stop_latched
                and self._below_shutdown_since is not None
                and (now - self._below_shutdown_since) >= cfg.shutdown_duration_sec):
            self._stop_latched = True

        # 警告：可解除（電壓回到 warn + 遲滯之上，計時器被 _update_timer 清掉）
        if (self._below_warn_since is not None
                and (now - self._below_warn_since) >= cfg.warn_duration_sec):
            self._warning_active = True
        elif self._below_warn_since is None:
            self._warning_active = False

        if self._stop_latched:
            state = STATE_SHUTDOWN
            reason = (f'電壓 {arbitrated:.2f}V 曾持續低於 {cfg.shutdown_voltage}V '
                      f'達 {cfg.shutdown_duration_sec:.0f}s，停機已鎖存')
        elif self._warning_active:
            state = STATE_WARNING
            reason = (f'電壓 {arbitrated:.2f}V 低於警告門檻 {cfg.warn_voltage}V，'
                      f'請儘快充電')
        else:
            state = STATE_OK
            reason = f'電壓 {arbitrated:.2f}V 正常'

        return BatteryDecision(
            state=state, voltage=arbitrated, source_voltages=per_source,
            sources_used=used, stop_latched=self._stop_latched, reason=reason)

    # ------------------------------------------------------------------
    # 測試輔助
    # ------------------------------------------------------------------

    def reset(self) -> None:
        """清除鎖存與計時器。**只給單元測試用**：真機解除停機的唯一方式是
        充電後重啟節點，否則鎖存就沒有意義了。"""
        self._below_warn_since = None
        self._below_shutdown_since = None
        self._warning_active = False
        self._stop_latched = False
