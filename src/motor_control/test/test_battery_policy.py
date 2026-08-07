"""battery_policy.py 的離線測試（不需要硬體，也不需要 rclpy）

時間全部由測試自己餵，不用 sleep，因此可以精確驗證「持續 N 秒」這類條件。

最高風險的兩條路徑，各自成節：
  1. **不該停卻停了**（加速 sag 誤觸發）→ 機器人在任務中途無預警停住。
  2. **該停卻沒停**（門檻抖動、單源消失、鎖存被解除）→ 電池過放報廢。
"""

import pytest

from motor_control.battery_policy import (
    BatteryPolicy,
    BatteryPolicyConfig,
    SOURCE_MOTOR,
    SOURCE_PICO,
    STATE_OK,
    STATE_SHUTDOWN,
    STATE_UNKNOWN,
    STATE_WARNING,
)


def make_policy(**overrides) -> BatteryPolicy:
    """預設門檻（22.4 / 21.7）的 policy，個別參數可覆寫。"""
    return BatteryPolicy(BatteryPolicyConfig(**overrides))


def feed(policy, voltage, start, duration, *, hz=10.0,
         sources=(SOURCE_MOTOR,), evaluate=True):
    """以 hz 的取樣率餵入固定電壓 duration 秒，回傳最後一次 evaluate 的結果。

    每餵一筆就 evaluate 一次（模擬節點的定時器），確保狀態機被真的推進，
    而不是靠最後一次呼叫一口氣算完。
    """
    step = 1.0 / hz
    decision = None
    t = start
    end = start + duration
    while t <= end + 1e-9:
        for source in sources:
            policy.submit(source, voltage, t)
        if evaluate:
            decision = policy.evaluate(t)
        t += step
    return decision, t - step


# --------------------------------------------------------------------------
# 門檻與持續時間
# --------------------------------------------------------------------------

def test_normal_voltage_stays_ok():
    """24V 穩定 → 一路 OK，不發停機。"""
    policy = make_policy()
    decision, _ = feed(policy, 24.0, 0.0, 10.0)
    assert decision.state == STATE_OK
    assert decision.stop_latched is False


def test_below_warn_threshold_needs_duration():
    """22.3V：未滿 warn_duration_sec 前不進警告，滿了才進。"""
    policy = make_policy()

    decision, t = feed(policy, 22.3, 0.0, 1.0)   # 1s < 2s
    assert decision.state == STATE_OK

    decision, _ = feed(policy, 22.3, t, 1.5)     # 累計 2.5s
    assert decision.state == STATE_WARNING
    assert decision.stop_latched is False


def test_below_shutdown_threshold_latches_after_duration():
    """21.6V 持續 → 滿 shutdown_duration_sec 後鎖存停機。"""
    policy = make_policy()

    decision, t = feed(policy, 21.6, 0.0, 2.0)   # 2s < 3s
    assert decision.state == STATE_WARNING       # 已低於警告門檻，但還沒到停機
    assert decision.stop_latched is False

    decision, _ = feed(policy, 21.6, t, 2.0)     # 累計 4s
    assert decision.state == STATE_SHUTDOWN
    assert decision.stop_latched is True


def test_shutdown_is_latched_against_voltage_rebound():
    """停機後電壓回彈到 23V 仍維持停機。

    負載一消失電壓就會回彈，那是假象不是復原；自動解除等於邊放電邊重啟。
    """
    policy = make_policy()
    _, t = feed(policy, 21.6, 0.0, 4.0)

    decision, _ = feed(policy, 23.0, t, 10.0)
    assert decision.state == STATE_SHUTDOWN
    assert decision.stop_latched is True


def test_warning_clears_only_above_hysteresis():
    """警告可解除，但必須回到 warn + 遲滯之上（22.4 + 0.3 = 22.7V）。"""
    policy = make_policy()
    _, t = feed(policy, 22.3, 0.0, 3.0)

    # 22.5V：高於門檻但仍在遲滯帶內 → 不解除
    decision, t = feed(policy, 22.5, t, 2.0)
    assert decision.state == STATE_WARNING

    # 22.8V：超過遲滯 → 解除
    decision, _ = feed(policy, 22.8, t, 2.0)
    assert decision.state == STATE_OK


# --------------------------------------------------------------------------
# 抗誤觸發（不該停卻停了）
# --------------------------------------------------------------------------

def test_acceleration_sag_does_not_trigger_shutdown():
    """23V 基準、0.5 秒瞬降 21.5V 後回復 → 不得停機、也不得進警告。

    這就是馬達起步時電池內阻造成的 sag，是本功能最容易誤觸發的情境。
    """
    policy = make_policy()
    _, t = feed(policy, 23.0, 0.0, 5.0)
    decision, t = feed(policy, 21.5, t, 0.5)     # sag
    assert decision.stop_latched is False

    decision, _ = feed(policy, 23.0, t, 5.0)     # 回復
    assert decision.state == STATE_OK
    assert decision.stop_latched is False


def test_repeated_sag_bursts_do_not_accumulate():
    """反覆 sag（走走停停）不得靠累加湊滿持續時間。"""
    policy = make_policy()
    t = 0.0
    for _ in range(6):
        _, t = feed(policy, 21.5, t, 0.5)        # 每次 sag 只有 0.5s
        _, t = feed(policy, 23.0, t, 1.5)        # 回到門檻 + 遲滯之上 → 計時歸零
    decision = policy.evaluate(t)
    assert decision.stop_latched is False


def test_threshold_chatter_still_shuts_down():
    """在停機門檻附近抖動（21.6 ↔ 21.8）仍必須停機。

    21.8V 落在遲滯帶 [21.7, 22.0) 內，不足以把計時歸零；
    若這裡歸零，電量耗盡時電壓正好在門檻附近抖動，保護會永遠不生效。
    """
    policy = make_policy()
    t = 0.0
    for _ in range(8):
        _, t = feed(policy, 21.6, t, 0.3)
        _, t = feed(policy, 21.8, t, 0.3)
    decision = policy.evaluate(t)
    assert decision.state == STATE_SHUTDOWN


def test_invalid_readings_are_rejected():
    """0.0 / NaN / 離譜值不得進入判定（驅動器解析異常會吐 0.0）。"""
    policy = make_policy()
    assert policy.submit(SOURCE_MOTOR, 0.0, 0.0) is False
    assert policy.submit(SOURCE_MOTOR, float('nan'), 0.0) is False
    assert policy.submit(SOURCE_MOTOR, float('inf'), 0.0) is False
    assert policy.submit(SOURCE_MOTOR, 999.0, 0.0) is False
    assert policy.rejected_count(SOURCE_MOTOR) == 4

    # 全是無效值 → 等同沒有來源，不能因此停機
    decision = policy.evaluate(5.0)
    assert decision.state == STATE_UNKNOWN
    assert decision.stop_latched is False


def test_zero_voltage_flood_does_not_shut_down():
    """驅動器持續吐 0.0 十秒也不得停機（無效值不是低電壓）。"""
    policy = make_policy()
    decision, _ = feed(policy, 0.0, 0.0, 10.0)
    assert decision.state == STATE_UNKNOWN
    assert decision.stop_latched is False


# --------------------------------------------------------------------------
# 雙源仲裁與降級
# --------------------------------------------------------------------------

def test_arbitration_takes_minimum_of_fresh_sources():
    """兩源都活著 → 取小值（保守）。"""
    policy = make_policy()
    t = 0.0
    for _ in range(30):                          # 2.9s > warn_duration_sec
        policy.submit(SOURCE_MOTOR, 23.5, t)
        policy.submit(SOURCE_PICO, 22.2, t)
        decision = policy.evaluate(t)
        t += 0.1
    assert decision.sources_used == (SOURCE_MOTOR, SOURCE_PICO)
    assert decision.voltage == pytest.approx(22.2, abs=1e-6)
    assert decision.state == STATE_WARNING       # 低的那一路說了算


def test_pico_silence_degrades_to_motor_only():
    """pico ok=0 不發布 → 逾時後降級單源，保護仍必須有效。"""
    policy = make_policy()

    # 兩源共存 1 秒，pico 讀值偏高
    _, t = feed(policy, 25.0, 0.0, 1.0, sources=(SOURCE_MOTOR, SOURCE_PICO))

    # pico 靜默，motor 掉到停機門檻以下
    decision, t = feed(policy, 21.5, t, 6.0, sources=(SOURCE_MOTOR,))
    assert decision.sources_used == (SOURCE_MOTOR,)
    assert decision.source_voltages[SOURCE_PICO] is None
    assert decision.state == STATE_SHUTDOWN


def test_motor_silence_degrades_to_pico_only():
    """反向：只剩 pico 也一樣要能停機。"""
    policy = make_policy()
    decision, _ = feed(policy, 21.5, 0.0, 6.0, sources=(SOURCE_PICO,))
    assert decision.sources_used == (SOURCE_PICO,)
    assert decision.state == STATE_SHUTDOWN


def test_all_sources_stale_reports_unknown_without_stopping():
    """兩源都消失 → UNKNOWN，且不得觸發停機（沒資料 ≠ 低電壓）。"""
    policy = make_policy()
    _, t = feed(policy, 24.0, 0.0, 2.0, sources=(SOURCE_MOTOR, SOURCE_PICO))

    decision = policy.evaluate(t + 10.0)
    assert decision.state == STATE_UNKNOWN
    assert decision.voltage is None
    assert decision.stop_latched is False


def test_latched_shutdown_survives_source_loss():
    """已鎖存後兩源全失聯，狀態仍是 SHUTDOWN、停機命令仍要發。"""
    policy = make_policy()
    _, t = feed(policy, 21.5, 0.0, 4.0)
    assert policy.evaluate(t).state == STATE_SHUTDOWN

    decision = policy.evaluate(t + 30.0)
    assert decision.state == STATE_SHUTDOWN
    assert decision.stop_latched is True


def test_blind_gap_does_not_count_toward_duration():
    """資料中斷期間不得算進「持續低於門檻」的時間。

    低壓 1 秒 → 失聯 10 秒 → 資料回來的當下不能立刻停機，
    否則一次序列埠斷線就會把還沒到門檻時間的電壓判成過放。
    """
    policy = make_policy()
    _, t = feed(policy, 21.5, 0.0, 1.0)

    t += 10.0                                    # 失聯，超過 source_timeout_sec
    assert policy.evaluate(t).state == STATE_UNKNOWN

    policy.submit(SOURCE_MOTOR, 21.5, t)
    decision = policy.evaluate(t)
    assert decision.stop_latched is False        # 重新計時，不是立刻停


def test_moving_average_smooths_single_outlier():
    """單一低值離群點被時間窗平均吃掉，不足以把仲裁值壓到門檻以下。"""
    policy = make_policy()
    _, t = feed(policy, 23.0, 0.0, 2.0)
    policy.submit(SOURCE_MOTOR, 18.0, t)         # 一筆離群
    decision = policy.evaluate(t)
    assert decision.voltage > 21.7
    assert decision.stop_latched is False


# --------------------------------------------------------------------------
# 設定驗證
# --------------------------------------------------------------------------

def test_config_rejects_inverted_thresholds():
    with pytest.raises(ValueError, match='shutdown_voltage'):
        BatteryPolicyConfig(warn_voltage=21.0, shutdown_voltage=22.0).validate()


def test_config_rejects_hysteresis_crossing_warn_threshold():
    with pytest.raises(ValueError, match='hysteresis'):
        BatteryPolicyConfig(hysteresis_voltage=2.0).validate()


def test_config_rejects_min_valid_above_shutdown():
    """否則真的放到停機門檻以下時，樣本全被當無效值丟掉，保護失效。"""
    with pytest.raises(ValueError, match='min_valid_voltage'):
        BatteryPolicyConfig(min_valid_voltage=22.0).validate()


def test_config_rejects_non_positive_durations():
    with pytest.raises(ValueError, match='shutdown_duration_sec'):
        BatteryPolicyConfig(shutdown_duration_sec=0.0).validate()
