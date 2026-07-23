"""
kinematics.py 對照測試（不需要 ROS/硬體）

驗證方式：在本測試檔內以「抽取前」HSMotorController /
MockMotorController 原始程式碼逐字複製一份 reference 公式，與抽取後的
motor_control.kinematics.DifferentialDriveKinematics 互相比對，確保運算
順序與數值完全一致（golden sample 對照，而非重新推導公式）。

用法::

    /usr/bin/python3 -m pytest tests/test_kinematics.py -v
"""

import math
import os
import random
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, 'src', 'motor_control', 'motor_control'))

from motor_control.kinematics import DifferentialDriveKinematics  # noqa: E402


# ---------------------------------------------------------------------------
# Reference 公式：逐字複製自抽取前 HSMotorController / MockMotorController
# ---------------------------------------------------------------------------

def ref_twist_to_wheel_vel(linear_x, angular_z, wheel_separation):
    """HSMotorController.cmd_vel_callback / MockMotorController.cmd_vel_callback"""
    left_vel = linear_x - (angular_z * wheel_separation / 2.0)
    right_vel = linear_x + (angular_z * wheel_separation / 2.0)
    return left_vel, right_vel


def ref_hs_set_motor_speeds_rpm(left_vel, right_vel, wheel_radius, gear_ratio):
    """HSMotorController.set_motor_speeds 的輪速 -> 馬達 RPM 轉換段"""
    left_wheel_rpm = abs(left_vel) / (2 * math.pi * wheel_radius) * 60.0
    right_wheel_rpm = abs(right_vel) / (2 * math.pi * wheel_radius) * 60.0
    left_motor_rpm = left_wheel_rpm * gear_ratio
    right_motor_rpm = right_wheel_rpm * gear_ratio
    return left_motor_rpm, right_motor_rpm


def ref_quantize_rpm(motor_rpm, min_rpm, max_rpm, zero_epsilon):
    """HSMotorController._quantize_rpm"""
    if motor_rpm < zero_epsilon:
        return 0
    return int(max(min(motor_rpm, max_rpm), min_rpm))


def ref_mock_quantize_wheel_vel(wheel_vel, wheel_radius, gear_ratio, min_rpm, max_rpm,
                                 zero_epsilon):
    """MockMotorController._quantize_wheel_vel"""
    motor_rpm = abs(wheel_vel) / (2 * math.pi * wheel_radius) * 60.0 * gear_ratio
    if motor_rpm < zero_epsilon:
        return 0.0
    motor_rpm = max(min(motor_rpm, max_rpm), min_rpm)
    quantized = (motor_rpm / gear_ratio / 60.0) * (2 * math.pi * wheel_radius)
    return math.copysign(quantized, wheel_vel)


def ref_hs_update_odometry_twist(motor_rpm_a, motor_rpm_b, logical_dir_a, logical_dir_b,
                                  gear_ratio, wheel_radius, wheel_separation):
    """HSMotorController.update_odometry 的 RPM -> vx, vth 段"""
    wheel_rpm_a = motor_rpm_a / gear_ratio
    wheel_rpm_b = motor_rpm_b / gear_ratio
    vel_a = (wheel_rpm_a / 60.0) * (2 * math.pi * wheel_radius)
    vel_b = (wheel_rpm_b / 60.0) * (2 * math.pi * wheel_radius)
    if motor_rpm_a > 0 and logical_dir_a == 1:
        vel_a = -vel_a
    if motor_rpm_b > 0 and logical_dir_b == 1:
        vel_b = -vel_b
    vx = (vel_a + vel_b) / 2.0
    vth = (vel_a - vel_b) / wheel_separation
    return vx, vth


def ref_mock_current_twist(left_vel, right_vel, wheel_separation):
    """MockMotorController.cmd_vel_callback 的換算回機器人速度段"""
    current_linear_x = (left_vel + right_vel) / 2.0
    current_angular_z = (right_vel - left_vel) / wheel_separation
    return current_linear_x, current_angular_z


def ref_integrate(x, y, theta, vx, vth, dt):
    """HSMotorController.update_odometry / MockMotorController.update_odometry 的積分段"""
    new_x = x + vx * math.cos(theta) * dt
    new_y = y + vx * math.sin(theta) * dt
    new_theta = theta + vth * dt
    return new_x, new_y, new_theta


# ---------------------------------------------------------------------------
# 測試用參數（與 hs_motor_config.yaml 預設值一致）
# ---------------------------------------------------------------------------

WHEEL_SEPARATION = 0.27
WHEEL_RADIUS = 0.065
GEAR_RATIO = 20.0
MIN_RPM = 100.0
MAX_RPM = 3000.0
ZERO_RPM_EPSILON = 1.0


def make_k():
    return DifferentialDriveKinematics(
        wheel_separation=WHEEL_SEPARATION,
        wheel_radius=WHEEL_RADIUS,
        gear_ratio=GEAR_RATIO,
        min_rpm=MIN_RPM,
        max_rpm=MAX_RPM,
        zero_rpm_epsilon=ZERO_RPM_EPSILON,
    )


# ---------------------------------------------------------------------------
# twist_to_wheel_vel
# ---------------------------------------------------------------------------

def test_twist_to_wheel_vel_matches_reference():
    k = make_k()
    rng = random.Random(1)
    for _ in range(200):
        linear_x = rng.uniform(-0.05, 0.05)
        angular_z = rng.uniform(-0.4, 0.4)
        expected = ref_twist_to_wheel_vel(linear_x, angular_z, WHEEL_SEPARATION)
        actual = k.twist_to_wheel_vel(linear_x, angular_z)
        assert actual == expected


def test_twist_to_wheel_vel_known_values():
    k = make_k()
    # 純直行
    left, right = k.twist_to_wheel_vel(0.05, 0.0)
    assert left == 0.05
    assert right == 0.05
    # 純旋轉
    left, right = k.twist_to_wheel_vel(0.0, 0.4)
    assert left == -0.4 * WHEEL_SEPARATION / 2.0
    assert right == 0.4 * WHEEL_SEPARATION / 2.0


# ---------------------------------------------------------------------------
# wheel_vel_to_motor_rpm / motor_rpm_to_wheel_vel（真機路徑）
# ---------------------------------------------------------------------------

def test_wheel_vel_to_motor_rpm_matches_hs_reference():
    k = make_k()
    rng = random.Random(2)
    for _ in range(200):
        left_vel = rng.uniform(-0.05, 0.05)
        right_vel = rng.uniform(-0.05, 0.05)
        expected_left, expected_right = ref_hs_set_motor_speeds_rpm(
            left_vel, right_vel, WHEEL_RADIUS, GEAR_RATIO)
        assert k.wheel_vel_to_motor_rpm(left_vel) == expected_left
        assert k.wheel_vel_to_motor_rpm(right_vel) == expected_right


def test_quantize_rpm_matches_hs_reference():
    k = make_k()
    rng = random.Random(3)
    for _ in range(500):
        motor_rpm = rng.uniform(0, 5000)
        expected = ref_quantize_rpm(motor_rpm, MIN_RPM, MAX_RPM, ZERO_RPM_EPSILON)
        actual = int(k.quantize_motor_rpm(motor_rpm))
        assert actual == expected

    # 邊界值
    assert int(k.quantize_motor_rpm(0.0)) == 0
    assert int(k.quantize_motor_rpm(0.999)) == 0          # < epsilon
    assert int(k.quantize_motor_rpm(1.0)) == int(MIN_RPM)  # 非零但 < min_rpm -> clamp
    assert int(k.quantize_motor_rpm(MIN_RPM)) == int(MIN_RPM)
    assert int(k.quantize_motor_rpm(MAX_RPM)) == int(MAX_RPM)
    assert int(k.quantize_motor_rpm(MAX_RPM + 1000)) == int(MAX_RPM)


def test_motor_rpm_to_wheel_vel_matches_hs_odometry_reference():
    """驗證 motor_rpm_to_wheel_vel 與 HSMotorController.update_odometry
    的 RPM -> 輪速轉換公式完全相同（含死區/方向套用後的完整 twist）"""
    k = make_k()
    rng = random.Random(4)
    for _ in range(200):
        motor_rpm_a = rng.uniform(0, 3000)
        motor_rpm_b = rng.uniform(0, 3000)
        logical_dir_a = rng.randint(0, 1)
        logical_dir_b = rng.randint(0, 1)

        expected_vx, expected_vth = ref_hs_update_odometry_twist(
            motor_rpm_a, motor_rpm_b, logical_dir_a, logical_dir_b,
            GEAR_RATIO, WHEEL_RADIUS, WHEEL_SEPARATION)

        vel_a = k.motor_rpm_to_wheel_vel(motor_rpm_a)
        vel_b = k.motor_rpm_to_wheel_vel(motor_rpm_b)
        if motor_rpm_a > 0 and logical_dir_a == 1:
            vel_a = -vel_a
        if motor_rpm_b > 0 and logical_dir_b == 1:
            vel_b = -vel_b
        vx, vth = k.wheel_vel_to_twist(vel_b, vel_a)

        assert vx == expected_vx
        assert vth == expected_vth


# ---------------------------------------------------------------------------
# quantize（mock 路徑）：wheel_vel -> motor_rpm -> quantize -> wheel_vel
# ---------------------------------------------------------------------------

def test_mock_quantize_path_matches_reference():
    k = make_k()
    rng = random.Random(5)
    for _ in range(500):
        wheel_vel = rng.uniform(-0.1, 0.1)
        expected = ref_mock_quantize_wheel_vel(
            wheel_vel, WHEEL_RADIUS, GEAR_RATIO, MIN_RPM, MAX_RPM, ZERO_RPM_EPSILON)

        motor_rpm = k.wheel_vel_to_motor_rpm(wheel_vel)
        quantized_rpm = k.quantize_motor_rpm(motor_rpm)
        if quantized_rpm == 0.0:
            actual = 0.0
        else:
            actual = math.copysign(k.motor_rpm_to_wheel_vel(quantized_rpm), wheel_vel)

        assert actual == expected


def test_mock_quantize_path_zero_and_clamp_boundaries():
    k = make_k()

    # 完全靜止
    assert ref_mock_quantize_wheel_vel(0.0, WHEEL_RADIUS, GEAR_RATIO, MIN_RPM, MAX_RPM,
                                        ZERO_RPM_EPSILON) == 0.0

    # 極小速度 -> 視為零命令
    tiny = 1e-6
    expected = ref_mock_quantize_wheel_vel(tiny, WHEEL_RADIUS, GEAR_RATIO, MIN_RPM, MAX_RPM,
                                            ZERO_RPM_EPSILON)
    motor_rpm = k.wheel_vel_to_motor_rpm(tiny)
    quantized_rpm = k.quantize_motor_rpm(motor_rpm)
    actual = 0.0 if quantized_rpm == 0.0 else math.copysign(
        k.motor_rpm_to_wheel_vel(quantized_rpm), tiny)
    assert actual == expected == 0.0

    # 方向保留（負值）
    neg_vel = -0.03
    expected_neg = ref_mock_quantize_wheel_vel(
        neg_vel, WHEEL_RADIUS, GEAR_RATIO, MIN_RPM, MAX_RPM, ZERO_RPM_EPSILON)
    assert expected_neg < 0


# ---------------------------------------------------------------------------
# wheel_vel_to_twist（mock cmd_vel 路徑）
# ---------------------------------------------------------------------------

def test_wheel_vel_to_twist_matches_mock_reference():
    k = make_k()
    rng = random.Random(6)
    for _ in range(200):
        left_vel = rng.uniform(-0.1, 0.1)
        right_vel = rng.uniform(-0.1, 0.1)
        expected = ref_mock_current_twist(left_vel, right_vel, WHEEL_SEPARATION)
        actual = k.wheel_vel_to_twist(left_vel, right_vel)
        assert actual == expected


# ---------------------------------------------------------------------------
# integrate_odometry
# ---------------------------------------------------------------------------

def test_integrate_odometry_matches_reference():
    rng = random.Random(7)
    for _ in range(200):
        x = rng.uniform(-5, 5)
        y = rng.uniform(-5, 5)
        theta = rng.uniform(-math.pi, math.pi)
        vx = rng.uniform(-0.05, 0.05)
        vth = rng.uniform(-0.4, 0.4)
        dt = rng.uniform(0.001, 0.1)

        expected = ref_integrate(x, y, theta, vx, vth, dt)
        actual = DifferentialDriveKinematics.integrate_odometry(x, y, theta, vx, vth, dt)
        assert actual == expected


def test_integrate_odometry_pure_translation():
    new_x, new_y, new_theta = DifferentialDriveKinematics.integrate_odometry(
        0.0, 0.0, 0.0, vx=1.0, vth=0.0, dt=1.0)
    assert new_x == 1.0
    assert new_y == 0.0
    assert new_theta == 0.0


def test_integrate_odometry_no_normalization():
    """與抽取前 HSMotorController 行為一致：不做角度正規化，可累積超過 pi"""
    _, _, new_theta = DifferentialDriveKinematics.integrate_odometry(
        0.0, 0.0, math.pi - 0.01, vx=0.0, vth=1.0, dt=1.0)
    assert new_theta > math.pi
