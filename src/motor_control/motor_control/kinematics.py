"""
差動驅動運動學純模組 - 無 ROS 依賴

從 HSMotorController 與 MockMotorController 抽出「兩者本就等價」的
差動驅動運動學公式：twist <-> 輪速轉換、輪速 <-> 馬達 RPM 轉換、
RPM 量化 clamp、里程計積分。所有公式的運算順序與數值皆與抽取前
完全相同（浮點運算順序刻意保持一致，避免任何位元級差異）。

未合併的部分（刻意保留在各自檔案中，見對應模組的說明）：
- HSMotorController 依實際回饋 RPM (actual_rpm_a/b) 搭配 RPM_DEADZONE
  過濾雜訊後才轉換為輪速，這段「回饋濾波」邏輯是真機特有、mock 沒有
  對應行為，因此保留在 hs_motor_controller.py 內，只共用其後的
  motor_rpm_to_wheel_vel 轉換公式本身。
- MockMotorController 的 odom_theta 正規化 (wrap 到 [-pi, pi]) 是
  mock 特有行為，HSMotorController 原本沒有這段正規化，因此不納入
  共用的 integrate_odometry，避免改變真機行為。
"""

import math


class DifferentialDriveKinematics:
    """差動驅動運動學計算

    封裝 wheel_separation / wheel_radius / gear_ratio / min_rpm / max_rpm /
    zero_rpm_epsilon，提供 twist <-> 輪速 <-> 馬達 RPM 的純函式轉換。
    """

    def __init__(
        self,
        wheel_separation: float,
        wheel_radius: float,
        gear_ratio: float,
        min_rpm: float,
        max_rpm: float,
        zero_rpm_epsilon: float = 1.0,
    ):
        self.wheel_separation = wheel_separation
        self.wheel_radius = wheel_radius
        self.gear_ratio = gear_ratio
        self.min_rpm = min_rpm
        self.max_rpm = max_rpm
        self.zero_rpm_epsilon = zero_rpm_epsilon

    def twist_to_wheel_vel(self, linear_x: float, angular_z: float) -> tuple:
        """cmd_vel (linear_x, angular_z) -> (left_vel, right_vel) 差動驅動運動學

        與抽取前 HSMotorController.cmd_vel_callback /
        MockMotorController.cmd_vel_callback 的兩行公式完全相同。
        """
        left_vel = linear_x - (angular_z * self.wheel_separation / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_separation / 2.0)
        return left_vel, right_vel

    def wheel_vel_to_twist(self, left_vel: float, right_vel: float) -> tuple:
        """(left_vel, right_vel) -> (vx, vth) 差動驅動正向運動學

        與抽取前 HSMotorController.update_odometry 的
        vx = (vel_a + vel_b) / 2.0 、 vth = (vel_a - vel_b) / wheel_separation
        （vel_a=右輪, vel_b=左輪）以及 MockMotorController.cmd_vel_callback 的
        current_linear_x / current_angular_z 公式完全相同（僅換成具名參數，
        加法交換律不影響浮點結果）。
        """
        vx = (left_vel + right_vel) / 2.0
        vth = (right_vel - left_vel) / self.wheel_separation
        return vx, vth

    def wheel_vel_to_motor_rpm(self, wheel_vel: float) -> float:
        """輪速 (m/s) -> 馬達 RPM (取絕對值，方向另外處理)

        與抽取前 HSMotorController.set_motor_speeds 的
        wheel_rpm = abs(vel) / (2*pi*wheel_radius) * 60.0；motor_rpm = wheel_rpm * gear_ratio
        以及 MockMotorController._quantize_wheel_vel 的
        motor_rpm = abs(wheel_vel) / (2*pi*wheel_radius) * 60.0 * gear_ratio
        運算順序完全相同（皆為由左至右依序 除、乘60、乘 gear_ratio）。
        """
        wheel_rpm = abs(wheel_vel) / (2 * math.pi * self.wheel_radius) * 60.0
        return wheel_rpm * self.gear_ratio

    def motor_rpm_to_wheel_vel(self, motor_rpm: float) -> float:
        """馬達 RPM -> 輪速 (m/s)，wheel_vel_to_motor_rpm 的反向轉換

        與抽取前 HSMotorController.update_odometry 的
        wheel_rpm_a = motor_rpm_a / gear_ratio；vel_a = (wheel_rpm_a/60.0)*(2*pi*wheel_radius)
        以及 MockMotorController._quantize_wheel_vel 的
        quantized = (motor_rpm/gear_ratio/60.0) * (2*pi*wheel_radius)
        運算順序完全相同。
        """
        wheel_rpm = motor_rpm / self.gear_ratio
        return (wheel_rpm / 60.0) * (2 * math.pi * self.wheel_radius)

    def quantize_motor_rpm(self, motor_rpm: float) -> float:
        """量化馬達 RPM 到驅動器有效範圍 [min_rpm, max_rpm]

        - 低於 zero_rpm_epsilon 視為零命令 -> 0.0
        - 非零但低於 min_rpm -> clamp 到 min_rpm（避免低速死區導致不動）
        - 其餘 clamp 到 max_rpm

        與抽取前 HSMotorController._quantize_rpm（回傳前另外 int() 轉型，
        呼叫端維持 int()）以及 MockMotorController._quantize_wheel_vel
        內對應的 clamp 邏輯完全相同。
        """
        if motor_rpm < self.zero_rpm_epsilon:
            return 0.0
        return max(min(motor_rpm, self.max_rpm), self.min_rpm)

    @staticmethod
    def integrate_odometry(
        x: float, y: float, theta: float, vx: float, vth: float, dt: float
    ) -> tuple:
        """單軌模型 (unicycle model) 里程計積分，回傳 (new_x, new_y, new_theta)

        與抽取前 HSMotorController.update_odometry 以及
        MockMotorController.update_odometry 的積分公式完全相同：
            x += vx * cos(theta) * dt
            y += vx * sin(theta) * dt
            theta += vth * dt
        （不含角度正規化；正規化行為由呼叫端自行決定是否套用，
        見本模組頂端說明。）
        """
        new_x = x + vx * math.cos(theta) * dt
        new_y = y + vx * math.sin(theta) * dt
        new_theta = theta + vth * dt
        return new_x, new_y, new_theta
