"""Jetson Orin NX CPU 親和性 (affinity) 共用工具 - 無 ROS 依賴

供 motor_control/launch/bringup.launch.py 與
nav2/launch/autonomous_navigation.launch.py 共用，避免
parse_cpu_list / get_online_cpus / resolve / prefix 邏輯各寫一份而產生
行為分岔。ROS2 workspace 會把每個套件安裝到共同的 site-packages 並加入
PYTHONPATH（source install/setup.bash 時），因此其他套件的 launch 檔可
直接 `from motor_control.cpu_affinity import ...`（nav2 的
autonomous_navigation.launch.py 本身也已用同樣機制 `from nav2.keepout
import regenerate_keepout` 匯入自家套件模組）。

純模組，不得 import rclpy。

8 核心分配策略（Jetson Orin NX）：
    核心 0-1: 馬達控制（實時性最高）
    核心 2-3: LiDAR/IMU 感測器處理
    核心 4-5: EKF/AMCL 定位
    核心 6-7: Web 服務/API（優先級最低）
實際綁定範圍會與 /sys/devices/system/cpu/online 取交集，因為 nvpmodel
低功耗模式（15W 只保留 0-3）會讓部分核心離線，詳見 resolve_cpu_affinity()。
"""

CPU_AFFINITY = {
    'motor': '0-1',
    'sensor': '2-3',
    'localization': '4-5',
    'web': '6-7',
}


def parse_cpu_list(spec: str) -> list:
    """解析 '0-3'、'0,2-4' 這類 CPU 清單字串"""
    cpus = set()
    for part in spec.split(','):
        part = part.strip()
        if not part:
            continue
        if '-' in part:
            start, end = part.split('-', 1)
            cpus.update(range(int(start), int(end) + 1))
        else:
            cpus.add(int(part))
    return sorted(cpus)


def get_online_cpus() -> list:
    """讀取目前線上的 CPU 核心；讀取失敗回傳空清單"""
    try:
        with open('/sys/devices/system/cpu/online') as f:
            return parse_cpu_list(f.read().strip())
    except (OSError, ValueError):
        return []


def resolve_cpu_affinity(affinity_map: dict = None) -> tuple:
    """將 affinity_map（預設 CPU_AFFINITY）對照到實際線上的核心，
    回傳 (對照表, 警告訊息清單)。

    nvpmodel 的低功耗模式（例如 15W 只保留核心 0-3）會讓部分核心離線。
    taskset 綁到離線核心會立即失敗，配合 respawn 會讓節點陷入無限重啟，
    因此離線核心必須先剔除；整組都離線時該類節點就不綁定。
    """
    if affinity_map is None:
        affinity_map = CPU_AFFINITY

    online = get_online_cpus()
    if not online:
        return {}, ['[cpu_affinity] 無法讀取線上 CPU 清單，已停用 CPU 親和性綁定']

    resolved = {}
    warnings = []
    online_str = ','.join(str(c) for c in online)
    for name, spec in affinity_map.items():
        requested = parse_cpu_list(spec)
        usable = [c for c in requested if c in online]
        if not usable:
            warnings.append(
                f'[cpu_affinity] {name} 指定核心 {spec} 全部離線（線上核心：{online_str}），'
                f'該類節點改為不綁定 CPU'
            )
            continue
        if len(usable) != len(requested):
            warnings.append(
                f'[cpu_affinity] {name} 指定核心 {spec} 僅 {",".join(str(c) for c in usable)} '
                f'線上，已縮減綁定範圍'
            )
        resolved[name] = ','.join(str(c) for c in usable)
    return resolved, warnings


def resolve_affinity_spec(spec: str, online: list = None) -> list:
    """將單一 CPU spec 字串（如 '4-5'）與線上核心取交集，回傳可用核心 (int) 清單。

    給只需要單一類別（非整份 CPU_AFFINITY map）的呼叫端使用，例如
    nav2 launch 只綁定 localization 這一類節點。
    """
    if online is None:
        online = get_online_cpus()
    requested = parse_cpu_list(spec)
    return [c for c in requested if c in online]


def get_cpu_prefix_list(cpus) -> list:
    """由已解析的 CPU 清單字串（如 '0,1'）取得 taskset prefix，list 形式，
    用於 ExecuteProcess 的 cmd 參數。cpus 為 None/空字串時回傳 []。"""
    return ['taskset', '-c', cpus] if cpus else []


def get_cpu_prefix(cpus) -> str:
    """由已解析的 CPU 清單字串取得 taskset prefix，str 形式，用於 Node 的
    prefix 參數。由 get_cpu_prefix_list 組出，兩者行為保證一致（L4）。"""
    prefix_list = get_cpu_prefix_list(cpus)
    return ' '.join(prefix_list) if prefix_list else ''
