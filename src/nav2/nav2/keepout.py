"""Nav2 keepout mask 產生器（Winstec API 虛擬牆 → costmap filter mask）。

Winstec Robot API v1.1 §8「儲存契約」定義虛擬牆為**線段**，
座標為整數公分，可歸屬多個 group，只有 ``is_enable: true`` 的 group 生效。

本模組把這些線段轉成 Nav2 ``KeepoutFilter`` 使用的 mask 影像：

* 輸入（皆位於 ``map_dir``）::

      <map>.yaml               # 地圖 metadata（resolution / origin）
      <map>.pgm                # 地圖本體（取尺寸）
      <map>.virtual_walls.json # [{id, map, name, start_position, end_position}]
      <map>.groups.json        # [{id, map, name, is_enable, virtual_wall_ids: []}]

* 輸出::

      <map>.keepout.pgm        # 自由區 0、禁行區 254
      <map>.keepout.yaml       # mode: scale

.. note::
   mask 的像素語意刻意與規格一致（**0 = 自由、254 = 禁行**），
   這與一般地圖 PGM（黑色 0 = 障礙）相反，因此 ``keepout.yaml`` 必須
   帶 ``negate: 1``。搭配 ``free_thresh: 0.0`` / ``occupied_thresh: 1.0``
   與 ``mode: scale``，map_server 會把像素值線性映射成 0..100 的 occupancy：
   像素 0 → 0（自由）、像素 254 → 100（禁行，KeepoutFilter 視為 LETHAL）。

對外 API（robot_api_server 會 import）::

    regenerate_keepout(map_name: str, map_dir: str) -> str | None
    reload_keepout_mask(mask_yaml_path: str | None = None, ...) -> bool
"""

from __future__ import annotations

import json
import logging
import math
import os
import re
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np
import yaml

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# 常數
# ---------------------------------------------------------------------------

#: mask 中代表「可通行」的像素值
FREE_VALUE = 0
#: mask 中代表「禁止進入」的像素值
KEEPOUT_VALUE = 254
#: 線寬計算用的機器人半徑（footprint 0.5x0.5 的內切半徑，單位公尺）
ROBOT_RADIUS_M = 0.25
#: API 整數公分 → ROS 公尺
COORD_SCALE = 100.0

_PGM_MAGIC = b'P5'


# ---------------------------------------------------------------------------
# 檔案輔助
# ---------------------------------------------------------------------------

def keepout_yaml_path(map_name: str, map_dir: str) -> str:
    """回傳 ``<map_dir>/<map_name>.keepout.yaml`` 的絕對路徑（不保證存在）。"""
    return os.path.join(os.path.abspath(map_dir), f'{map_name}.keepout.yaml')


def _load_json(path: str, default):
    if not os.path.isfile(path):
        return default
    try:
        with open(path, 'r', encoding='utf-8') as fp:
            return json.load(fp)
    except (OSError, ValueError) as exc:
        logger.warning('讀取 %s 失敗：%s', path, exc)
        return default


def _read_pgm_size(path: str) -> Tuple[int, int]:
    """解析 binary PGM (P5) 標頭，回傳 ``(width, height)``。"""
    with open(path, 'rb') as fp:
        data = fp.read(4096)

    if not data.startswith(_PGM_MAGIC):
        raise ValueError(f'{path} 不是 binary PGM (P5)')

    # 逐一取出 token，略過 # 註解
    tokens: List[bytes] = []
    idx = len(_PGM_MAGIC)
    while len(tokens) < 3 and idx < len(data):
        ch = data[idx:idx + 1]
        if ch == b'#':
            while idx < len(data) and data[idx:idx + 1] not in (b'\n', b'\r'):
                idx += 1
        elif ch.isspace():
            idx += 1
        else:
            match = re.match(rb'\S+', data[idx:])
            if not match:
                break
            tokens.append(match.group(0))
            idx += match.end()

    if len(tokens) < 3:
        raise ValueError(f'{path} PGM 標頭不完整')

    return int(tokens[0]), int(tokens[1])


def _write_pgm(path: str, mask: np.ndarray) -> None:
    """以 binary PGM (P5, maxval 255) 輸出 mask。"""
    height, width = mask.shape
    header = (
        f'P5\n'
        f'# Nav2 keepout mask generated from Winstec virtual walls\n'
        f'{width} {height}\n255\n'
    ).encode('ascii')
    tmp_path = f'{path}.tmp'
    with open(tmp_path, 'wb') as fp:
        fp.write(header)
        fp.write(np.ascontiguousarray(mask, dtype=np.uint8).tobytes())
    os.replace(tmp_path, path)


# ---------------------------------------------------------------------------
# 幾何
# ---------------------------------------------------------------------------

def bresenham(x0: int, y0: int, x1: int, y1: int) -> List[Tuple[int, int]]:
    """整數座標 Bresenham 直線，回傳含頭尾的像素列表。"""
    points: List[Tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    x, y = x0, y0
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        err2 = 2 * err
        if err2 >= dy:
            err += dy
            x += sx
        if err2 <= dx:
            err += dx
            y += sy
    return points


def _position_xy(position) -> Optional[Tuple[int, int]]:
    """從 Position Object（dict 或 [x, y]）取出整數公分座標。"""
    if isinstance(position, dict):
        if 'x' not in position or 'y' not in position:
            return None
        try:
            return int(round(float(position['x']))), int(round(float(position['y'])))
        except (TypeError, ValueError):
            return None
    if isinstance(position, (list, tuple)) and len(position) >= 2:
        try:
            return int(round(float(position[0]))), int(round(float(position[1])))
        except (TypeError, ValueError):
            return None
    return None


def world_to_pixel(
    x_m: float,
    y_m: float,
    origin: Sequence[float],
    resolution: float,
    height: int,
) -> Tuple[int, int]:
    """公尺世界座標 → PGM 像素 ``(col, row)``。

    PGM 的第一列在影像上方，而地圖 origin 位於左下角，所以 row 要上下翻轉。
    """
    col = int(math.floor((x_m - float(origin[0])) / resolution))
    row_from_bottom = int(math.floor((y_m - float(origin[1])) / resolution))
    row = height - 1 - row_from_bottom
    return col, row


def _stamp(mask: np.ndarray, col: int, row: int, line_width: int) -> None:
    """以 ``line_width`` 見方的方形筆刷在 (col, row) 落點。"""
    height, width = mask.shape
    half = line_width // 2
    c0 = col - half
    r0 = row - half
    c1 = c0 + line_width
    r1 = r0 + line_width
    c0 = max(0, c0)
    r0 = max(0, r0)
    c1 = min(width, c1)
    r1 = min(height, r1)
    if c0 < c1 and r0 < r1:
        mask[r0:r1, c0:c1] = KEEPOUT_VALUE


# ---------------------------------------------------------------------------
# 主要流程
# ---------------------------------------------------------------------------

def collect_enabled_wall_ids(groups: Iterable[dict]) -> set:
    """收集所有 ``is_enable: true`` 的 group 內的 virtual_wall_ids。"""
    enabled = set()
    for group in groups or []:
        if not isinstance(group, dict):
            continue
        if not bool(group.get('is_enable', False)):
            continue
        for wall_id in group.get('virtual_wall_ids') or []:
            if wall_id:
                enabled.add(wall_id)
    return enabled


def regenerate_keepout(map_name: str, map_dir: str) -> Optional[str]:
    """依虛擬牆與 group 狀態重新產生 keepout mask。

    :param map_name: 地圖名稱（不含副檔名），例如 ``"01"``
    :param map_dir: 地圖目錄，例如 ``~/base_dev/map``
    :returns: 產生的 ``<map>.keepout.yaml`` 絕對路徑；地圖不存在或失敗時回傳 ``None``

    即使沒有任何 enabled 的虛擬牆，也會輸出一張全 0 的 mask，
    讓 costmap filter 永遠有東西可載。
    """
    map_dir = os.path.abspath(os.path.expanduser(map_dir))
    yaml_path = os.path.join(map_dir, f'{map_name}.yaml')

    if not os.path.isfile(yaml_path):
        logger.warning('找不到地圖 metadata：%s', yaml_path)
        return None

    try:
        with open(yaml_path, 'r', encoding='utf-8') as fp:
            meta = yaml.safe_load(fp) or {}
    except (OSError, yaml.YAMLError) as exc:
        logger.warning('解析 %s 失敗：%s', yaml_path, exc)
        return None

    try:
        resolution = float(meta['resolution'])
        origin = [float(v) for v in meta['origin']]
    except (KeyError, TypeError, ValueError) as exc:
        logger.warning('%s 缺少 resolution/origin：%s', yaml_path, exc)
        return None

    if resolution <= 0.0:
        logger.warning('%s resolution 非法：%s', yaml_path, resolution)
        return None
    while len(origin) < 3:
        origin.append(0.0)

    # 地圖影像可由 yaml 的 image 欄位指定（相對於 yaml 所在目錄）
    image_name = meta.get('image') or f'{map_name}.pgm'
    image_path = image_name if os.path.isabs(image_name) else os.path.join(map_dir, image_name)
    if not os.path.isfile(image_path):
        logger.warning('找不到地圖影像：%s', image_path)
        return None

    try:
        width, height = _read_pgm_size(image_path)
    except (OSError, ValueError) as exc:
        logger.warning('讀取 %s 尺寸失敗：%s', image_path, exc)
        return None

    mask = np.full((height, width), FREE_VALUE, dtype=np.uint8)

    groups = _load_json(os.path.join(map_dir, f'{map_name}.groups.json'), [])
    walls = _load_json(os.path.join(map_dir, f'{map_name}.virtual_walls.json'), [])
    enabled_ids = collect_enabled_wall_ids(groups if isinstance(groups, list) else [])

    line_width = max(1, math.ceil(ROBOT_RADIUS_M / resolution))
    drawn = 0

    if enabled_ids and isinstance(walls, list):
        for wall in walls:
            if not isinstance(wall, dict) or wall.get('id') not in enabled_ids:
                continue
            start = _position_xy(wall.get('start_position'))
            end = _position_xy(wall.get('end_position'))
            if start is None or end is None:
                logger.warning('虛擬牆 %s 座標不合法，略過', wall.get('id'))
                continue

            c0, r0 = world_to_pixel(
                start[0] / COORD_SCALE, start[1] / COORD_SCALE, origin, resolution, height)
            c1, r1 = world_to_pixel(
                end[0] / COORD_SCALE, end[1] / COORD_SCALE, origin, resolution, height)

            for col, row in bresenham(c0, r0, c1, r1):
                _stamp(mask, col, row, line_width)
            drawn += 1

    pgm_name = f'{map_name}.keepout.pgm'
    pgm_path = os.path.join(map_dir, pgm_name)
    out_yaml_path = keepout_yaml_path(map_name, map_dir)

    try:
        _write_pgm(pgm_path, mask)
        mask_meta = {
            'image': pgm_name,
            'mode': 'scale',
            'resolution': resolution,
            'origin': [origin[0], origin[1], origin[2]],
            # mask 語意為「0 = 自由、254 = 禁行」，與一般地圖相反，故 negate: 1
            'negate': 1,
            'occupied_thresh': 1.0,
            'free_thresh': 0.0,
        }
        tmp_yaml = f'{out_yaml_path}.tmp'
        with open(tmp_yaml, 'w', encoding='utf-8') as fp:
            yaml.safe_dump(mask_meta, fp, default_flow_style=False, sort_keys=False)
        os.replace(tmp_yaml, out_yaml_path)
    except OSError as exc:
        logger.warning('寫入 keepout mask 失敗：%s', exc)
        return None

    logger.info(
        'keepout mask 已更新：%s（%dx%d，線寬 %d px，畫入 %d 面牆）',
        out_yaml_path, width, height, line_width, drawn)
    return out_yaml_path


# ---------------------------------------------------------------------------
# 重載
# ---------------------------------------------------------------------------

def reload_keepout_mask(
    mask_yaml_path: Optional[str] = None,
    map_name: Optional[str] = None,
    map_dir: Optional[str] = None,
    node_name: str = 'filter_mask_server',
    timeout_sec: float = 2.0,
) -> bool:
    """呼叫 ``/<node_name>/load_map`` 讓 mask 更新後生效。

    :param mask_yaml_path: keepout yaml 路徑；未指定時由 ``map_name``/``map_dir`` 推導
    :param node_name: 發布 mask 的 map_server 節點名稱
    :param timeout_sec: 等待 service 與回應的秒數
    :returns: 是否成功觸發重載

    filter 節點未執行時（例如仍在建圖模式）**不會拋例外**，只回傳 ``False``。
    """
    if mask_yaml_path is None:
        if not map_name or not map_dir:
            logger.warning('reload_keepout_mask 需要 mask_yaml_path 或 map_name+map_dir')
            return False
        mask_yaml_path = keepout_yaml_path(map_name, map_dir)

    mask_yaml_path = os.path.abspath(os.path.expanduser(mask_yaml_path))
    if not os.path.isfile(mask_yaml_path):
        logger.warning('keepout mask 不存在，略過重載：%s', mask_yaml_path)
        return False

    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from nav2_msgs.srv import LoadMap
    except ImportError as exc:  # ROS 環境未 source
        logger.info('rclpy/nav2_msgs 不可用，略過 keepout 重載：%s', exc)
        return False

    context = None
    node = None
    try:
        # 使用獨立 context，避免干擾呼叫端既有的 rclpy 狀態
        context = rclpy.Context()
        rclpy.init(context=context)
        node = rclpy.create_node('keepout_reload_client', context=context)
        client = node.create_client(LoadMap, f'/{node_name}/load_map')

        if not client.wait_for_service(timeout_sec=timeout_sec):
            logger.info('%s/load_map 未就緒（filter 未執行？），略過重載', node_name)
            return False

        request = LoadMap.Request()
        request.map_url = mask_yaml_path
        future = client.call_async(request)

        executor = SingleThreadedExecutor(context=context)
        rclpy.spin_until_future_complete(
            node, future, executor=executor, timeout_sec=timeout_sec)

        if not future.done():
            logger.warning('%s/load_map 逾時', node_name)
            return False

        response = future.result()
        ok = response is not None and response.result == LoadMap.Response.RESULT_SUCCESS
        if not ok:
            logger.warning('%s/load_map 失敗，result=%s', node_name,
                           getattr(response, 'result', None))
        return ok
    except Exception as exc:  # noqa: BLE001 - 重載失敗絕不能拖垮呼叫端
        logger.warning('keepout 重載發生例外：%s', exc)
        return False
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if context is not None and context.ok():
                rclpy.shutdown(context=context)
        except Exception:  # noqa: BLE001
            pass


def regenerate_and_reload(
    map_name: str,
    map_dir: str,
    node_name: str = 'filter_mask_server',
    timeout_sec: float = 2.0,
) -> Optional[str]:
    """產生 mask 後嘗試重載；重載失敗不影響回傳值。"""
    path = regenerate_keepout(map_name, map_dir)
    if path:
        reload_keepout_mask(mask_yaml_path=path, node_name=node_name, timeout_sec=timeout_sec)
    return path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main(argv=None) -> int:
    """CLI 入口：``ros2 run nav2 keepout_regen <map_name> [--map-dir ...]``。"""
    import argparse

    parser = argparse.ArgumentParser(
        description='由 Winstec 虛擬牆產生 Nav2 keepout mask 並通知重載')
    parser.add_argument('map_name', help='地圖名稱（不含副檔名）')
    parser.add_argument(
        '--map-dir',
        default=os.environ.get('ROBOT_MAP_PATH',
                               os.path.join(os.path.expanduser('~'), 'base_dev', 'map')),
        help='地圖目錄（預設 $ROBOT_MAP_PATH 或 ~/base_dev/map）')
    parser.add_argument('--no-reload', action='store_true', help='只產生檔案，不呼叫 load_map')
    parser.add_argument('--mask-server', default='filter_mask_server',
                        help='發布 mask 的 map_server 節點名稱')
    args = parser.parse_args(argv)

    logging.basicConfig(level=logging.INFO, format='[%(levelname)s] %(message)s')

    path = regenerate_keepout(args.map_name, args.map_dir)
    if not path:
        return 1
    print(path)
    if not args.no_reload:
        reload_keepout_mask(mask_yaml_path=path, node_name=args.mask_server)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
