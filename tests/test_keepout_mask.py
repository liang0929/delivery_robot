#!/usr/bin/env python3
"""keepout mask 產生器離線驗證腳本（不需要 ROS/硬體）。

用法::

    /usr/bin/python3 tests/test_keepout_mask.py

以 ``map/01.yaml`` / ``map/01.pgm`` 為底，在暫存目錄造假的
``virtual_walls.json`` / ``groups.json``，驗證：

1. 輸出 PGM 尺寸與原圖一致
2. 線段畫在正確的像素位置（含公分→公尺→像素、PGM y 軸翻轉）
3. 只有 ``is_enable: true`` 的 group 內的牆會生效
4. 沒有任何 enabled 牆時仍輸出一張全 0 的 mask
5. keepout.yaml 可用 yaml.safe_load 解析且 mode 為 scale
"""

import json
import math
import os
import shutil
import sys
import tempfile

import numpy as np
import yaml
from PIL import Image

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO_ROOT, 'src', 'nav2'))

from nav2.keepout import (  # noqa: E402
    FREE_VALUE,
    KEEPOUT_VALUE,
    ROBOT_RADIUS_M,
    regenerate_keepout,
    world_to_pixel,
)

SRC_MAP = os.path.join(REPO_ROOT, 'map', '01')
MAP_NAME = 'testmap'

_failures = []


def check(condition, message):
    status = 'PASS' if condition else 'FAIL'
    print(f'[{status}] {message}')
    if not condition:
        _failures.append(message)


def make_workspace(tmpdir, walls, groups):
    """複製底圖並寫入假的 walls/groups JSON。"""
    shutil.copy(f'{SRC_MAP}.pgm', os.path.join(tmpdir, f'{MAP_NAME}.pgm'))
    with open(f'{SRC_MAP}.yaml', 'r', encoding='utf-8') as fp:
        meta = yaml.safe_load(fp)
    meta['image'] = f'{MAP_NAME}.pgm'
    with open(os.path.join(tmpdir, f'{MAP_NAME}.yaml'), 'w', encoding='utf-8') as fp:
        yaml.safe_dump(meta, fp, default_flow_style=False, sort_keys=False)

    with open(os.path.join(tmpdir, f'{MAP_NAME}.virtual_walls.json'), 'w', encoding='utf-8') as fp:
        json.dump(walls, fp)
    with open(os.path.join(tmpdir, f'{MAP_NAME}.groups.json'), 'w', encoding='utf-8') as fp:
        json.dump(groups, fp)
    return meta


def wall(wall_id, name, start_cm, end_cm):
    return {
        'id': wall_id,
        'map': MAP_NAME,
        'name': name,
        'start_position': {'x': start_cm[0], 'y': start_cm[1]},
        'end_position': {'x': end_cm[0], 'y': end_cm[1]},
    }


def main():
    orig = np.array(Image.open(f'{SRC_MAP}.pgm'))
    orig_h, orig_w = orig.shape
    print(f'底圖 01.pgm: {orig_w}x{orig_h}')

    # --- 情境 1：一面 enabled、一面 disabled、一面沒歸屬 group ---
    walls = [
        wall('vw_enabled', 'enabled-wall', (0, 0), (100, 0)),        # 水平線 y=0m, x 0→1m
        wall('vw_disabled', 'disabled-wall', (200, -100), (200, -200)),  # 垂直線 x=2m
        wall('vw_orphan', 'orphan-wall', (300, -100), (300, -200)),      # 不在任何 group
    ]
    groups = [
        {'id': 'gp_on', 'map': MAP_NAME, 'name': 'on', 'is_enable': True,
         'virtual_wall_ids': ['vw_enabled']},
        {'id': 'gp_off', 'map': MAP_NAME, 'name': 'off', 'is_enable': False,
         'virtual_wall_ids': ['vw_disabled']},
    ]

    with tempfile.TemporaryDirectory() as tmpdir:
        meta = make_workspace(tmpdir, walls, groups)
        resolution = float(meta['resolution'])
        origin = meta['origin']

        out_yaml = regenerate_keepout(MAP_NAME, tmpdir)
        check(out_yaml == os.path.join(tmpdir, f'{MAP_NAME}.keepout.yaml'),
              f'regenerate_keepout 回傳 keepout.yaml 路徑: {out_yaml}')

        with open(out_yaml, 'r', encoding='utf-8') as fp:
            mask_meta = yaml.safe_load(fp)
        check(mask_meta['mode'] == 'scale', "keepout.yaml mode == 'scale'")
        check(mask_meta['image'] == f'{MAP_NAME}.keepout.pgm', 'keepout.yaml image 欄位正確')
        check(abs(float(mask_meta['resolution']) - resolution) < 1e-9,
              'keepout.yaml resolution 與地圖一致')
        check([round(v, 6) for v in mask_meta['origin'][:2]] == [round(v, 6) for v in origin[:2]],
              'keepout.yaml origin 與地圖一致')

        mask = np.array(Image.open(os.path.join(tmpdir, f'{MAP_NAME}.keepout.pgm')))
        check(mask.shape == (orig_h, orig_w),
              f'mask 尺寸 {mask.shape[1]}x{mask.shape[0]} 與原圖一致')
        check(set(np.unique(mask).tolist()) <= {FREE_VALUE, KEEPOUT_VALUE},
              f'mask 只含 {FREE_VALUE}/{KEEPOUT_VALUE}，實際 {np.unique(mask).tolist()}')

        line_width = max(1, math.ceil(ROBOT_RADIUS_M / resolution))
        check(line_width == 10, f'線寬 = {line_width} px (0.25/{resolution})')

        # enabled 牆：檢查兩端點與中點
        for label, (x_cm, y_cm) in [('起點', (0, 0)), ('中點', (50, 0)), ('終點', (100, 0))]:
            col, row = world_to_pixel(x_cm / 100.0, y_cm / 100.0, origin, resolution, orig_h)
            check(mask[row, col] == KEEPOUT_VALUE,
                  f'enabled 牆 {label} 像素 ({col},{row}) == {KEEPOUT_VALUE}')

        # y 軸翻轉驗證：y 越大 row 越小
        col_a, row_a = world_to_pixel(0.0, 0.0, origin, resolution, orig_h)
        _, row_b = world_to_pixel(0.0, -1.0, origin, resolution, orig_h)
        check(row_b > row_a, f'PGM y 軸翻轉正確 (y=0 → row {row_a}, y=-1 → row {row_b})')

        # 線寬垂直分布：中心 ±(line_width//2 - 1) 應仍在牆內，遠處為自由
        check(mask[row_a - (line_width // 2 - 1), col_a + 5] == KEEPOUT_VALUE,
              '線寬覆蓋中心上方像素')
        check(mask[row_a + line_width, col_a + 5] == FREE_VALUE,
              f'距中心 {line_width} px 已在牆外')

        # disabled group 的牆不得出現
        for label, wall_id, (x_cm, y_cm) in [
            ('disabled', 'vw_disabled', (200, -150)),
            ('orphan', 'vw_orphan', (300, -150)),
        ]:
            col, row = world_to_pixel(x_cm / 100.0, y_cm / 100.0, origin, resolution, orig_h)
            check(mask[row, col] == FREE_VALUE,
                  f'{label} 牆 ({wall_id}) 未畫入，像素 ({col},{row}) == {FREE_VALUE}')

        drawn = int((mask == KEEPOUT_VALUE).sum())
        expected_min = (100 / 100.0) / resolution * line_width  # 1m 長 × 線寬
        check(drawn >= expected_min * 0.9,
              f'禁行像素數 {drawn} 合理 (>= {expected_min * 0.9:.0f})')

    # --- 情境 2：group 全部 disabled → 全 0 mask ---
    with tempfile.TemporaryDirectory() as tmpdir:
        make_workspace(tmpdir, walls, [
            {'id': 'gp_off', 'map': MAP_NAME, 'name': 'off', 'is_enable': False,
             'virtual_wall_ids': ['vw_enabled', 'vw_disabled']},
        ])
        out_yaml = regenerate_keepout(MAP_NAME, tmpdir)
        check(out_yaml is not None, '全 disabled 時仍輸出 mask')
        mask = np.array(Image.open(os.path.join(tmpdir, f'{MAP_NAME}.keepout.pgm')))
        check(mask.shape == (orig_h, orig_w), '全 0 mask 尺寸正確')
        check(int(mask.max()) == 0, '全 disabled → mask 全 0')

    # --- 情境 3：沒有任何 walls/groups 檔案 ---
    with tempfile.TemporaryDirectory() as tmpdir:
        shutil.copy(f'{SRC_MAP}.pgm', os.path.join(tmpdir, f'{MAP_NAME}.pgm'))
        with open(f'{SRC_MAP}.yaml', 'r', encoding='utf-8') as fp:
            meta = yaml.safe_load(fp)
        meta['image'] = f'{MAP_NAME}.pgm'
        with open(os.path.join(tmpdir, f'{MAP_NAME}.yaml'), 'w', encoding='utf-8') as fp:
            yaml.safe_dump(meta, fp)
        out_yaml = regenerate_keepout(MAP_NAME, tmpdir)
        check(out_yaml is not None, '缺 walls/groups JSON 時仍輸出 mask')
        mask = np.array(Image.open(os.path.join(tmpdir, f'{MAP_NAME}.keepout.pgm')))
        check(int(mask.max()) == 0, '缺 JSON → mask 全 0')

    # --- 情境 4：多 group 共用同一面牆，其一 enabled ---
    with tempfile.TemporaryDirectory() as tmpdir:
        meta = make_workspace(tmpdir, walls, [
            {'id': 'gp_a', 'map': MAP_NAME, 'name': 'a', 'is_enable': False,
             'virtual_wall_ids': ['vw_orphan']},
            {'id': 'gp_b', 'map': MAP_NAME, 'name': 'b', 'is_enable': True,
             'virtual_wall_ids': ['vw_orphan']},
        ])
        regenerate_keepout(MAP_NAME, tmpdir)
        mask = np.array(Image.open(os.path.join(tmpdir, f'{MAP_NAME}.keepout.pgm')))
        col, row = world_to_pixel(3.0, -1.5, meta['origin'], float(meta['resolution']), orig_h)
        check(mask[row, col] == KEEPOUT_VALUE,
              '牆同時屬於 disabled/enabled group 時仍生效')

    # --- 情境 5：地圖不存在 → None ---
    with tempfile.TemporaryDirectory() as tmpdir:
        check(regenerate_keepout('nonexistent', tmpdir) is None,
              '地圖不存在時回傳 None')

    print()
    if _failures:
        print(f'{len(_failures)} 項失敗：')
        for item in _failures:
            print(f'  - {item}')
        return 1
    print('全部通過')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
