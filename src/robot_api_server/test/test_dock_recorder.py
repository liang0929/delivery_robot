"""充電座位置記錄的離線測試（不需要 ROS 執行期，也不起節點）。

涵蓋四件事：

1. **yaw 換算**——``dock_yaw = base_link_yaw + 180°``。記反的後果是 staging
   pose 落到充電座**內側**（牆裡），車會被導去一個進不去的點。
2. **寫檔原子性與 yaml 有效性**——寫完立刻 re-parse，並確認 temp 檔沒殘留、
   寫入失敗時原檔不被截斷。docking_server 是 lifecycle node，它會在
   on_configure 讀這個檔，讀到寫一半的內容就是啟動失敗。
3. **既有條目不被吃掉**——重生成整份 yaml 是為了寫得出說明註解，代價是
   「沒被記錄到的 dock 也會經過我們的手」，所以必須測它們原封不動。
4. **端點三條路徑**——查不到 TF（Nav2 未啟動的現況）、odom 測試模式標記、
   map 正式路徑；TF 用假的 service 注入，不依賴車上真實 ``/tf``。

端點測試用 FastAPI 的 TestClient，並以 monkeypatch 換掉 router 模組層級的
``service`` 與 ``DOCK_DATABASE_PATH``——真正要驗的是「TF 結果如何變成 HTTP
回應與檔案內容」，不是 rclpy。
"""

import math
import os

import pytest
import yaml

from robot_api_server.dock_recorder import (
    DEFAULT_DOCK_ID,
    DockDatabaseError,
    base_link_to_dock_pose,
    is_test_frame,
    load_docks,
    record_dock_pose,
    render_database,
    wrap_to_pi,
)


# --------------------------------------------------------------- yaw 換算

@pytest.mark.parametrize(
    "base_yaw_deg, expect_dock_yaw_deg",
    [
        (0.0, 180.0),      # 車頭朝 +x → 車尾（dock）朝 -x
        (90.0, -90.0),     # 正規化到 (-π, π]，不是 270°
        (-90.0, 90.0),
        (180.0, 0.0),
        (179.0, -1.0),     # 跨越 ±π 的邊界
    ],
)
def test_dock_yaw_is_base_link_yaw_plus_pi(base_yaw_deg, expect_dock_yaw_deg):
    _, _, dock_yaw = base_link_to_dock_pose(0.0, 0.0, math.radians(base_yaw_deg))
    assert dock_yaw == pytest.approx(math.radians(expect_dock_yaw_deg), abs=1e-9)


def test_position_unchanged_without_contact_offset():
    """contact_offset 為 0 時位置就是 base_link 原點，不得自作聰明平移。"""
    x, y, _ = base_link_to_dock_pose(1.25, -3.5, 0.7)
    assert (x, y) == (1.25, -3.5)


def test_contact_offset_moves_along_dock_x_axis():
    """offset 沿 dock 的 +x（指向充電座內部）＝ base_link 的 -x（車尾方向）。"""
    x, y, dock_yaw = base_link_to_dock_pose(0.0, 0.0, 0.0, contact_offset_m=0.3)
    assert dock_yaw == pytest.approx(math.pi)
    assert x == pytest.approx(-0.3)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_wrap_to_pi_normalizes_multiple_turns():
    # ±π 是同一個朝向，atan2 落在哪一側由浮點誤差決定，只斷言大小
    assert abs(wrap_to_pi(3 * math.pi)) == pytest.approx(math.pi)
    assert abs(wrap_to_pi(-3 * math.pi)) == pytest.approx(math.pi)
    assert wrap_to_pi(math.radians(370.0)) == pytest.approx(math.radians(10.0))


def test_only_map_is_a_production_frame():
    assert not is_test_frame('map')
    assert is_test_frame('odom')
    assert is_test_frame('base_footprint')


# --------------------------------------------------- 寫檔：原子性與有效性

@pytest.fixture
def db_path(tmp_path):
    """帶一個佔位條目的 database，貼近 repo 內的初始狀態。"""
    path = tmp_path / 'dock_database.yaml'
    path.write_text(
        '# 註解\ndocks:\n  home_dock:\n    type: "charging_dock"\n'
        '    frame: "map"\n    pose: [0.0, 0.0, 0.0]\n',
        encoding='utf-8',
    )
    return str(path)


def test_written_yaml_reparses_with_the_recorded_pose(db_path):
    result = record_dock_pose(db_path, 2.0, -1.0, 0.0, frame='map')

    reparsed = yaml.safe_load(open(db_path, encoding='utf-8'))
    entry = reparsed['docks'][DEFAULT_DOCK_ID]
    assert entry['type'] == 'charging_dock'
    assert entry['frame'] == 'map'
    # opennav_docking 的 parseDockFile 對長度不是 3 的 pose 直接報錯
    assert len(entry['pose']) == 3
    assert entry['pose'][0] == pytest.approx(2.0)
    assert entry['pose'][1] == pytest.approx(-1.0)
    assert entry['pose'][2] == pytest.approx(math.pi, abs=1e-3)
    assert result['test_only'] is False


def test_no_temp_file_left_behind(db_path):
    record_dock_pose(db_path, 1.0, 1.0, 0.0)
    leftovers = [f for f in os.listdir(os.path.dirname(db_path)) if f.endswith('.tmp')]
    assert leftovers == []


def test_original_file_survives_a_failed_write(db_path):
    """寫入失敗（例如磁碟滿）不得留下截斷的檔案——原檔必須原封不動。"""
    before = open(db_path, encoding='utf-8').read()

    def boom(*_args, **_kwargs):
        raise OSError("No space left on device")

    import robot_api_server.dock_recorder as mod
    original = mod.write_atomic
    mod.write_atomic = boom
    try:
        with pytest.raises(DockDatabaseError):
            record_dock_pose(db_path, 1.0, 1.0, 0.0)
    finally:
        mod.write_atomic = original

    assert open(db_path, encoding='utf-8').read() == before


def test_other_docks_are_preserved(tmp_path):
    path = str(tmp_path / 'db.yaml')
    open(path, 'w', encoding='utf-8').write(
        'docks:\n'
        '  home_dock:\n    type: "charging_dock"\n    frame: "map"\n    pose: [0.0, 0.0, 0.0]\n'
        '  spare_dock:\n    type: "charging_dock"\n    frame: "map"\n    pose: [9.0, 8.0, 1.5]\n'
    )
    record_dock_pose(path, 1.0, 2.0, 0.0)

    docks = yaml.safe_load(open(path, encoding='utf-8'))['docks']
    assert set(docks) == {'home_dock', 'spare_dock'}
    assert docks['spare_dock']['pose'] == [9.0, 8.0, 1.5]


def test_odom_record_is_marked_as_test_only_in_file_and_result(db_path):
    result = record_dock_pose(db_path, 0.5, 0.5, 0.0, frame='odom')
    assert result['test_only'] is True
    assert result['frame'] == 'odom'

    text = open(db_path, encoding='utf-8').read()
    assert 'frame: "odom"' in text
    # 現場讀檔的人必須一眼看到這不是正式值
    assert '測試值' in text


def test_test_frame_warning_names_the_actual_frame(db_path):
    """警告文字不得寫死 odom——base_footprint 之類的 frame 也走這條路徑，
    寫死會讓現場讀到一段與檔案內容矛盾的說明。"""
    record_dock_pose(db_path, 0.0, 0.0, 0.0, frame='base_footprint')
    text = open(db_path, encoding='utf-8').read()

    assert 'base_footprint' in text
    assert 'odom' not in text


def test_corrupt_database_is_not_silently_overwritten(tmp_path):
    path = str(tmp_path / 'broken.yaml')
    open(path, 'w', encoding='utf-8').write('docks: [this is a list, not a map]\n')
    with pytest.raises(DockDatabaseError):
        record_dock_pose(path, 1.0, 1.0, 0.0)


def test_load_docks_on_missing_file_returns_empty(tmp_path):
    assert load_docks(str(tmp_path / 'nope.yaml')) == {}


def test_render_is_stable_and_parseable_for_a_fresh_entry():
    text = render_database({'home_dock': {'type': 'charging_dock', 'frame': 'map',
                                          'pose': [0.0, 0.0, 0.0]}})
    assert yaml.safe_load(text)['docks']['home_dock']['pose'] == [0.0, 0.0, 0.0]
    # 沒有記錄過的條目要明講是佔位值
    assert '佔位值' in text


# ------------------------------------------------------------- 端點三條路徑

@pytest.fixture
def endpoint(db_path, monkeypatch):
    """直接呼叫 router 的 coroutine，回傳 ``(call, fake_service)``。

    不走 ``fastapi.testclient``：它需要 httpx，而本機沒有、本工單也不准裝。
    直接 await router 函式一樣涵蓋要驗的東西（錯誤碼、detail、回應模型、
    寫進去的檔案內容），只是少了 HTTP 序列化那一層——那一層由
    ``errors.register_exception_handlers`` 負責，另有 ``test_error_body_*``
    直接驗它的輸出格式。
    """
    from robot_api_server.routers import dock as dock_router

    class FakeService:
        """只實作 lookup_pose：router 用得到的就這一個。"""

        def __init__(self):
            self.poses = {}

        def lookup_pose(self, target_frame, source_frame='base_link', timeout_sec=1.0):
            if target_frame in self.poses:
                return self.poses[target_frame], f"{target_frame}→{source_frame}"
            return None, f"查不到 {target_frame}→{source_frame} 的 TF：frame 不存在"

    fake = FakeService()
    monkeypatch.setattr(dock_router, 'service', fake)
    monkeypatch.setattr(dock_router, 'DOCK_DATABASE_PATH', db_path)

    def call(**kwargs):
        import asyncio
        params = {'frame': 'map', 'dock_id': DEFAULT_DOCK_ID, 'contact_offset': 0.0}
        params.update(kwargs)
        return asyncio.run(dock_router.record_dock_pose_endpoint(**params))

    return call, fake


def test_endpoint_without_map_frame_fails_with_actionable_detail(endpoint):
    """Nav2 未啟動的現況：必須明確失敗，並講得出替代路徑。"""
    from robot_api_server import errors

    call, _ = endpoint
    with pytest.raises(errors.ApiError) as exc:
        call()
    assert exc.value.code == 'DOCK_POSE_UNAVAILABLE'
    assert exc.value.status_code == 409
    assert 'Nav2' in exc.value.detail
    assert 'odom' in exc.value.detail


def test_endpoint_odom_test_mode_succeeds_and_flags_test_only(endpoint, db_path):
    call, fake = endpoint
    fake.poses['odom'] = (1.5, -0.5, 0.0)

    res = call(frame='odom')
    assert res.test_only is True
    assert res.frame == 'odom'
    assert res.pose.yaw_deg == pytest.approx(180.0)
    assert res.base_link.yaw_deg == pytest.approx(0.0)

    entry = yaml.safe_load(open(db_path, encoding='utf-8'))['docks']['home_dock']
    assert entry['frame'] == 'odom'
    assert entry['pose'][0] == pytest.approx(1.5)


def test_endpoint_map_frame_writes_production_value(endpoint, db_path):
    """map frame 目前不存在，用注入的 TF 證明正式路徑的換算與寫檔正確。"""
    call, fake = endpoint
    fake.poses['map'] = (3.0, 4.0, math.radians(90.0))

    res = call()
    assert res.test_only is False
    assert res.frame == 'map'
    assert res.pose.yaw_deg == pytest.approx(-90.0)

    entry = yaml.safe_load(open(db_path, encoding='utf-8'))['docks']['home_dock']
    assert entry['pose'][2] == pytest.approx(math.radians(-90.0), abs=1e-3)


def test_endpoint_contact_offset_shifts_toward_the_dock(endpoint):
    call, fake = endpoint
    fake.poses['map'] = (0.0, 0.0, 0.0)

    res = call(contact_offset=0.25)
    assert res.contact_offset_m == pytest.approx(0.25)
    assert res.pose.x_m == pytest.approx(-0.25)


def test_endpoint_write_failure_maps_to_500(endpoint, db_path):
    """database 壞掉時不能回 200，也不能讓例外裸奔成 FastAPI 的 500 預設格式。"""
    from robot_api_server import errors

    call, fake = endpoint
    fake.poses['map'] = (0.0, 0.0, 0.0)
    open(db_path, 'w', encoding='utf-8').write('docks: not-a-mapping\n')

    with pytest.raises(errors.ApiError) as exc:
        call()
    assert exc.value.code == 'DOCK_DB_WRITE_FAILED'
    assert exc.value.status_code == 500


# ------------------------------------------------------------ 錯誤回應格式

def test_error_body_without_detail_matches_spec():
    """既有端點一個字都不能變：沒有 detail 就是規格 §6.1 的原樣。"""
    from robot_api_server import errors

    assert errors.error_body('POINT_NOT_FOUND') == {"event": {"code": "POINT_NOT_FOUND"}}


def test_error_body_with_detail_adds_extension_field():
    from robot_api_server import errors

    body = errors.error_body('DOCK_POSE_UNAVAILABLE', '沒有 map frame')
    assert body == {"event": {"code": "DOCK_POSE_UNAVAILABLE", "detail": "沒有 map frame"}}
