/**
 * Winstec Robot API v1.1 契約測試
 *
 * 針對 docs/winstec_api_v1.1.md 驗證真實服務的行為，重點放在「規格容易被實作偏離」
 * 的地方：成功碼、錯誤回應外層鍵名、staging 交易語意、group 兩種列表的形狀差異。
 *
 * 前置：API server 需在 ROBOT_API_BASE 上執行，且機器人上至少有一張地圖。
 *
 * 安全性：所有測試資料都以 E2E_PREFIX 命名，清理時逐一刪除，
 * 絕不使用 `DELETE /points?map=...` 這類會清空整張地圖的批次操作，
 * 因此可以安全地跑在有真實點位的地圖上。
 */
import { test, expect } from '@playwright/test';

const API_BASE = process.env.ROBOT_API_BASE ?? 'http://localhost:5000';
const WS_BASE = process.env.ROBOT_WS_BASE ?? 'ws://localhost:5001';
const V1 = `${API_BASE}/v1/robot`;

/** 測試建立的實體一律用這個前綴，方便精準清理 */
const E2E_PREFIX = 'e2e_';

/** 由 GET /maps 探測而來，在 beforeAll 決定 */
let testMap = '';

interface Res<T> {
  status: number;
  body: T;
}

async function call<T = any>(
  method: string,
  path: string,
  body?: unknown,
): Promise<Res<T>> {
  const res = await fetch(`${V1}${path}`, {
    method,
    headers: body === undefined ? {} : { 'Content-Type': 'application/json' },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const text = await res.text(); // 204 沒有回應體
  return { status: res.status, body: (text ? JSON.parse(text) : null) as T };
}

/** 只刪除本測試建立的實體，不碰地圖上原有的資料 */
async function cleanupE2EData() {
  await call('POST', '/edits/discard');

  const groups = await call<{ groups: Array<{ id: string; name: string }> }>(
    'GET',
    `/groups?map=${testMap}`,
  );
  for (const g of groups.body?.groups ?? []) {
    if (g.name.startsWith(E2E_PREFIX)) await call('DELETE', `/groups/${g.id}`);
  }

  const walls = await call<{
    virtual_walls: Array<{ id: string; name: string }>;
  }>('GET', `/virtual-walls?map=${testMap}`);
  for (const w of walls.body?.virtual_walls ?? []) {
    if (w.name.startsWith(E2E_PREFIX)) {
      await call('DELETE', `/virtual-walls/${w.id}?map=${testMap}`);
    }
  }

  const points = await call<{ points: Array<{ id: string; name: string }> }>(
    'GET',
    `/points?map=${testMap}`,
  );
  for (const p of points.body?.points ?? []) {
    if (p.name.startsWith(E2E_PREFIX)) {
      await call('DELETE', `/points/${p.id}?map=${testMap}`);
    }
  }

  await call('POST', '/edits/commit');
}

test.beforeAll(async () => {
  const maps = await call<{ maps: Array<{ name: string } | string> }>(
    'GET',
    '/maps',
  );
  const list = maps.body?.maps ?? [];
  const first = list[0];
  testMap = typeof first === 'string' ? first : first?.name ?? '';
  expect(testMap, '機器人上至少要有一張地圖才能跑這套測試').toBeTruthy();
});

test.beforeEach(async () => {
  await cleanupE2EData();
});

test.afterAll(async () => {
  await cleanupE2EData();
});

test.describe('Robot Information (§5)', () => {
  test('GET /info 回傳規格要求的四個欄位', async () => {
    const { status, body } = await call('GET', '/info');
    expect(status).toBe(200);

    expect(['explore', 'navigate']).toContain(body.op_mode);
    expect([
      'init',
      'idle',
      'relocating',
      'moving',
      'go_charging',
      'switching_mode',
    ]).toContain(body.status);

    // battery 必須是 0-100 的整數，而不是原始電壓
    expect(Number.isInteger(body.battery)).toBe(true);
    expect(body.battery).toBeGreaterThanOrEqual(0);
    expect(body.battery).toBeLessThanOrEqual(100);

    // Location Object：x/y 整數、orientation 浮點
    expect(Number.isInteger(body.location.x)).toBe(true);
    expect(Number.isInteger(body.location.y)).toBe(true);
    expect(typeof body.location.orientation).toBe('number');
  });
});

test.describe('錯誤回應格式 (§6)', () => {
  test('外層鍵名是 event 而不是 error', async () => {
    const { status, body } = await call('GET', '/points/pt_does_not_exist');
    expect(status).toBe(404);
    // 這是最容易被「順手修正」成 error 的地方，規格明確寫 event
    expect(body).toHaveProperty('event');
    expect(body).not.toHaveProperty('error');
    expect(body.event.code).toBe('POINT_NOT_FOUND');
  });

  test('未知 group 回 404 GROUP_NOT_FOUND', async () => {
    const { status, body } = await call('GET', '/groups/gp_does_not_exist');
    expect(status).toBe(404);
    expect(body.event.code).toBe('GROUP_NOT_FOUND');
  });

  test('不存在的地圖回 404 MAP_NOT_FOUND', async () => {
    const { status, body } = await call('POST', '/points', {
      map: 'map_that_does_not_exist',
      name: `${E2E_PREFIX}orphan`,
      type: 'point',
      location: { x: 1, y: 1, orientation: 0 },
    });
    expect(status).toBe(404);
    expect(body.event.code).toBe('MAP_NOT_FOUND');
  });
});

test.describe('Points CRUD (§5)', () => {
  test('建立回 201、查詢回 200、刪除回 204', async () => {
    const created = await call('POST', '/points', {
      map: testMap,
      name: `${E2E_PREFIX}p1`,
      type: 'point',
      location: { x: 150, y: -80, orientation: 90 },
    });
    expect(created.status).toBe(201);
    expect(created.body.id).toMatch(/^pt_/);
    expect(created.body.map).toBe(testMap);
    expect(created.body.location).toEqual({ x: 150, y: -80, orientation: 90 });

    const id = created.body.id;

    const got = await call('GET', `/points/${id}`);
    expect(got.status).toBe(200);
    expect(got.body.name).toBe(`${E2E_PREFIX}p1`);

    // PATCH 是部分更新，未提供的欄位要保留
    const patched = await call('PATCH', `/points/${id}`, {
      name: `${E2E_PREFIX}renamed`,
    });
    expect(patched.status).toBe(200);
    expect(patched.body.name).toBe(`${E2E_PREFIX}renamed`);
    expect(patched.body.location).toEqual({ x: 150, y: -80, orientation: 90 });

    const deleted = await call('DELETE', `/points/${id}?map=${testMap}`);
    expect(deleted.status).toBe(204);

    const after = await call('GET', `/points/${id}`);
    expect(after.status).toBe(404);
  });

  test('charge 型點位可建立', async () => {
    const { status, body } = await call('POST', '/points', {
      map: testMap,
      name: `${E2E_PREFIX}charge`,
      type: 'charge',
      location: { x: -4, y: 0, orientation: 356 },
    });
    expect(status).toBe(201);
    expect(body.type).toBe('charge');
    // 規格範例的 orientation 是 0-360 而非 -180..180
    expect(body.location.orientation).toBeCloseTo(356, 1);
  });

  test('省略 location 時採用機器人目前位置，取不到位姿則回 GET_LOCATION_FAILED', async () => {
    const info = await call('GET', '/info');
    const { status, body } = await call('POST', '/points', {
      map: testMap,
      name: `${E2E_PREFIX}here`,
      type: 'point',
    });

    // 兩種都是合法結果：有定位時採用當前位置，沒有定位時回規格定義的錯誤碼。
    // 未接硬體或 AMCL 尚未收斂時會走後者。
    if (status === 201) {
      expect(body.location.x).toBe(info.body.location.x);
      expect(body.location.y).toBe(info.body.location.y);
    } else {
      expect(status).toBe(400);
      expect(body.event.code).toBe('GET_LOCATION_FAILED');
    }
  });
});

test.describe('Virtual Walls 與 Groups (§4, §5)', () => {
  test('虛擬牆是線段，用 Position Object（無 orientation）', async () => {
    const { status, body } = await call('POST', '/virtual-walls', {
      map: testMap,
      name: `${E2E_PREFIX}w1`,
      start_position: { x: 624, y: 159 },
      end_position: { x: 524, y: -241 },
    });
    expect(status).toBe(201);
    expect(body.id).toMatch(/^vw_/);
    expect(body.start_position).toEqual({ x: 624, y: 159 });
    expect(body.start_position).not.toHaveProperty('orientation');
  });

  test('group 預設 disabled，兩種虛擬牆列表的形狀不同', async () => {
    const wall = await call('POST', '/virtual-walls', {
      map: testMap,
      name: `${E2E_PREFIX}w_grouped`,
      start_position: { x: 0, y: 0 },
      end_position: { x: 100, y: 0 },
    });
    const group = await call('POST', '/groups', {
      map: testMap,
      name: `${E2E_PREFIX}g1`,
    });
    expect(group.status).toBe(201);

    // is_enable 只在 List Groups 出現，Create 的回應沒有這個欄位
    const listed = await call('GET', `/groups?map=${testMap}`);
    const mine = listed.body.groups.find((g: any) => g.id === group.body.id);
    expect(mine.is_enable).toBe(false); // 規格：預設為 disabled

    const added = await call('POST', `/groups/${group.body.id}/virtual-walls`, {
      id: wall.body.id,
    });
    expect(added.status).toBe(201);
    expect(added.body).toEqual({
      group_id: group.body.id,
      virtual_wall_id: wall.body.id,
    });

    // GET /groups/{id}/virtual-walls 只回 id
    const refs = await call('GET', `/groups/${group.body.id}/virtual-walls`);
    expect(refs.status).toBe(200);
    expect(Object.keys(refs.body.virtual_walls[0])).toEqual(['id']);

    // GET /groups/{id} 回完整虛擬牆物件
    const detail = await call('GET', `/groups/${group.body.id}`);
    expect(detail.status).toBe(200);
    expect(detail.body.virtual_walls[0]).toHaveProperty('start_position');
    expect(detail.body.virtual_walls[0]).toHaveProperty('name');

    // 啟用後 apply 應成功（會觸發 keepout mask 重新產生）
    const enabled = await call('PATCH', `/groups/${group.body.id}`, {
      is_enable: true,
    });
    expect(enabled.status).toBe(200);
    expect(enabled.body.is_enable).toBe(true);

    const applied = await call('POST', '/groups/actions/apply');
    expect(applied.status).toBe(200);

    const removed = await call(
      'DELETE',
      `/groups/${group.body.id}/virtual-walls/${wall.body.id}`,
    );
    expect(removed.status).toBe(204);
  });
});

test.describe('編輯交易語意 (§4.2, §5)', () => {
  test('discard 丟棄未提交變更', async () => {
    const before = await call('GET', `/points?map=${testMap}`);
    const n = before.body.points.length;

    await call('POST', '/points', {
      map: testMap,
      name: `${E2E_PREFIX}staged_only`,
      type: 'point',
      location: { x: 1, y: 1, orientation: 0 },
    });

    // 暫存中的變更對讀取是可見的
    const staged = await call('GET', `/points?map=${testMap}`);
    expect(staged.body.points.length).toBe(n + 1);

    const discarded = await call('POST', '/edits/discard');
    expect(discarded.status).toBe(200);

    const after = await call('GET', `/points?map=${testMap}`);
    expect(after.body.points.length).toBe(n);
    expect(
      after.body.points.some((p: any) => p.name === `${E2E_PREFIX}staged_only`),
    ).toBe(false);
  });

  test('commit 之後 discard 不會回滾已提交的資料', async () => {
    await call('POST', '/points', {
      map: testMap,
      name: `${E2E_PREFIX}committed`,
      type: 'point',
      location: { x: 2, y: 2, orientation: 0 },
    });
    const committed = await call('POST', '/edits/commit');
    expect(committed.status).toBe(200);

    await call('POST', '/edits/discard');

    const after = await call('GET', `/points?map=${testMap}`);
    expect(
      after.body.points.some((p: any) => p.name === `${E2E_PREFIX}committed`),
    ).toBe(true);
  });
});

test.describe('WebSocket 事件 (§5 Event Reference)', () => {
  test('robot_info 每秒推播且欄位符合規格', async ({ page }) => {
    const frames = await page.evaluate(
      ({ wsUrl }) =>
        new Promise<any[]>((resolve, reject) => {
          const got: any[] = [];
          const ws = new WebSocket(wsUrl);
          const timer = setTimeout(() => {
            ws.close();
            reject(new Error(`只收到 ${got.length} 則 robot_info`));
          }, 20000);

          ws.onmessage = (ev) => {
            const msg = JSON.parse(ev.data);
            if (msg.event === 'robot_info') got.push(msg);
            if (got.length >= 3) {
              clearTimeout(timer);
              ws.close();
              resolve(got);
            }
          };
          ws.onerror = () => {
            clearTimeout(timer);
            reject(new Error('WebSocket 連線失敗'));
          };
        }),
      { wsUrl: WS_BASE },
    );

    expect(frames.length).toBeGreaterThanOrEqual(3);
    for (const f of frames) {
      expect(f.event).toBe('robot_info');
      expect(Number.isInteger(f.battery)).toBe(true);
      expect(f.location).toHaveProperty('orientation');
    }
  });
});
