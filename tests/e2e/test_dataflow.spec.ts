import { test, expect } from '@playwright/test';

const FRONTEND_URL = 'http://localhost:3001';
const API_URL = 'http://localhost:8000';

// Helper: call API directly
async function apiGet(path: string) {
  const res = await fetch(`${API_URL}${path}`);
  return res.json();
}

async function apiPost(path: string, body?: object) {
  const res = await fetch(`${API_URL}${path}`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    ...(body ? { body: JSON.stringify(body) } : {}),
  });
  const text = await res.text();
  let data: any;
  try {
    data = JSON.parse(text);
  } catch {
    data = { error: text };
  }
  return { status: res.status, data };
}

// Helper: 等待條件成立（polling）
async function waitFor(
  fn: () => Promise<boolean>,
  timeoutMs: number = 30000,
  intervalMs: number = 1000,
): Promise<void> {
  const start = Date.now();
  while (Date.now() - start < timeoutMs) {
    if (await fn()) return;
    await new Promise(r => setTimeout(r, intervalMs));
  }
  throw new Error(`waitFor timeout after ${timeoutMs}ms`);
}

// Helper: 確保乾淨狀態（取消送餐 + 停止導航）
async function ensureCleanState() {
  try { await apiPost('/delivery/cancel'); } catch {}
  await new Promise(r => setTimeout(r, 500));
  try { await apiPost('/navigation/stop'); } catch {}
  await new Promise(r => setTimeout(r, 2000));
}

// ============================
// 1. API 後端基礎健康檢查
// ============================
test.describe('API 後端健康檢查', () => {
  test('GET /health 回傳正確結構', async () => {
    const data = await apiGet('/health');
    expect(data).toHaveProperty('robot_core');
    expect(data).toHaveProperty('slam');
    expect(data).toHaveProperty('navigation');
    expect(data.robot_core).toHaveProperty('running');
    expect(data.slam).toHaveProperty('status');
    expect(data.navigation).toHaveProperty('status');
  });

  test('GET /robot/status 回傳 is_running', async () => {
    const data = await apiGet('/robot/status');
    expect(data).toHaveProperty('is_running');
    expect(typeof data.is_running).toBe('boolean');
  });

  test('GET /robot/e_stop 回傳 active 狀態', async () => {
    const data = await apiGet('/robot/e_stop');
    expect(data).toHaveProperty('active');
    expect(typeof data.active).toBe('boolean');
  });

  test('GET /maps/list 回傳地圖列表', async () => {
    const data = await apiGet('/maps/list');
    expect(data).toHaveProperty('maps');
    expect(Array.isArray(data.maps)).toBe(true);
    expect(data.maps.length).toBeGreaterThan(0);
  });
});

// ============================
// 2. SLAM 建圖資料流
// ============================
test.describe('SLAM 建圖資料流', () => {
  test('啟動建圖 → 查詢狀態 → 停止建圖', async () => {
    // 啟動建圖
    const startRes = await apiPost('/slam/start');
    expect(startRes.status).toBe(200);

    // 等待 SLAM 進程啟動
    await new Promise(r => setTimeout(r, 3000));

    // 查詢狀態應為 mapping
    const status = await apiGet('/slam/status');
    expect(status.status).toBe('mapping');

    // 停止建圖
    const stopRes = await apiPost('/slam/stop');
    expect(stopRes.status).toBe(200);

    // 等待進程停止
    await new Promise(r => setTimeout(r, 2000));

    // 狀態應回到 idle
    const statusAfter = await apiGet('/slam/status');
    expect(statusAfter.status).toBe('idle');
  });
});

// ============================
// 3. 地圖管理資料流
// ============================
test.describe('地圖管理', () => {
  test('列出地圖並取得元數據', async () => {
    const list = await apiGet('/maps/list');
    expect(list.maps.length).toBeGreaterThan(0);

    const mapName = list.maps[0].name;

    // 取得元數據
    const metadata = await apiGet(`/maps/${mapName}/metadata`);
    expect(metadata).toHaveProperty('resolution');
    expect(metadata).toHaveProperty('width');
    expect(metadata).toHaveProperty('height');
    expect(metadata).toHaveProperty('origin');
  });

  test('取得地圖圖像', async () => {
    const list = await apiGet('/maps/list');
    const mapName = list.maps[0].name;

    const res = await fetch(`${API_URL}/maps/${mapName}/image`);
    expect(res.status).toBe(200);
    expect(res.headers.get('content-type')).toContain('image/png');
  });
});

// ============================
// 4. 桌位管理 CRUD 資料流
// ============================
test.describe('桌位管理 CRUD', () => {
  let mapName: string;
  let tableId: string;

  test.beforeAll(async () => {
    const list = await apiGet('/maps/list');
    mapName = list.maps[0].name;
  });

  test('建立桌位 → 查詢 → 更新 → 刪除', async () => {
    // 建立
    const createRes = await apiPost(`/maps/${mapName}/tables`, {
      number: 99,
      name: 'Test Table',
      x: 1.0,
      y: 2.0,
      yaw_deg: 90.0,
      isActive: true,
    });
    expect(createRes.status).toBe(200);
    tableId = createRes.data.id;
    expect(createRes.data.number).toBe(99);
    expect(createRes.data.name).toBe('Test Table');

    // 查詢
    const tables = await apiGet(`/maps/${mapName}/tables`);
    const found = tables.find((t: any) => t.id === tableId);
    expect(found).toBeDefined();
    expect(found.x).toBe(1.0);
    expect(found.y).toBe(2.0);

    // 更新
    const updateRes = await fetch(`${API_URL}/maps/${mapName}/tables/${tableId}`, {
      method: 'PUT',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        number: 99,
        name: 'Updated Table',
        x: 3.0,
        y: 4.0,
        yaw_deg: 180.0,
        isActive: true,
      }),
    });
    expect(updateRes.status).toBe(200);
    const updated = await updateRes.json();
    expect(updated.name).toBe('Updated Table');
    expect(updated.x).toBe(3.0);

    // 刪除
    const deleteRes = await fetch(`${API_URL}/maps/${mapName}/tables/${tableId}`, {
      method: 'DELETE',
    });
    expect(deleteRes.status).toBe(200);

    // 確認已刪除
    const tablesAfter = await apiGet(`/maps/${mapName}/tables`);
    const notFound = tablesAfter.find((t: any) => t.id === tableId);
    expect(notFound).toBeUndefined();
  });
});

// ============================
// 5. 導航資料流
// ============================
test.describe('導航資料流', () => {
  test.beforeAll(async () => {
    // 確保乾淨狀態
    await ensureCleanState();
  });

  test('啟動導航 → 查詢狀態 → 停止導航', async () => {
    const list = await apiGet('/maps/list');
    const mapName = list.maps[0].name;

    // 啟動導航
    const startRes = await apiPost('/navigation/start', { map_name: mapName });
    expect(startRes.status).toBe(200);

    // Polling 等待 Nav2 啟動（Jetson 上需要較長時間）
    await waitFor(async () => {
      const status = await apiGet('/navigation/status');
      return status.nav_running === true;
    }, 90000, 2000);

    // 確認狀態
    const status = await apiGet('/navigation/status');
    expect(status.nav_running).toBe(true);

    // 停止導航
    const stopRes = await apiPost('/navigation/stop');
    expect(stopRes.status).toBe(200);

    // 等待停止
    await waitFor(async () => {
      const s = await apiGet('/navigation/status');
      return s.nav_running === false;
    }, 15000, 1000);

    const statusAfter = await apiGet('/navigation/status');
    expect(statusAfter.nav_running).toBe(false);
  });
});

// ============================
// 6. 送餐資料流（完整狀態機）
// ============================
test.describe('送餐資料流', () => {
  let mapName: string;
  let tableId: string;

  test.beforeAll(async () => {
    // 確保乾淨狀態
    await ensureCleanState();

    // 找一個有桌位的地圖
    const list = await apiGet('/maps/list');
    for (const map of list.maps) {
      const tables = await apiGet(`/maps/${map.name}/tables`);
      if (tables.length > 0) {
        mapName = map.name;
        tableId = tables[0].id;
        break;
      }
    }

    // 如果沒有桌位，建立一個
    if (!tableId) {
      mapName = list.maps[0].name;
      const res = await apiPost(`/maps/${mapName}/tables`, {
        number: 1,
        name: 'Delivery Test',
        x: 1.0,
        y: 1.0,
        yaw_deg: 0.0,
        isActive: true,
      });
      tableId = res.data.id;
    }

    // 啟動導航（Nav2 進程啟動即可，不等待 lifecycle nodes 就緒）
    const navStatus = await apiGet('/navigation/status');
    if (!navStatus.nav_running) {
      await apiPost('/navigation/start', { map_name: mapName });
      await waitFor(async () => {
        const s = await apiGet('/navigation/status');
        return s.nav_running === true;
      }, 30000, 1000);
    }
  });

  test('啟動送餐 → 查詢狀態 → 取消送餐', async () => {
    // 啟動送餐（導航目標發送為非阻塞，不影響 API 回應）
    const startRes = await apiPost('/delivery/start', {
      mapName: mapName,
      tableIds: [tableId],
      startPosition: { x: 0.0, y: 0.0, yaw: 0.0 },
    });
    expect(startRes.status).toBe(200);
    expect(startRes.data).toHaveProperty('id');
    expect(startRes.data.status).toBe('delivering');
    expect(startRes.data.stops).toHaveLength(1);
    expect(startRes.data.stops[0].tableId).toBe(tableId);

    // 等一下讓 monitor loop 跑幾次
    await new Promise(r => setTimeout(r, 3000));

    // 查詢狀態 — task 不應該被誤判為完成（N1 bug 驗證）
    const status = await apiGet('/delivery/status');
    expect(status).toHaveProperty('task');
    expect(status).toHaveProperty('isStuck');
    if (status.task) {
      // 不應該是 idle（誤判完成）或 at_table（機器人沒動不可能到達）
      expect(['delivering', 'stuck']).toContain(status.task.status);
    }

    // 取消送餐
    const cancelRes = await apiPost('/delivery/cancel');
    expect(cancelRes.status).toBe(200);

    // 確認已取消
    await new Promise(r => setTimeout(r, 500));
    const statusAfter = await apiGet('/delivery/status');
    expect(statusAfter.task).toBeNull();
  });

  test.afterAll(async () => {
    await ensureCleanState();
  });
});

// ============================
// 7. 前端頁面載入測試
// ============================
test.describe('前端頁面載入', () => {
  test('首頁載入成功', async ({ page }) => {
    await page.goto(FRONTEND_URL);
    await page.waitForLoadState('networkidle');
    const body = await page.textContent('body');
    expect(body).toBeTruthy();
  });

  test('前端能從 API 取得資料', async ({ page }) => {
    const apiResponses: { url: string; status: number }[] = [];
    page.on('response', (res) => {
      const url = res.url();
      // 匹配 API 呼叫：port 8000
      if (url.includes(':8000')) {
        apiResponses.push({ url, status: res.status() });
      }
    });

    // 導航到會打 API 的頁面（waiter dashboard 會 fetch maps）
    await page.goto(`${FRONTEND_URL}/waiter`);
    await page.waitForLoadState('networkidle');
    // 給前端時間打 API
    await page.waitForTimeout(5000);

    // 印出來方便 debug
    console.log('Frontend API calls:', apiResponses.map(r => `${r.status} ${r.url}`));

    // 前端應該有打過 API
    expect(apiResponses.length).toBeGreaterThan(0);
  });
});

// ============================
// 8. WebSocket 狀態推送
// ============================
test.describe('WebSocket 狀態推送', () => {
  test('WebSocket 連線能收到狀態更新', async ({ page }) => {
    // 先導航到一個頁面，讓 page.evaluate 能正常運作
    await page.goto(FRONTEND_URL);
    await page.waitForLoadState('domcontentloaded');

    // 在瀏覽器中建立 WebSocket 連線
    const wsMessage = await page.evaluate(async () => {
      return new Promise<any>((resolve, reject) => {
        const ws = new WebSocket('ws://localhost:8000/ws/status');
        const timeout = setTimeout(() => {
          ws.close();
          reject(new Error('WS timeout after 20s'));
        }, 20000);

        ws.onmessage = (event) => {
          clearTimeout(timeout);
          const data = JSON.parse(event.data);
          ws.close();
          resolve(data);
        };

        ws.onerror = (e) => {
          clearTimeout(timeout);
          reject(new Error('WS connection error'));
        };
      });
    });

    expect(wsMessage).toHaveProperty('type');
    expect(wsMessage).toHaveProperty('data');
    expect(wsMessage.type).toBe('status_update');
    console.log('WS message type:', wsMessage.type);
  });
});
