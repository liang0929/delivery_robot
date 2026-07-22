// 極薄的 fetch 包裝：統一 base URL、JSON 編解碼與錯誤模型。
// 錯誤回應結構為 { "event": { "code": "..." } }（規格 §6，注意不是 "error"）。

import { ROBOT_CONFIG } from '../config/robot.config';
import type { ApiErrorBody } from './types';

export class ApiError extends Error {
  readonly status: number;
  readonly code: string;

  constructor(status: number, code: string, message?: string) {
    super(message ?? `${code} (HTTP ${status})`);
    this.name = 'ApiError';
    this.status = status;
    this.code = code;
  }
}

/** 把任意 throw 出來的東西轉成可顯示的字串 */
export function describeError(err: unknown): string {
  if (err instanceof ApiError) return `${err.code} (HTTP ${err.status})`;
  if (err instanceof DOMException && err.name === 'AbortError') return '請求已取消';
  if (err instanceof Error) return err.message;
  return String(err);
}

/** AbortError 通常是元件卸載或競態取消，UI 不該顯示 */
export function isAbortError(err: unknown): boolean {
  return err instanceof DOMException && err.name === 'AbortError';
}

/**
 * 預設請求逾時（毫秒）。
 *
 * 沒有逾時的話，後端若卡在等待（例如 Nav2 遲遲無法就緒），請求會無限期
 * 掛著，UI 的 busy 狀態也就永遠不會解除，按鈕全部卡在停用。
 */
const DEFAULT_TIMEOUT_MS = 15_000;

/** 模式切換要啟動整個 Nav2 或 SLAM，正常就需要數十秒 */
export const LONG_TIMEOUT_MS = 60_000;

interface RequestOptions {
  method?: 'GET' | 'POST' | 'PATCH' | 'DELETE';
  body?: unknown;
  query?: Record<string, string | number | undefined>;
  signal?: AbortSignal;
  /** 覆寫預設逾時；傳 0 表示不設逾時 */
  timeoutMs?: number;
}

/**
 * 把呼叫端的 signal 與逾時合併成單一 signal。
 * 回傳的 cleanup 必須在請求結束後呼叫，以免計時器洩漏。
 */
function withTimeout(
  signal: AbortSignal | undefined,
  timeoutMs: number,
): { signal: AbortSignal | undefined; cleanup: () => void; didTimeout: () => boolean } {
  if (!timeoutMs) return { signal, cleanup: () => {}, didTimeout: () => false };

  const ctrl = new AbortController();
  let timedOut = false;
  const timer = setTimeout(() => {
    timedOut = true;
    ctrl.abort();
  }, timeoutMs);

  const onOuterAbort = () => ctrl.abort();
  if (signal) {
    if (signal.aborted) ctrl.abort();
    else signal.addEventListener('abort', onOuterAbort);
  }

  return {
    signal: ctrl.signal,
    cleanup: () => {
      clearTimeout(timer);
      signal?.removeEventListener('abort', onOuterAbort);
    },
    didTimeout: () => timedOut,
  };
}

function buildUrl(path: string, query?: RequestOptions['query']): string {
  const url = new URL(ROBOT_CONFIG.API_BASE_URL + path);
  if (query) {
    for (const [k, v] of Object.entries(query)) {
      if (v !== undefined && v !== '') url.searchParams.set(k, String(v));
    }
  }
  return url.toString();
}

async function toApiError(res: Response): Promise<ApiError> {
  let code = `HTTP_${res.status}`;
  try {
    const body = (await res.json()) as ApiErrorBody;
    if (body?.event?.code) code = body.event.code;
  } catch {
    // 回應非 JSON，維持預設 code
  }
  return new ApiError(res.status, code);
}

export async function request<T>(
  path: string,
  opts: RequestOptions = {},
): Promise<T> {
  const { method = 'GET', body, query, signal, timeoutMs = DEFAULT_TIMEOUT_MS } = opts;
  const t = withTimeout(signal, timeoutMs);

  let res: Response;
  try {
    res = await fetch(buildUrl(path, query), {
      method,
      signal: t.signal,
      headers: body === undefined ? undefined : { 'Content-Type': 'application/json' },
      body: body === undefined ? undefined : JSON.stringify(body),
    });
  } catch (err) {
    // 逾時要能與「呼叫端主動取消」區分：後者不該顯示錯誤，前者必須顯示
    if (t.didTimeout()) {
      throw new ApiError(0, `REQUEST_TIMEOUT (${timeoutMs / 1000}s)`);
    }
    throw err;
  } finally {
    t.cleanup();
  }

  if (!res.ok) throw await toApiError(res);

  // 204 或空 body
  if (res.status === 204) return undefined as T;
  const text = await res.text();
  if (!text) return undefined as T;
  return JSON.parse(text) as T;
}

/** 取得二進位資源（地圖 PNG）為 object URL；呼叫端負責 revoke */
export async function requestBlobUrl(
  path: string,
  opts: { query?: RequestOptions['query']; signal?: AbortSignal } = {},
): Promise<string> {
  const res = await fetch(buildUrl(path, opts.query), { signal: opts.signal });
  if (!res.ok) throw await toApiError(res);
  return URL.createObjectURL(await res.blob());
}

/** 組出可直接放進 <img src> 的絕對網址 */
export function absoluteUrl(
  path: string,
  query?: RequestOptions['query'],
): string {
  return buildUrl(path, query);
}
