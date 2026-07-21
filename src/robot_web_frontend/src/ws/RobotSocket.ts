// WebSocket client（port 5001），含 3 秒 backoff 自動重連。
//
// 設計要點：
//   - 單一實例，由 store 在 App 掛載時 start()、卸載時 stop()
//   - 重連時不清掉最後一次已知狀態，僅標記 connected=false，讓 UI 顯示「連線中」
//   - 只解析 JSON 物件且含 `event` 欄位的訊息，其餘忽略

import { ROBOT_CONFIG } from '../config/robot.config';
import type { RobotSocketMessage } from '../api/types';

export type ConnectionState = 'connecting' | 'open' | 'closed';

interface Handlers {
  onMessage: (msg: RobotSocketMessage) => void;
  onStateChange: (state: ConnectionState) => void;
}

export class RobotSocket {
  private ws: WebSocket | null = null;
  private retryTimer: ReturnType<typeof setTimeout> | null = null;
  private stopped = true;

  constructor(private readonly handlers: Handlers) {}

  start(): void {
    if (!this.stopped) return;
    this.stopped = false;
    this.connect();
  }

  stop(): void {
    this.stopped = true;
    if (this.retryTimer !== null) {
      clearTimeout(this.retryTimer);
      this.retryTimer = null;
    }
    if (this.ws) {
      // 先解除 handler 再關閉，避免 onclose 觸發重連
      this.ws.onopen = null;
      this.ws.onclose = null;
      this.ws.onerror = null;
      this.ws.onmessage = null;
      try {
        this.ws.close();
      } catch {
        /* 忽略關閉時的例外 */
      }
      this.ws = null;
    }
    this.handlers.onStateChange('closed');
  }

  private connect(): void {
    if (this.stopped) return;
    this.handlers.onStateChange('connecting');

    let ws: WebSocket;
    try {
      ws = new WebSocket(ROBOT_CONFIG.WS_URL);
    } catch {
      this.scheduleReconnect();
      return;
    }
    this.ws = ws;

    ws.onopen = () => {
      if (this.stopped) return;
      this.handlers.onStateChange('open');
    };

    ws.onmessage = (ev) => {
      if (this.stopped) return;
      const msg = parseMessage(ev.data);
      if (msg) this.handlers.onMessage(msg);
    };

    ws.onerror = () => {
      // onerror 之後必定接 onclose，重連交給 onclose 處理
    };

    ws.onclose = () => {
      if (this.stopped) return;
      this.ws = null;
      this.handlers.onStateChange('connecting');
      this.scheduleReconnect();
    };
  }

  private scheduleReconnect(): void {
    if (this.stopped || this.retryTimer !== null) return;
    this.retryTimer = setTimeout(() => {
      this.retryTimer = null;
      this.connect();
    }, ROBOT_CONFIG.WS_RECONNECT_MS);
  }
}

function parseMessage(data: unknown): RobotSocketMessage | null {
  if (typeof data !== 'string') return null;
  try {
    const parsed: unknown = JSON.parse(data);
    if (
      parsed &&
      typeof parsed === 'object' &&
      typeof (parsed as { event?: unknown }).event === 'string'
    ) {
      return parsed as RobotSocketMessage;
    }
  } catch {
    /* 非 JSON，忽略 */
  }
  return null;
}
