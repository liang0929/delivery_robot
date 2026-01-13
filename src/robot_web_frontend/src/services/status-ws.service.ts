import { ROBOT_CONFIG } from '../config/robot.config';

export interface RobotCoreStatus {
  running: boolean;
}

export interface SlamStatusData {
  status: 'idle' | 'mapping' | 'saving';
  is_mapping: boolean;
}

export interface NavigationStatusData {
  status: 'idle' | 'running';
  nav_running: boolean;
  is_complete: boolean;
  distance_remaining: number | null;
}

export interface CrashInfo {
  exit_code: number;
  time: string;
}

export interface SystemStatus {
  robot_core: RobotCoreStatus;
  slam: SlamStatusData;
  navigation: NavigationStatusData;
  crash_info: {
    robot_core?: CrashInfo;
    slam?: CrashInfo;
    navigation?: CrashInfo;
  };
}

type StatusCallback = (status: SystemStatus) => void;
type ConnectionCallback = (connected: boolean) => void;

class StatusWebSocketService {
  private ws: WebSocket | null = null;
  private statusCallbacks: Set<StatusCallback> = new Set();
  private connectionCallbacks: Set<ConnectionCallback> = new Set();
  private reconnectTimer: number | null = null;
  private pingTimer: number | null = null;
  private connected = false;
  private lastStatus: SystemStatus | null = null;

  private get wsUrl(): string {
    return `ws://${ROBOT_CONFIG.ROBOT_IP}:${ROBOT_CONFIG.API_PORT}/ws/status`;
  }

  connect(): void {
    if (this.ws?.readyState === WebSocket.OPEN) {
      return;
    }

    this.cleanup();

    try {
      this.ws = new WebSocket(this.wsUrl);

      this.ws.onopen = () => {
        console.log('Status WebSocket connected');
        this.connected = true;
        this.notifyConnectionChange(true);
        this.startPing();
      };

      this.ws.onmessage = (event) => {
        try {
          const message = JSON.parse(event.data);
          if (message.type === 'status_update' && message.data) {
            this.lastStatus = message.data;
            this.notifyStatusChange(message.data);
          }
        } catch (e) {
          // 忽略非 JSON 訊息 (如 pong)
        }
      };

      this.ws.onclose = () => {
        console.log('Status WebSocket disconnected');
        this.connected = false;
        this.notifyConnectionChange(false);
        this.cleanup();
        this.scheduleReconnect();
      };

      this.ws.onerror = (error) => {
        console.error('Status WebSocket error:', error);
      };
    } catch (e) {
      console.error('Failed to create WebSocket:', e);
      this.scheduleReconnect();
    }
  }

  disconnect(): void {
    this.cleanup();
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  private cleanup(): void {
    if (this.pingTimer) {
      clearInterval(this.pingTimer);
      this.pingTimer = null;
    }
    if (this.ws) {
      this.ws.onopen = null;
      this.ws.onmessage = null;
      this.ws.onclose = null;
      this.ws.onerror = null;
      if (this.ws.readyState === WebSocket.OPEN) {
        this.ws.close();
      }
      this.ws = null;
    }
  }

  private startPing(): void {
    // 每 30 秒發送 ping 保持連線
    this.pingTimer = window.setInterval(() => {
      if (this.ws?.readyState === WebSocket.OPEN) {
        this.ws.send('ping');
      }
    }, 30000);
  }

  private scheduleReconnect(): void {
    if (this.reconnectTimer) return;
    // 3 秒後重連
    this.reconnectTimer = window.setTimeout(() => {
      this.reconnectTimer = null;
      this.connect();
    }, 3000);
  }

  private notifyStatusChange(status: SystemStatus): void {
    this.statusCallbacks.forEach((cb) => cb(status));
  }

  private notifyConnectionChange(connected: boolean): void {
    this.connectionCallbacks.forEach((cb) => cb(connected));
  }

  // --- Public API ---
  onStatusChange(callback: StatusCallback): () => void {
    this.statusCallbacks.add(callback);
    // 如果已有最新狀態，立即通知
    if (this.lastStatus) {
      callback(this.lastStatus);
    }
    // 回傳取消訂閱函數
    return () => this.statusCallbacks.delete(callback);
  }

  onConnectionChange(callback: ConnectionCallback): () => void {
    this.connectionCallbacks.add(callback);
    callback(this.connected);
    return () => this.connectionCallbacks.delete(callback);
  }

  isConnected(): boolean {
    return this.connected;
  }

  getLastStatus(): SystemStatus | null {
    return this.lastStatus;
  }
}

export const statusWsService = new StatusWebSocketService();
