// 連線設定
// ROBOT_IP 優先使用環境變數 VITE_ROBOT_IP，其次為當前頁面 hostname，最後 localhost。
const resolveRobotIp = (): string => {
  if (import.meta.env.VITE_ROBOT_IP) {
    return import.meta.env.VITE_ROBOT_IP;
  }
  if (typeof window !== 'undefined' && window.location.hostname) {
    return window.location.hostname;
  }
  return 'localhost';
};

const ROBOT_IP = resolveRobotIp();

/** Winstec Robot API v1.1：HTTP 5000 / WebSocket 5001 */
export const ROBOT_CONFIG = {
  ROBOT_IP,
  HTTP_PORT: 5000,
  WS_PORT: 5001,

  /** REST 前綴，例如 http://192.168.1.10:5000/v1/robot */
  API_BASE_URL: `http://${ROBOT_IP}:5000/v1/robot`,
  /** WebSocket 位址 */
  WS_URL: `ws://${ROBOT_IP}:5001`,

  /** 建圖時 live map 輪詢間隔（ms）→ 1.25 Hz */
  LIVE_MAP_POLL_MS: 800,
  /** WebSocket 斷線重連 backoff（ms） */
  WS_RECONNECT_MS: 3000,
} as const;
