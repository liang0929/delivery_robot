import { useState, useEffect } from 'react';
import { statusWsService, SystemStatus } from '../services/status-ws.service';

export function useStatusWs() {
  const [status, setStatus] = useState<SystemStatus | null>(null);
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    // 連線
    statusWsService.connect();

    // 訂閱狀態更新
    const unsubStatus = statusWsService.onStatusChange(setStatus);
    const unsubConn = statusWsService.onConnectionChange(setConnected);

    return () => {
      unsubStatus();
      unsubConn();
    };
  }, []);

  return { status, connected };
}

// 選擇性訂閱特定狀態
export function useNavigationStatus() {
  const { status, connected } = useStatusWs();
  return {
    navStatus: status?.navigation ?? null,
    connected,
  };
}

export function useSlamStatus() {
  const { status, connected } = useStatusWs();
  return {
    slamStatus: status?.slam ?? null,
    connected,
  };
}

export function useRobotCoreStatus() {
  const { status, connected } = useStatusWs();
  return {
    robotCoreStatus: status?.robot_core ?? null,
    crashInfo: status?.crash_info ?? null,
    connected,
  };
}
