// 頂部狀態列：WebSocket 連線狀態 + robot_info（op_mode / status / battery / location）。
// 只有這個元件訂閱整個 info，避免每秒一次的 robot_info 讓整棵樹重繪。

import { useRobotStore } from '../store/useRobotStore';
import { ROBOT_CONFIG } from '../config/robot.config';
import type { OpMode, RobotStatus } from '../api/types';
import styles from './StatusBar.module.css';

const MODE_LABEL: Record<OpMode, string> = {
  explore: '建圖 (explore)',
  navigate: '導航 (navigate)',
};

const STATUS_LABEL: Record<RobotStatus, string> = {
  init: '初始化中',
  idle: '閒置',
  relocating: '重定位中',
  moving: '導航中',
  go_charging: '前往充電座',
  switching_mode: '模式切換中',
};

const CONNECTION_LABEL = {
  open: '已連線',
  connecting: '連線中…',
  closed: '未連線',
} as const;

export function StatusBar() {
  const connection = useRobotStore((s) => s.connection);
  const info = useRobotStore((s) => s.info);

  const dotClass =
    connection === 'open'
      ? styles.dotOpen
      : connection === 'connecting'
        ? styles.dotConnecting
        : styles.dotClosed;

  // voltage 為 null 代表後端還沒收到 /motor/voltage。此時 battery 會是 0，
  // 直接畫成空電量條會被誤讀成沒電，因此一律顯示「—」。
  const voltage = info?.voltage ?? null;
  const battery = voltage !== null ? (info?.battery ?? null) : null;

  return (
    <div className={styles.bar}>
      <div className={styles.item}>
        <span className={`${styles.dot} ${dotClass}`} />
        <span className={styles.value}>{CONNECTION_LABEL[connection]}</span>
        <span className={styles.label}>
          {ROBOT_CONFIG.ROBOT_IP}:{ROBOT_CONFIG.WS_PORT}
        </span>
      </div>

      <div className={styles.item}>
        <span className={styles.label}>模式</span>
        <span className={styles.value}>
          {info ? MODE_LABEL[info.op_mode] ?? info.op_mode : '—'}
        </span>
      </div>

      <div className={styles.item}>
        <span className={styles.label}>狀態</span>
        <span className={styles.value}>
          {info ? STATUS_LABEL[info.status] ?? info.status : '—'}
        </span>
      </div>

      <div className={styles.item}>
        <span className={styles.label}>電量</span>
        <div className={styles.battery}>
          <div
            className={`${styles.batteryFill} ${
              battery !== null && battery <= 20 ? styles.batteryLow : ''
            }`}
            style={{ width: `${Math.max(0, Math.min(100, battery ?? 0))}%` }}
          />
        </div>
        <span className={styles.value}>
          {battery !== null && voltage !== null
            ? `${battery}% · ${voltage.toFixed(1)}V`
            : '—'}
        </span>
      </div>

      <div className={styles.item}>
        <span className={styles.label}>位置</span>
        <span className={styles.value}>
          {info
            ? `${info.location.x}, ${info.location.y} cm @ ${info.location.orientation.toFixed(0)}°`
            : '—'}
        </span>
      </div>
    </div>
  );
}
