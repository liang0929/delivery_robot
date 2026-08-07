// 頂部狀態列：WebSocket 連線狀態 + robot_info（op_mode / status / battery / location）。
// 只有這個元件訂閱整個 info，避免每秒一次的 robot_info 讓整棵樹重繪。

import { useRobotStore } from '../store/useRobotStore';
import { ROBOT_CONFIG } from '../config/robot.config';
import type { BatteryState, OpMode, RobotStatus } from '../api/types';
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

// 低電壓保護（battery_guard）。ok 不顯示——電池正常是常態，不佔版面也不製造
// 視覺噪音；其餘三態各自有 badge。shutdown 必須寫出解除方式，否則操作者只會
// 看到「機器人不動了」而不知道能做什麼。
const BATTERY_STATE_BADGE: Record<
  Exclude<BatteryState, 'ok'>,
  { className: string; text: string; title: string }
> = {
  warning: {
    className: 'protectWarning',
    text: '⚠ 電池低電壓警告',
    title: '電池電壓已低於警告門檻，請儘快前往充電。',
  },
  shutdown: {
    className: 'protectShutdown',
    text: '🔴 低電壓停機 · 充電後重啟解除',
    title:
      'battery_guard 已鎖存停機命令，馬達不會動作。' +
      '鎖存設計上只能由「充電後重啟 battery_guard」解除，前端無法解除。',
  },
  unknown: {
    className: 'protectUnknown',
    text: '電池保護狀態未知',
    title: '尚未收到 /battery/state，battery_guard 可能未啟動。',
  },
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

  // 鎖存優先：只要 stop_latched 就當成停機顯示。正常情況下 shutdown 必然鎖存，
  // 但反過來若上游只給了旗標沒給狀態，寧可顯示得嚴重一點也不要漏報。
  const protectState: BatteryState | null = info
    ? info.battery_stop_latched
      ? 'shutdown'
      : info.battery_state
    : null;
  const protectBadge =
    protectState !== null && protectState !== 'ok'
      ? BATTERY_STATE_BADGE[protectState]
      : null;

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

      {protectBadge && (
        <div className={styles.item}>
          <span
            className={`${styles.protectBadge} ${styles[protectBadge.className]}`}
            title={protectBadge.title}
            data-battery-state={protectState}
          >
            {protectBadge.text}
          </span>
        </div>
      )}

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
