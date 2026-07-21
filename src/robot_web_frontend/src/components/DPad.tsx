// 方向鍵盤：按住送方向、放開送 stop。
// 停止的保證邏輯在 hooks/useManualDrive.ts，這裡只負責事件轉發。
// 每顆按鈕都掛 pointerdown/up/cancel/leave，離開按鈕範圍即視為放開。

import { useEffect } from 'react';
import type { ManualDirection } from '../api/types';
import type { ManualDrive } from '../hooks/useManualDrive';
import styles from './DPad.module.css';

interface DPadProps {
  drive: ManualDrive;
  /** 是否啟用鍵盤方向鍵操作 */
  keyboard?: boolean;
}

const KEY_MAP: Record<string, ManualDirection> = {
  ArrowUp: 'forward',
  ArrowDown: 'backward',
  ArrowLeft: 'left',
  ArrowRight: 'right',
  w: 'forward',
  s: 'backward',
  a: 'left',
  d: 'right',
};

export function DPad({ drive, keyboard = true }: DPadProps) {
  const { active, press, release, enabled } = drive;

  useEffect(() => {
    if (!keyboard || !enabled) return;
    const onKeyDown = (ev: KeyboardEvent) => {
      if (ev.repeat) return;
      const dir = KEY_MAP[ev.key];
      if (!dir) return;
      ev.preventDefault();
      press(dir);
    };
    const onKeyUp = (ev: KeyboardEvent) => {
      if (KEY_MAP[ev.key]) release();
    };
    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);
    return () => {
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      release();
    };
  }, [keyboard, enabled, press, release]);

  const button = (
    dir: ManualDirection,
    area: string,
    label: string,
    title: string,
  ) => (
    <button
      type="button"
      title={title}
      disabled={!enabled}
      className={`${styles.btn} ${styles[area]} ${
        active === dir ? styles.active : ''
      }`}
      onPointerDown={(ev) => {
        ev.preventDefault();
        press(dir);
      }}
      onPointerUp={release}
      onPointerCancel={release}
      onPointerLeave={release}
      onContextMenu={(ev) => ev.preventDefault()}
    >
      {label}
    </button>
  );

  return (
    <div>
      <div className={styles.pad}>
        {button('forward', 'up', '▲', '前進 (W / ↑)')}
        {button('left', 'left', '◀', '左轉 (A / ←)')}
        <button
          type="button"
          title="立即停止"
          disabled={!enabled}
          className={`${styles.btn} ${styles.center} ${styles.stop}`}
          onPointerDown={(ev) => {
            ev.preventDefault();
            release();
          }}
        >
          STOP
        </button>
        {button('right', 'right', '▶', '右轉 (D / →)')}
        {button('backward', 'down', '▼', '後退 (S / ↓)')}
      </div>
      <div className={styles.caption}>
        {enabled ? '按住移動，放開自動停止（也可用 WASD / 方向鍵）' : '切換到建圖模式後才能遙控'}
      </div>
    </div>
  );
}
