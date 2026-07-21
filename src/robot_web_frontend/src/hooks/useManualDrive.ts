// 手動遙控。
//
// 「放開就停」的保證路徑（上一版的坑）——以下任一情況都會送 stop：
//   - pointerup / pointercancel / pointerleave（元件層）
//   - window 的 pointerup / pointercancel / touchcancel / touchend（指標跑出按鈕外）
//   - window blur（切換視窗、alt-tab）
//   - document visibilitychange 轉為 hidden（切分頁、鎖螢幕）
//   - pagehide / beforeunload（改用 sendBeacon，fetch 在卸載期不保證送出）
//   - React effect cleanup（元件卸載、切換分頁）
//
// 另外以 250ms 週期重送目前方向作為 keepalive，避免後端 cmd_vel timeout
// 造成「按著卻停住」；停止指令則永遠只送一次 stop。

import { useCallback, useEffect, useRef, useState } from 'react';
import { manualMove } from '../api/robot.api';
import { absoluteUrl, describeError } from '../api/client';
import { pushToast } from '../store/useRobotStore';
import type { ManualDirection } from '../api/types';

const KEEPALIVE_MS = 250;

export interface ManualDrive {
  /** 目前按住的方向，未按時為 null */
  active: ManualDirection | null;
  /** 按下：開始朝該方向移動 */
  press: (direction: ManualDirection) => void;
  /** 放開：送 stop（重複呼叫安全） */
  release: () => void;
  /** 是否啟用（非 explore 模式時關閉） */
  enabled: boolean;
}

/** 頁面卸載期間用 sendBeacon 送 stop，fetch 此時可能被瀏覽器丟棄 */
function beaconStop(): void {
  try {
    const body = new Blob([JSON.stringify({ direction: 'stop' })], {
      type: 'application/json',
    });
    navigator.sendBeacon?.(absoluteUrl('/manual/move'), body);
  } catch {
    /* 盡力而為 */
  }
}

export function useManualDrive(enabled: boolean): ManualDrive {
  const [active, setActive] = useState<ManualDirection | null>(null);
  const activeRef = useRef<ManualDirection | null>(null);
  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null);

  const send = useCallback((direction: ManualDirection) => {
    // 遙控指令刻意不阻塞 UI；失敗只提示一次，不讓 rejection 逸散
    manualMove(direction).catch((err) => {
      pushToast('error', '遙控指令失敗', describeError(err));
    });
  }, []);

  const clearTimer = useCallback(() => {
    if (timerRef.current !== null) {
      clearInterval(timerRef.current);
      timerRef.current = null;
    }
  }, []);

  const release = useCallback(() => {
    clearTimer();
    if (activeRef.current === null) return;
    activeRef.current = null;
    setActive(null);
    send('stop');
  }, [clearTimer, send]);

  const press = useCallback(
    (direction: ManualDirection) => {
      if (!enabled) return;
      if (direction === 'stop') {
        release();
        return;
      }
      if (activeRef.current === direction) return;
      activeRef.current = direction;
      setActive(direction);
      send(direction);
      clearTimer();
      timerRef.current = setInterval(() => {
        if (activeRef.current) send(activeRef.current);
      }, KEEPALIVE_MS);
    },
    [enabled, release, send, clearTimer],
  );

  // 全域保底：任何「離開」訊號都停
  useEffect(() => {
    const onVisibility = () => {
      if (document.visibilityState === 'hidden') release();
    };
    const onPageHide = () => {
      if (activeRef.current !== null) {
        clearTimer();
        activeRef.current = null;
        beaconStop();
      }
    };

    window.addEventListener('pointerup', release);
    window.addEventListener('pointercancel', release);
    window.addEventListener('touchend', release);
    window.addEventListener('touchcancel', release);
    window.addEventListener('blur', release);
    window.addEventListener('pagehide', onPageHide);
    window.addEventListener('beforeunload', onPageHide);
    document.addEventListener('visibilitychange', onVisibility);

    return () => {
      window.removeEventListener('pointerup', release);
      window.removeEventListener('pointercancel', release);
      window.removeEventListener('touchend', release);
      window.removeEventListener('touchcancel', release);
      window.removeEventListener('blur', release);
      window.removeEventListener('pagehide', onPageHide);
      window.removeEventListener('beforeunload', onPageHide);
      document.removeEventListener('visibilitychange', onVisibility);
    };
  }, [release, clearTimer]);

  // 卸載保底：切換分頁 / 離開建圖頁一定送 stop
  useEffect(
    () => () => {
      clearTimer();
      if (activeRef.current !== null) {
        activeRef.current = null;
        send('stop');
      }
    },
    [clearTimer, send],
  );

  // 功能被關閉（例如離開 explore 模式）時也要停
  useEffect(() => {
    if (!enabled) release();
  }, [enabled, release]);

  return { active, press, release, enabled };
}
