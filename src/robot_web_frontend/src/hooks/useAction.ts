// 非同步動作包裝：統一 try/catch、忙碌狀態與錯誤 toast。
// 所有 async handler 都應該走這裡，避免 unhandled rejection。

import { useCallback, useEffect, useRef, useState } from 'react';
import { describeError, isAbortError } from '../api/client';
import { pushToast } from '../store/useRobotStore';

/** 追蹤元件是否仍掛載，供非同步完成後的 setState 保護 */
export function useIsMounted(): { readonly current: boolean } {
  const mounted = useRef(true);
  useEffect(() => {
    mounted.current = true;
    return () => {
      mounted.current = false;
    };
  }, []);
  return mounted;
}

export interface ActionRunner {
  /** 目前執行中的動作標籤，未執行時為 null */
  busy: string | null;
  /**
   * 執行一個 async 動作。
   * @param label   顯示用標籤，同時作為 busy 識別
   * @param fn      實際動作
   * @param success 成功時要顯示的 toast 文字（可省略）
   * @returns 動作是否成功
   */
  run: (label: string, fn: () => Promise<void>, success?: string) => Promise<boolean>;
}

export function useAction(): ActionRunner {
  const mounted = useIsMounted();
  const [busy, setBusy] = useState<string | null>(null);

  const run = useCallback(
    async (label: string, fn: () => Promise<void>, success?: string) => {
      setBusy(label);
      try {
        await fn();
        if (success) pushToast('success', success);
        return true;
      } catch (err) {
        if (!isAbortError(err)) {
          pushToast('error', `${label}失敗`, describeError(err));
        }
        return false;
      } finally {
        if (mounted.current) setBusy(null);
      }
    },
    [mounted],
  );

  return { busy, run };
}
