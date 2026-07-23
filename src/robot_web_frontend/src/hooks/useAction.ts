// 非同步動作包裝：統一 try/catch、忙碌狀態與錯誤 toast。
// 所有 async handler 都應該走這裡，避免 unhandled rejection。
//
// 不做 mounted-ref 卸載保護：React 18 起 setState 在卸載後呼叫已是安全的
// no-op（不會警告也不會出錯），加上該 pattern 官方本就勸退，故直接省略。

import { useCallback, useState } from 'react';
import { describeError, isAbortError } from '../api/client';
import { pushToast } from '../store/useRobotStore';

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
        setBusy(null);
      }
    },
    [],
  );

  return { busy, run };
}
