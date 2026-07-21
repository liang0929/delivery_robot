// 事件 / 錯誤提示。WebSocket 事件（go_point、relocate、switch_mode …）
// 與所有 async handler 的錯誤都會走到這裡，不會變成 unhandled rejection。

import { useEffect } from 'react';
import { useRobotStore, type Toast } from '../store/useRobotStore';
import styles from './Toasts.module.css';

const AUTO_DISMISS_MS = 5000;

function ToastItem({ toast }: { toast: Toast }) {
  const dismiss = useRobotStore((s) => s.dismissToast);

  useEffect(() => {
    const timer = setTimeout(() => dismiss(toast.id), AUTO_DISMISS_MS);
    return () => clearTimeout(timer);
  }, [toast.id, dismiss]);

  return (
    <div
      className={`${styles.toast} ${styles[toast.level]}`}
      onClick={() => dismiss(toast.id)}
      role="status"
    >
      <div className={styles.title}>{toast.title}</div>
      {toast.detail && <div className={styles.detail}>{toast.detail}</div>}
    </div>
  );
}

export function Toasts() {
  const toasts = useRobotStore((s) => s.toasts);
  if (toasts.length === 0) return null;
  return (
    <div className={styles.stack}>
      {toasts.map((t) => (
        <ToastItem key={t.id} toast={t} />
      ))}
    </div>
  );
}
