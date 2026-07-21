import { useEffect } from 'react';
import { useDeliveryStore } from '../../stores/useDeliveryStore';
import styles from './TaskProgress.module.css';

interface TaskProgressProps {
  onCancel?: () => void;
}

export function TaskProgress({ onCancel }: TaskProgressProps) {
  const {
    status,
    stops,
    currentStopIndex,
    distanceRemaining,
    error,
    isLoading,
    refreshStatus,
    cancelDelivery,
    retryDelivery,
  } = useDeliveryStore();

  // 定期刷新狀態
  useEffect(() => {
    if (status === 'idle') {
      return;
    }

    const interval = setInterval(() => {
      refreshStatus();
    }, 1000);

    return () => clearInterval(interval);
  }, [status, refreshStatus]);

  if (status === 'idle') {
    return null;
  }

  const currentStop = stops[currentStopIndex];
  const completedCount = stops.filter(
    (s) => s.status === 'completed' || s.status === 'skipped'
  ).length;
  const progress = (completedCount / stops.length) * 100;

  const handleCancel = async () => {
    try {
      await cancelDelivery();
      onCancel?.();
    } catch (err) {
      // 錯誤訊息已寫入 store.error，由下方錯誤區塊顯示
      console.error('Failed to cancel delivery:', err);
    }
  };

  const handleRetry = async () => {
    try {
      await retryDelivery();
    } catch (err) {
      console.error('Failed to retry delivery:', err);
    }
  };

  const getStatusText = () => {
    switch (status) {
      case 'delivering':
        return `Heading to Table ${currentStop?.tableNumber || '?'}`;
      case 'at_table':
        return `Arrived at Table ${currentStop?.tableNumber || '?'}`;
      case 'returning':
        return 'Returning to start position';
      case 'stuck':
        return 'Robot is stuck! Please clear the path.';
      default:
        return 'Processing...';
    }
  };

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h2 className={styles.title}>Delivery in Progress</h2>
        <button className={styles.cancelButton} onClick={handleCancel}>
          Cancel
        </button>
      </div>

      <div className={styles.statusSection}>
        <div className={`${styles.statusIcon} ${styles[status]}`}>
          {status === 'delivering' && 'Moving'}
          {status === 'at_table' && 'Arrived'}
          {status === 'returning' && 'Returning'}
          {status === 'stuck' && 'STUCK'}
        </div>
        <div className={styles.statusText}>{getStatusText()}</div>
        {status === 'stuck' && (
          <button
            className={styles.retryButton}
            onClick={handleRetry}
            disabled={isLoading}
          >
            {isLoading ? 'Retrying...' : 'Retry'}
          </button>
        )}
      </div>

      {error && <div className={styles.error}>{error}</div>}

      {distanceRemaining !== null && (
        <div className={styles.distance}>
          <span className={styles.distanceLabel}>Distance remaining:</span>
          <span className={styles.distanceValue}>{distanceRemaining.toFixed(2)} m</span>
        </div>
      )}

      <div className={styles.progressSection}>
        <div className={styles.progressLabel}>
          Progress: {completedCount} / {stops.length} tables
        </div>
        <div className={styles.progressBar}>
          <div className={styles.progressFill} style={{ width: `${progress}%` }} />
        </div>
      </div>

      <div className={styles.stopsPreview}>
        {stops.map((stop, index) => {
          const isCurrent = index === currentStopIndex;
          const isCompleted = stop.status === 'completed' || stop.status === 'skipped';

          return (
            <div
              key={stop.tableId}
              className={`${styles.stopDot} ${isCurrent ? styles.current : ''} ${isCompleted ? styles.completed : ''}`}
              title={`Table ${stop.tableNumber}`}
            >
              {stop.tableNumber}
            </div>
          );
        })}
      </div>
    </div>
  );
}
