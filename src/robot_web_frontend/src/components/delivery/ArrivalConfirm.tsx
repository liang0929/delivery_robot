import { useDeliveryStore } from '../../stores/useDeliveryStore';
import styles from './ArrivalConfirm.module.css';

export function ArrivalConfirm() {
  const { status, stops, currentStopIndex, confirmArrival, skipTable, isLoading, error } =
    useDeliveryStore();

  // 只在到達桌位時顯示
  if (status !== 'at_table') {
    return null;
  }

  const currentStop = stops[currentStopIndex];

  const handleConfirm = async () => {
    try {
      await confirmArrival();
    } catch (err) {
      // 錯誤訊息已寫入 store.error，由 modal 內錯誤區塊顯示
      console.error('Failed to confirm arrival:', err);
    }
  };

  const handleSkip = async () => {
    try {
      await skipTable();
    } catch (err) {
      console.error('Failed to skip table:', err);
    }
  };

  return (
    <div className={styles.overlay}>
      <div className={styles.modal}>
        <div className={styles.icon}>Arrived</div>

        <h2 className={styles.title}>Table {currentStop?.tableNumber}</h2>

        {currentStop?.tableName && (
          <p className={styles.subtitle}>{currentStop.tableName}</p>
        )}

        <p className={styles.message}>Please serve the food and confirm when done.</p>

        {error && <div className={styles.error}>{error}</div>}

        <div className={styles.buttons}>
          <button
            className={styles.confirmButton}
            onClick={handleConfirm}
            disabled={isLoading}
          >
            {isLoading ? 'Processing...' : 'Confirm & Continue'}
          </button>

          <button
            className={styles.skipButton}
            onClick={handleSkip}
            disabled={isLoading}
          >
            Skip This Table
          </button>
        </div>

        <p className={styles.remainingInfo}>
          {stops.length - currentStopIndex - 1} tables remaining after this
        </p>
      </div>
    </div>
  );
}
