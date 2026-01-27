import { useDeliveryStore } from '../../stores/useDeliveryStore';
import styles from './ArrivalConfirm.module.css';

export function ArrivalConfirm() {
  const { status, stops, currentStopIndex, confirmArrival, skipTable, isLoading } =
    useDeliveryStore();

  // 只在到達桌位時顯示
  if (status !== 'at_table') {
    return null;
  }

  const currentStop = stops[currentStopIndex];

  const handleConfirm = async () => {
    await confirmArrival();
  };

  const handleSkip = async () => {
    await skipTable();
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
