import { useDeliveryStore } from '../../stores/useDeliveryStore';
import styles from './TaskQueue.module.css';

export function TaskQueue() {
  const { stops, status, currentStopIndex, removeStop, reorderStops, clearStops, startDelivery } =
    useDeliveryStore();

  const handleMoveUp = (index: number) => {
    if (index > 0) {
      reorderStops(index, index - 1);
    }
  };

  const handleMoveDown = (index: number) => {
    if (index < stops.length - 1) {
      reorderStops(index, index + 1);
    }
  };

  const isIdle = status === 'idle';

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h2 className={styles.title}>Delivery Queue</h2>
        {isIdle && stops.length > 0 && (
          <button className={styles.clearButton} onClick={clearStops}>
            Clear All
          </button>
        )}
      </div>

      {stops.length === 0 ? (
        <div className={styles.empty}>
          <p>No tables in queue</p>
          <p className={styles.hint}>Click tables on the left to add them</p>
        </div>
      ) : (
        <>
          <div className={styles.list}>
            {stops.map((stop, index) => {
              const isCurrent = !isIdle && index === currentStopIndex;
              const isCompleted = stop.status === 'completed' || stop.status === 'skipped';

              return (
                <div
                  key={stop.tableId}
                  className={`${styles.item} ${isCurrent ? styles.current : ''} ${isCompleted ? styles.completed : ''}`}
                >
                  <div className={styles.orderNumber}>{index + 1}</div>
                  <div className={styles.tableInfo}>
                    <span className={styles.tableNumber}>Table {stop.tableNumber}</span>
                    {stop.tableName && (
                      <span className={styles.tableName}>{stop.tableName}</span>
                    )}
                  </div>
                  <div className={styles.statusBadge}>
                    {stop.status === 'pending' && 'Pending'}
                    {stop.status === 'in_progress' && 'In Progress'}
                    {stop.status === 'arrived' && 'Arrived'}
                    {stop.status === 'completed' && 'Done'}
                    {stop.status === 'skipped' && 'Skipped'}
                  </div>
                  {isIdle && (
                    <div className={styles.actions}>
                      <button
                        className={styles.actionButton}
                        onClick={() => handleMoveUp(index)}
                        disabled={index === 0}
                      >
                        Up
                      </button>
                      <button
                        className={styles.actionButton}
                        onClick={() => handleMoveDown(index)}
                        disabled={index === stops.length - 1}
                      >
                        Down
                      </button>
                      <button
                        className={`${styles.actionButton} ${styles.remove}`}
                        onClick={() => removeStop(index)}
                      >
                        X
                      </button>
                    </div>
                  )}
                </div>
              );
            })}
          </div>

          {isIdle && (
            <button className={styles.startButton} onClick={startDelivery}>
              Start Delivery ({stops.length} tables)
            </button>
          )}
        </>
      )}
    </div>
  );
}
