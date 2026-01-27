import { useEffect } from 'react';
import { useTableStore } from '../../stores/useTableStore';
import { useDeliveryStore } from '../../stores/useDeliveryStore';
import type { Table } from '../../types/table.types';
import styles from './TableSelector.module.css';

interface TableSelectorProps {
  mapName: string;
}

export function TableSelector({ mapName }: TableSelectorProps) {
  const { tables, isLoading, error, fetchTables } = useTableStore();
  const { stops, status, addStop } = useDeliveryStore();

  useEffect(() => {
    if (mapName) {
      fetchTables(mapName);
    }
  }, [mapName, fetchTables]);

  const activeTables = tables.filter((t) => t.isActive);

  const isTableSelected = (tableId: string) => {
    return stops.some((s) => s.tableId === tableId);
  };

  const handleTableClick = (table: Table) => {
    if (status !== 'idle') {
      return;
    }
    addStop(table);
  };

  if (isLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>Loading tables...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.container}>
        <div className={styles.error}>{error}</div>
      </div>
    );
  }

  if (activeTables.length === 0) {
    return (
      <div className={styles.container}>
        <div className={styles.empty}>
          <p>No tables configured</p>
          <p className={styles.hint}>Go to Admin &gt; Table Setup to add tables</p>
        </div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <h2 className={styles.title}>Select Tables</h2>
      <div className={styles.grid}>
        {activeTables
          .sort((a, b) => a.number - b.number)
          .map((table) => {
            const selected = isTableSelected(table.id);
            const disabled = status !== 'idle';

            return (
              <button
                key={table.id}
                className={`${styles.tableButton} ${selected ? styles.selected : ''} ${disabled ? styles.disabled : ''}`}
                onClick={() => handleTableClick(table)}
                disabled={disabled}
              >
                <span className={styles.tableNumber}>{table.number}</span>
                {table.name && <span className={styles.tableName}>{table.name}</span>}
              </button>
            );
          })}
      </div>
    </div>
  );
}
