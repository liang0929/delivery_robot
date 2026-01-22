import { useState } from 'react';
import styles from './WaypointPanel.module.css';

interface AddWaypointDialogProps {
  x: number;
  y: number;
  yaw: number;
  onSave: (name: string) => void;
  onCancel: () => void;
  loading: boolean;
}

export function AddWaypointDialog({
  x,
  y,
  yaw,
  onSave,
  onCancel,
  loading,
}: AddWaypointDialogProps) {
  const [name, setName] = useState('');

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (name.trim()) {
      onSave(name.trim());
    }
  };

  return (
    <div className={styles.dialogOverlay} onClick={onCancel}>
      <div className={styles.dialog} onClick={(e) => e.stopPropagation()}>
        <h3 className={styles.dialogTitle}>Add Waypoint</h3>
        <form onSubmit={handleSubmit}>
          <div className={styles.dialogField}>
            <label className={styles.dialogLabel}>Name</label>
            <input
              type="text"
              className={styles.dialogInput}
              value={name}
              onChange={(e) => setName(e.target.value)}
              placeholder="e.g. Table 1"
              autoFocus
            />
          </div>
          <div className={styles.dialogCoordRow}>
            <div className={`${styles.dialogField} ${styles.dialogCoordField}`}>
              <label className={styles.dialogLabel}>X (m)</label>
              <input
                type="text"
                className={styles.dialogInput}
                value={x.toFixed(2)}
                readOnly
              />
            </div>
            <div className={`${styles.dialogField} ${styles.dialogCoordField}`}>
              <label className={styles.dialogLabel}>Y (m)</label>
              <input
                type="text"
                className={styles.dialogInput}
                value={y.toFixed(2)}
                readOnly
              />
            </div>
            <div className={`${styles.dialogField} ${styles.dialogCoordField}`}>
              <label className={styles.dialogLabel}>Yaw (deg)</label>
              <input
                type="text"
                className={styles.dialogInput}
                value={(yaw * 180 / Math.PI).toFixed(1)}
                readOnly
              />
            </div>
          </div>
          <div className={styles.dialogActions}>
            <button
              type="button"
              className={`${styles.dialogButton} ${styles.dialogButtonSecondary}`}
              onClick={onCancel}
              disabled={loading}
            >
              Cancel
            </button>
            <button
              type="submit"
              className={`${styles.dialogButton} ${styles.dialogButtonPrimary}`}
              disabled={loading || !name.trim()}
            >
              {loading ? 'Saving...' : 'Save'}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
}
