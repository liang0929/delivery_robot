import { useState } from 'react';
import { Waypoint } from '../../services/api.service';
import styles from './WaypointPanel.module.css';

interface EditWaypointDialogProps {
  waypoint: Waypoint;
  onSave: (id: string, name: string, x: number, y: number, yaw_deg: number) => void;
  onCancel: () => void;
  loading: boolean;
}

export function EditWaypointDialog({
  waypoint,
  onSave,
  onCancel,
  loading,
}: EditWaypointDialogProps) {
  const [name, setName] = useState(waypoint.name);
  const [x, setX] = useState(waypoint.x.toString());
  const [y, setY] = useState(waypoint.y.toString());
  const [yawDeg, setYawDeg] = useState(waypoint.yaw_deg.toString());

  const handleSubmit = (e: React.FormEvent) => {
    e.preventDefault();
    if (name.trim()) {
      onSave(waypoint.id, name.trim(), parseFloat(x), parseFloat(y), parseFloat(yawDeg));
    }
  };

  return (
    <div className={styles.dialogOverlay} onClick={onCancel}>
      <div className={styles.dialog} onClick={(e) => e.stopPropagation()}>
        <h3 className={styles.dialogTitle}>Edit Waypoint</h3>
        <form onSubmit={handleSubmit}>
          <div className={styles.dialogField}>
            <label className={styles.dialogLabel}>Name</label>
            <input
              type="text"
              className={styles.dialogInput}
              value={name}
              onChange={(e) => setName(e.target.value)}
              autoFocus
            />
          </div>
          <div className={styles.dialogCoordRow}>
            <div className={`${styles.dialogField} ${styles.dialogCoordField}`}>
              <label className={styles.dialogLabel}>X (m)</label>
              <input
                type="number"
                step="0.01"
                className={styles.dialogInput}
                value={x}
                onChange={(e) => setX(e.target.value)}
              />
            </div>
            <div className={`${styles.dialogField} ${styles.dialogCoordField}`}>
              <label className={styles.dialogLabel}>Y (m)</label>
              <input
                type="number"
                step="0.01"
                className={styles.dialogInput}
                value={y}
                onChange={(e) => setY(e.target.value)}
              />
            </div>
            <div className={`${styles.dialogField} ${styles.dialogCoordField}`}>
              <label className={styles.dialogLabel}>Yaw (deg)</label>
              <input
                type="number"
                step="1"
                className={styles.dialogInput}
                value={yawDeg}
                onChange={(e) => setYawDeg(e.target.value)}
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
