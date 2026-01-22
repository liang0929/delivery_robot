import { Waypoint } from '../../services/api.service';
import styles from './WaypointPanel.module.css';

interface WaypointItemProps {
  waypoint: Waypoint;
  isSelected: boolean;
  disabled: boolean;
  onNavigate: () => void;
  onEdit: () => void;
  onDelete: () => void;
  onSelect: () => void;
}

export function WaypointItem({
  waypoint,
  isSelected,
  disabled,
  onNavigate,
  onEdit,
  onDelete,
  onSelect,
}: WaypointItemProps) {
  return (
    <div
      className={`${styles.waypointItem} ${isSelected ? styles.selectedWaypoint : ''}`}
      onClick={onSelect}
    >
      <div className={styles.waypointInfo}>
        <p className={styles.waypointName}>{waypoint.name}</p>
        <span className={styles.waypointCoords}>
          ({waypoint.x.toFixed(2)}, {waypoint.y.toFixed(2)})
        </span>
      </div>
      <div className={styles.waypointActions}>
        <button
          className={`${styles.actionButton} ${styles.goButton}`}
          onClick={(e) => { e.stopPropagation(); onNavigate(); }}
          disabled={disabled}
        >
          Go
        </button>
        <button
          className={`${styles.actionButton} ${styles.editButton}`}
          onClick={(e) => { e.stopPropagation(); onEdit(); }}
          disabled={disabled}
        >
          Edit
        </button>
        <button
          className={`${styles.actionButton} ${styles.deleteButton}`}
          onClick={(e) => { e.stopPropagation(); onDelete(); }}
          disabled={disabled}
        >
          Del
        </button>
      </div>
    </div>
  );
}
