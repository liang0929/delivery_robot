import { useState, useEffect, useCallback } from 'react';
import { apiService, Waypoint } from '../../services/api.service';
import { WaypointItem } from './WaypointItem';
import { AddWaypointDialog } from './AddWaypointDialog';
import { EditWaypointDialog } from './EditWaypointDialog';
import styles from './WaypointPanel.module.css';

export type InteractionMode = 'navigate' | 'add_waypoint';

interface WaypointPanelProps {
  mapName: string;
  navRunning: boolean;
  mode: InteractionMode;
  onModeChange: (mode: InteractionMode) => void;
  pendingWaypoint: { x: number; y: number; yaw: number } | null;
  onWaypointAdded: () => void;
  onWaypointsChange: (waypoints: Waypoint[]) => void;
  selectedWaypointId: string | null;
  onSelectWaypoint: (id: string | null) => void;
}

export function WaypointPanel({
  mapName,
  navRunning,
  mode,
  onModeChange,
  pendingWaypoint,
  onWaypointAdded,
  onWaypointsChange,
  selectedWaypointId,
  onSelectWaypoint,
}: WaypointPanelProps) {
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [loading, setLoading] = useState(false);
  const [showAddDialog, setShowAddDialog] = useState(false);
  const [editingWaypoint, setEditingWaypoint] = useState<Waypoint | null>(null);

  // Fetch waypoints
  const fetchWaypoints = useCallback(async () => {
    if (!mapName) return;
    try {
      const data = await apiService.getWaypoints(mapName);
      setWaypoints(data);
      onWaypointsChange(data);
    } catch (e) {
      console.error('Failed to fetch waypoints:', e);
    }
  }, [mapName, onWaypointsChange]);

  useEffect(() => {
    fetchWaypoints();
  }, [fetchWaypoints]);

  // Show add dialog when pending waypoint is set
  useEffect(() => {
    if (pendingWaypoint && mode === 'add_waypoint') {
      setShowAddDialog(true);
    }
  }, [pendingWaypoint, mode]);

  const handleNavigate = async (waypoint: Waypoint) => {
    if (!navRunning) return;
    setLoading(true);
    try {
      await apiService.navigateToWaypoint(mapName, waypoint.id);
    } catch (e) {
      console.error('Failed to navigate to waypoint:', e);
    }
    setLoading(false);
  };

  const handleDelete = async (waypoint: Waypoint) => {
    if (!confirm(`確定要刪除「${waypoint.name}」嗎？`)) return;
    setLoading(true);
    try {
      await apiService.deleteWaypoint(mapName, waypoint.id);
      await fetchWaypoints();
    } catch (e) {
      console.error('Failed to delete waypoint:', e);
    }
    setLoading(false);
  };

  const handleAddWaypoint = async (name: string) => {
    if (!pendingWaypoint) return;
    setLoading(true);
    try {
      await apiService.createWaypoint(mapName, {
        name,
        x: pendingWaypoint.x,
        y: pendingWaypoint.y,
        yaw_deg: pendingWaypoint.yaw * 180 / Math.PI,
      });
      await fetchWaypoints();
      setShowAddDialog(false);
      onWaypointAdded();
    } catch (e) {
      console.error('Failed to create waypoint:', e);
    }
    setLoading(false);
  };

  const handleUpdateWaypoint = async (id: string, name: string, x: number, y: number, yaw_deg: number) => {
    setLoading(true);
    try {
      await apiService.updateWaypoint(mapName, id, { name, x, y, yaw_deg });
      await fetchWaypoints();
      setEditingWaypoint(null);
    } catch (e) {
      console.error('Failed to update waypoint:', e);
    }
    setLoading(false);
  };

  const handleCancelAdd = () => {
    setShowAddDialog(false);
    onWaypointAdded();
  };

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h4 className={styles.title}>Waypoints</h4>
        <button
          className={styles.addButton}
          onClick={() => onModeChange('add_waypoint')}
          disabled={!navRunning || mode === 'add_waypoint'}
        >
          + Add
        </button>
      </div>

      <div className={styles.modeSelector}>
        <label className={`${styles.modeOption} ${mode === 'navigate' ? styles.active : ''}`}>
          <input
            type="radio"
            name="mode"
            checked={mode === 'navigate'}
            onChange={() => onModeChange('navigate')}
            disabled={!navRunning}
          />
          Navigate
        </label>
        <label className={`${styles.modeOption} ${mode === 'add_waypoint' ? styles.active : ''}`}>
          <input
            type="radio"
            name="mode"
            checked={mode === 'add_waypoint'}
            onChange={() => onModeChange('add_waypoint')}
            disabled={!navRunning}
          />
          Add Waypoint
        </label>
      </div>

      <div className={styles.waypointList}>
        {waypoints.length === 0 ? (
          <div className={styles.emptyMessage}>
            No waypoints. Click "Add" or select "Add Waypoint" mode to create one.
          </div>
        ) : (
          waypoints.map((wp) => (
            <WaypointItem
              key={wp.id}
              waypoint={wp}
              isSelected={selectedWaypointId === wp.id}
              disabled={loading || !navRunning}
              onNavigate={() => handleNavigate(wp)}
              onEdit={() => setEditingWaypoint(wp)}
              onDelete={() => handleDelete(wp)}
              onSelect={() => onSelectWaypoint(wp.id === selectedWaypointId ? null : wp.id)}
            />
          ))
        )}
      </div>

      {showAddDialog && pendingWaypoint && (
        <AddWaypointDialog
          x={pendingWaypoint.x}
          y={pendingWaypoint.y}
          yaw={pendingWaypoint.yaw}
          onSave={handleAddWaypoint}
          onCancel={handleCancelAdd}
          loading={loading}
        />
      )}

      {editingWaypoint && (
        <EditWaypointDialog
          waypoint={editingWaypoint}
          onSave={handleUpdateWaypoint}
          onCancel={() => setEditingWaypoint(null)}
          loading={loading}
        />
      )}
    </div>
  );
}
