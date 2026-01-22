import { useState, useEffect, useCallback } from 'react';
import { apiService, MapInfo, Waypoint } from '../../services/api.service';
import { useNavigationStatus } from '../../hooks/useStatusWs';
import { MapView } from '../map/MapView';
import { WaypointPanel, InteractionMode } from '../waypoint';
import styles from './NavigationPanel.module.css';

export function NavigationPanel() {
  const [goal, setGoal] = useState<{ x: number; y: number; yaw: number } | null>(null);
  const { navStatus } = useNavigationStatus();
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);
  const [maps, setMaps] = useState<MapInfo[]>([]);
  const [selectedMap, setSelectedMap] = useState<string>('');

  // Waypoint states
  const [interactionMode, setInteractionMode] = useState<InteractionMode>('navigate');
  const [pendingWaypoint, setPendingWaypoint] = useState<{ x: number; y: number; yaw: number } | null>(null);
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [selectedWaypointId, setSelectedWaypointId] = useState<string | null>(null);

  // 從 WebSocket 取得狀態，提供預設值
  const status = {
    is_complete: navStatus?.is_complete ?? true,
    distance_remaining: navStatus?.distance_remaining ?? null,
    nav_running: navStatus?.nav_running ?? false,
  };

  // Fetch available maps (只在載入時執行一次)
  useEffect(() => {
    const fetchMaps = async () => {
      try {
        const result = await apiService.getMaps();
        setMaps(result.maps);
        if (result.maps.length > 0) {
          const defaultMap = result.maps.find(m => m.name === result.default);
          setSelectedMap(defaultMap ? defaultMap.name : result.maps[0].name);
        }
      } catch (e) {
        console.error('Failed to fetch maps:', e);
      }
    };
    fetchMaps();
  }, []);

  const handleStartNavigation = async () => {
    if (!selectedMap) {
      setMessage({ type: 'error', text: 'Please select a map first' });
      return;
    }
    setLoading(true);
    setMessage(null);
    try {
      await apiService.startNavigation(selectedMap);
      setMessage({ type: 'success', text: `Navigation started with map: ${selectedMap}` });
    } catch (e: unknown) {
      const error = e as { response?: { data?: { detail?: string } } };
      setMessage({ type: 'error', text: error.response?.data?.detail || 'Failed to start navigation' });
    }
    setLoading(false);
  };

  const handleStopNavigation = async () => {
    setLoading(true);
    setMessage(null);
    try {
      await apiService.stopNavigation();
      setMessage({ type: 'success', text: 'Navigation mode stopped' });
      setGoal(null);
    } catch (e: unknown) {
      const error = e as { response?: { data?: { detail?: string } } };
      setMessage({ type: 'error', text: error.response?.data?.detail || 'Failed to stop navigation' });
    }
    setLoading(false);
  };

  const handleGoalSelect = useCallback((x: number, y: number, yaw: number) => {
    if (interactionMode === 'add_waypoint') {
      setPendingWaypoint({ x, y, yaw });
    } else {
      setGoal({ x, y, yaw });
    }
    setMessage(null);
  }, [interactionMode]);

  const handleNavigate = async () => {
    if (!goal) {
      setMessage({ type: 'error', text: 'Please click on the map to select a goal' });
      return;
    }

    setLoading(true);
    setMessage(null);
    try {
      await apiService.navigateToGoal({
        x: goal.x,
        y: goal.y,
        yaw_deg: goal.yaw * 180 / Math.PI,
      });
      setMessage({ type: 'success', text: 'Navigation started' });
    } catch (e: unknown) {
      const error = e as { response?: { data?: { detail?: string } } };
      setMessage({ type: 'error', text: error.response?.data?.detail || 'Failed to start navigation' });
    }
    setLoading(false);
  };

  const handleCancel = async () => {
    setLoading(true);
    setMessage(null);
    try {
      await apiService.cancelNavigation();
      setMessage({ type: 'success', text: 'Navigation cancelled' });
      setGoal(null);
    } catch (e: unknown) {
      const error = e as { response?: { data?: { detail?: string } } };
      setMessage({ type: 'error', text: error.response?.data?.detail || 'Failed to cancel navigation' });
    }
    setLoading(false);
  };

  return (
    <div className={styles.container}>
      <h3 className={styles.title}>Navigation</h3>

      <div className={styles.statusBar}>
        <div className={styles.statusItem}>
          <span className={styles.label}>Navigation:</span>
          <span className={`${styles.value} ${status.nav_running ? styles.active : styles.idle}`}>
            {status.nav_running ? 'RUNNING' : 'STOPPED'}
          </span>
        </div>
        <div className={styles.statusItem}>
          <span className={styles.label}>Status:</span>
          <span className={`${styles.value} ${status.is_complete ? styles.idle : styles.active}`}>
            {status.is_complete ? 'IDLE' : 'NAVIGATING'}
          </span>
        </div>
        {status.distance_remaining !== null && (
          <div className={styles.statusItem}>
            <span className={styles.label}>Distance:</span>
            <span className={styles.value}>{status.distance_remaining.toFixed(2)} m</span>
          </div>
        )}
      </div>

      <div className={styles.mapSelector}>
        <label className={styles.mapLabel}>Select Map:</label>
        <select
          className={styles.mapSelect}
          value={selectedMap}
          onChange={(e) => setSelectedMap(e.target.value)}
          disabled={status.nav_running || maps.length === 0}
        >
          {maps.length === 0 ? (
            <option value="">No maps available</option>
          ) : (
            maps.map((map) => (
              <option key={map.name} value={map.name}>
                {map.name}
              </option>
            ))
          )}
        </select>
      </div>

      <div className={styles.modeControls}>
        <button
          className={`${styles.button} ${styles.start}`}
          onClick={handleStartNavigation}
          disabled={loading || status.nav_running || !selectedMap}
        >
          Start Navigation
        </button>
        <button
          className={`${styles.button} ${styles.stop}`}
          onClick={handleStopNavigation}
          disabled={loading || !status.nav_running}
        >
          Stop Navigation
        </button>
      </div>

      <div className={styles.mapContainer}>
        <MapView
          onClickGoal={handleGoalSelect}
          showGoalSelector={status.nav_running === true}
          waypoints={waypoints}
          selectedWaypointId={selectedWaypointId}
          mode={interactionMode}
        />
      </div>

      <WaypointPanel
        mapName={selectedMap}
        navRunning={status.nav_running}
        mode={interactionMode}
        onModeChange={setInteractionMode}
        pendingWaypoint={pendingWaypoint}
        onWaypointAdded={() => setPendingWaypoint(null)}
        onWaypointsChange={setWaypoints}
        selectedWaypointId={selectedWaypointId}
        onSelectWaypoint={setSelectedWaypointId}
      />

      {interactionMode === 'navigate' && (
        <>
          {goal && (
            <div className={styles.goalInfo}>
              <span>Goal: ({goal.x.toFixed(2)}, {goal.y.toFixed(2)})</span>
            </div>
          )}

          <div className={styles.controls}>
            <button
              className={`${styles.button} ${styles.navigate}`}
              onClick={handleNavigate}
              disabled={loading || !goal || !status.nav_running}
            >
              Navigate to Goal
            </button>
            <button
              className={`${styles.button} ${styles.cancel}`}
              onClick={handleCancel}
              disabled={loading || status.is_complete || !status.nav_running}
            >
              Cancel
            </button>
          </div>
        </>
      )}

      {message && (
        <div className={`${styles.message} ${styles[message.type]}`}>
          {message.text}
        </div>
      )}
    </div>
  );
}
