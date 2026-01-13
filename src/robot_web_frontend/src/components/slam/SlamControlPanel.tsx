import { useState } from 'react';
import { apiService } from '../../services/api.service';
import { useSlamStatus } from '../../hooks/useStatusWs';
import styles from './SlamControlPanel.module.css';

export function SlamControlPanel() {
  const { slamStatus } = useSlamStatus();
  const [mapName, setMapName] = useState('');
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);

  // 從 WebSocket 取得狀態，提供預設值
  const status = {
    status: slamStatus?.status ?? 'idle',
    is_mapping: slamStatus?.is_mapping ?? false,
  };

  const handleStartMapping = async () => {
    setLoading(true);
    setMessage(null);
    try {
      await apiService.startMapping();
      setMessage({ type: 'success', text: 'Mapping started' });
    } catch (e: unknown) {
      const error = e as { response?: { data?: { detail?: string } } };
      setMessage({ type: 'error', text: error.response?.data?.detail || 'Failed to start mapping' });
    }
    setLoading(false);
  };

  const handleStopMapping = async () => {
    setLoading(true);
    setMessage(null);
    try {
      await apiService.stopMapping();
      setMessage({ type: 'success', text: 'Mapping stopped' });
    } catch (e: unknown) {
      const error = e as { response?: { data?: { detail?: string } } };
      setMessage({ type: 'error', text: error.response?.data?.detail || 'Failed to stop mapping' });
    }
    setLoading(false);
  };

  const handleSaveMap = async () => {
    if (!mapName.trim()) {
      setMessage({ type: 'error', text: 'Please enter a map name' });
      return;
    }

    setLoading(true);
    setMessage(null);
    try {
      const result = await apiService.saveMap(mapName.trim());
      setMessage({ type: 'success', text: `Map saved: ${result.map_path}` });
      setMapName('');
    } catch (e: unknown) {
      const error = e as { response?: { data?: { detail?: string } } };
      setMessage({ type: 'error', text: error.response?.data?.detail || 'Failed to save map' });
    }
    setLoading(false);
  };

  return (
    <div className={styles.container}>
      <h3 className={styles.title}>SLAM Control</h3>

      <div className={styles.statusBar}>
        <span className={styles.statusLabel}>Status:</span>
        <span className={`${styles.statusValue} ${styles[status.status]}`}>
          {status.status.toUpperCase()}
        </span>
      </div>

      <div className={styles.controls}>
        <button
          className={`${styles.button} ${styles.start}`}
          onClick={handleStartMapping}
          disabled={loading || status.is_mapping}
        >
          Start Mapping
        </button>
        <button
          className={`${styles.button} ${styles.stop}`}
          onClick={handleStopMapping}
          disabled={loading || !status.is_mapping}
        >
          Stop Mapping
        </button>
      </div>

      <div className={styles.saveSection}>
        <input
          type="text"
          className={styles.input}
          placeholder="Map name"
          value={mapName}
          onChange={(e) => setMapName(e.target.value)}
          disabled={loading}
        />
        <button
          className={`${styles.button} ${styles.save}`}
          onClick={handleSaveMap}
          disabled={loading || status.status === 'saving'}
        >
          Save Map
        </button>
      </div>

      {message && (
        <div className={`${styles.message} ${styles[message.type]}`}>
          {message.text}
        </div>
      )}
    </div>
  );
}
