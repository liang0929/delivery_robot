import { useState, useEffect } from 'react';
import { apiService, RobotStatus } from '../../services/api.service';
import { rosbridgeService } from '../../services/rosbridge.service';
import styles from './RobotCorePanel.module.css';

export function RobotCorePanel() {
  const [status, setStatus] = useState<RobotStatus>({ is_running: false });
  const [loading, setLoading] = useState(false);
  const [message, setMessage] = useState<{ type: 'success' | 'error'; text: string } | null>(null);
  const [voltage, setVoltage] = useState<number>(0);
  const [currentA, setCurrentA] = useState<number>(0);
  const [currentB, setCurrentB] = useState<number>(0);

  // Poll status
  useEffect(() => {
    const fetchStatus = async () => {
      try {
        const s = await apiService.getRobotStatus();
        setStatus(s);
      } catch (e) {
        // API server might not be running
      }
    };

    fetchStatus();
    const interval = setInterval(fetchStatus, 2000);
    return () => clearInterval(interval);
  }, []);

  // Subscribe to motor data (with retry for connection)
  useEffect(() => {
    let retryInterval: number | null = null;

    const subscribe = () => {
      if (rosbridgeService.isConnected()) {
        rosbridgeService.subscribeToVoltage((v) => setVoltage(v));
        rosbridgeService.subscribeToCurrents((a, b) => {
          setCurrentA(a);
          setCurrentB(b);
        });
        if (retryInterval) {
          clearInterval(retryInterval);
          retryInterval = null;
        }
      }
    };

    // Try immediately, then retry every second until connected
    subscribe();
    retryInterval = window.setInterval(subscribe, 1000);

    return () => {
      if (retryInterval) clearInterval(retryInterval);
      rosbridgeService.unsubscribeFromVoltage();
      rosbridgeService.unsubscribeFromCurrents();
    };
  }, []);

  const handleStart = async () => {
    setLoading(true);
    setMessage(null);
    try {
      await apiService.startRobotCore();
      setMessage({ type: 'success', text: 'Robot core started!' });
    } catch (e: unknown) {
      const error = e as { response?: { data?: { detail?: string } } };
      setMessage({ type: 'error', text: error.response?.data?.detail || 'Failed to start' });
    }
    setLoading(false);
  };

  const handleStop = async () => {
    setLoading(true);
    setMessage(null);
    try {
      await apiService.stopRobotCore();
      setMessage({ type: 'success', text: 'Robot core stopped' });
    } catch (e: unknown) {
      const error = e as { response?: { data?: { detail?: string } } };
      setMessage({ type: 'error', text: error.response?.data?.detail || 'Failed to stop' });
    }
    setLoading(false);
  };

  return (
    <div className={styles.container}>
      <h3 className={styles.title}>Robot Core</h3>

      <div className={styles.statusBar}>
        <span className={styles.statusLabel}>Status:</span>
        <span className={`${styles.statusValue} ${status.is_running ? styles.running : styles.stopped}`}>
          {status.is_running ? 'RUNNING' : 'STOPPED'}
        </span>
      </div>

      <div className={styles.monitorGrid}>
        <div className={styles.monitorItem}>
          <span className={styles.monitorLabel}>Voltage</span>
          <span className={`${styles.monitorValue} ${voltage < 22 ? styles.warning : ''}`}>
            {voltage.toFixed(1)} V
          </span>
        </div>
        <div className={styles.monitorItem}>
          <span className={styles.monitorLabel}>Current A</span>
          <span className={styles.monitorValue}>{currentA.toFixed(2)} A</span>
        </div>
        <div className={styles.monitorItem}>
          <span className={styles.monitorLabel}>Current B</span>
          <span className={styles.monitorValue}>{currentB.toFixed(2)} A</span>
        </div>
      </div>

      <div className={styles.controls}>
        <button
          className={`${styles.button} ${styles.start}`}
          onClick={handleStart}
          disabled={loading || status.is_running}
        >
          Start
        </button>
        <button
          className={`${styles.button} ${styles.stop}`}
          onClick={handleStop}
          disabled={loading || !status.is_running}
        >
          Stop
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
