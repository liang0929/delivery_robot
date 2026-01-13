import { useState, useEffect } from 'react';
import { rosbridgeService } from '../../services/rosbridge.service';
import styles from './RobotCorePanel.module.css';

export function RobotCorePanel() {
  const [voltage, setVoltage] = useState<number>(0);
  const [currentA, setCurrentA] = useState<number>(0);
  const [currentB, setCurrentB] = useState<number>(0);
  const [connected, setConnected] = useState(false);

  // Subscribe to motor data (with retry for connection)
  useEffect(() => {
    let retryInterval: number | null = null;

    const subscribe = () => {
      const isConnected = rosbridgeService.isConnected();
      setConnected(isConnected);

      if (isConnected) {
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

  return (
    <div className={styles.container}>
      <h3 className={styles.title}>Robot Status</h3>

      <div className={styles.statusBar}>
        <span className={styles.statusLabel}>Connection:</span>
        <span className={`${styles.statusValue} ${connected ? styles.running : styles.stopped}`}>
          {connected ? 'CONNECTED' : 'DISCONNECTED'}
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
    </div>
  );
}
