import { ReactNode, useState, useEffect } from 'react';
import { Link } from 'react-router-dom';
import { rosbridgeService } from '../../services/rosbridge.service';
import { BatteryIndicator } from '../common/BatteryIndicator';
import styles from './WaiterLayout.module.css';

interface WaiterLayoutProps {
  children: ReactNode;
}

export function WaiterLayout({ children }: WaiterLayoutProps) {
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const unsubscribe = rosbridgeService.onConnectionChange(setConnected);

    const connect = async () => {
      try {
        await rosbridgeService.connect();
      } catch (e) {
        console.error('Failed to connect to rosbridge:', e);
      }
    };

    connect();

    return () => {
      unsubscribe();
    };
  }, []);

  return (
    <div className={styles.container}>
      <nav className={styles.nav}>
        <Link to="/" className={styles.backButton}>
          Back
        </Link>
        <div className={styles.title}>Delivery Robot</div>
        <div className={styles.rightSection}>
          <BatteryIndicator />
          <div
            className={`${styles.status} ${connected ? styles.connected : styles.disconnected}`}
          >
            {connected ? 'Online' : 'Offline'}
          </div>
        </div>
      </nav>
      <main className={styles.main}>{children}</main>
    </div>
  );
}
