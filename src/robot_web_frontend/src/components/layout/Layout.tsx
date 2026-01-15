import { ReactNode, useState, useEffect } from 'react';
import { Link, useLocation } from 'react-router-dom';
import { rosbridgeService } from '../../services/rosbridge.service';
import styles from './Layout.module.css';

interface LayoutProps {
  children: ReactNode;
}

export function Layout({ children }: LayoutProps) {
  const location = useLocation();
  const [connected, setConnected] = useState(false);

  useEffect(() => {
    const connect = async () => {
      try {
        await rosbridgeService.connect();
        setConnected(true);
      } catch (e) {
        console.error('Failed to connect to rosbridge:', e);
        setConnected(false);
      }
    };

    connect();

    return () => {
      rosbridgeService.disconnect();
    };
  }, []);

  const navItems = [
    { path: '/', label: 'Remote Control', icon: '🎮' },
    { path: '/slam', label: 'SLAM Mapping', icon: '🗺️' },
    { path: '/navigation', label: 'Navigation', icon: '📍' },
    { path: '/status', label: 'System Status', icon: '📊' },
  ];

  return (
    <div className={styles.container}>
      <nav className={styles.nav}>
        <div className={styles.logo}>Robot Control</div>
        <div className={styles.links}>
          {navItems.map((item) => (
            <Link
              key={item.path}
              to={item.path}
              className={`${styles.link} ${location.pathname === item.path ? styles.active : ''}`}
            >
              <span className={styles.icon}>{item.icon}</span>
              <span>{item.label}</span>
            </Link>
          ))}
        </div>
        <div className={`${styles.status} ${connected ? styles.connected : styles.disconnected}`}>
          {connected ? 'Connected' : 'Disconnected'}
        </div>
      </nav>
      <main className={styles.main}>{children}</main>
    </div>
  );
}
