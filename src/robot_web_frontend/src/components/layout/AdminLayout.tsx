import { ReactNode, useState, useEffect } from 'react';
import { Link, useLocation } from 'react-router-dom';
import { rosbridgeService } from '../../services/rosbridge.service';
import styles from './AdminLayout.module.css';

interface AdminLayoutProps {
  children: ReactNode;
}

export function AdminLayout({ children }: AdminLayoutProps) {
  const location = useLocation();
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

  const navItems = [
    { path: '/admin', label: 'Dashboard', exact: true },
    { path: '/admin/tables', label: 'Table Setup' },
    { path: '/admin/slam', label: 'SLAM Mapping' },
    { path: '/admin/remote', label: 'Remote Control' },
    { path: '/admin/status', label: 'System Status' },
  ];

  const isActive = (item: { path: string; exact?: boolean }) => {
    if (item.exact) {
      return location.pathname === item.path;
    }
    return location.pathname.startsWith(item.path);
  };

  return (
    <div className={styles.container}>
      <aside className={styles.sidebar}>
        <Link to="/" className={styles.logo}>
          Robot Control
        </Link>

        <nav className={styles.nav}>
          {navItems.map((item) => (
            <Link
              key={item.path}
              to={item.path}
              className={`${styles.navItem} ${isActive(item) ? styles.active : ''}`}
            >
              {item.label}
            </Link>
          ))}
        </nav>

        <div className={styles.statusSection}>
          <div
            className={`${styles.status} ${connected ? styles.connected : styles.disconnected}`}
          >
            {connected ? 'Connected' : 'Disconnected'}
          </div>
        </div>
      </aside>

      <main className={styles.main}>{children}</main>
    </div>
  );
}
