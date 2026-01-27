import { Link } from 'react-router-dom';
import { useStatusWs } from '../../hooks/useStatusWs';
import styles from './AdminDashboard.module.css';

export function AdminDashboard() {
  const { status, connected } = useStatusWs();

  const cards = [
    {
      title: 'Table Setup',
      description: 'Configure table positions on the map',
      path: '/admin/tables',
      status: null,
    },
    {
      title: 'SLAM Mapping',
      description: 'Create new maps of the environment',
      path: '/admin/slam',
      status: status?.slam?.status || 'idle',
    },
    {
      title: 'Remote Control',
      description: 'Manual robot control with joystick',
      path: '/admin/remote',
      status: null,
    },
    {
      title: 'System Status',
      description: 'Monitor system health and processes',
      path: '/admin/status',
      status: connected ? 'connected' : 'disconnected',
    },
  ];

  return (
    <div className={styles.container}>
      <h1 className={styles.title}>Admin Dashboard</h1>

      <div className={styles.grid}>
        {cards.map((card) => (
          <Link key={card.path} to={card.path} className={styles.card}>
            <h2 className={styles.cardTitle}>{card.title}</h2>
            <p className={styles.cardDescription}>{card.description}</p>
            {card.status && (
              <div className={`${styles.cardStatus} ${styles[card.status]}`}>
                {card.status}
              </div>
            )}
          </Link>
        ))}
      </div>

      <div className={styles.quickStats}>
        <h2 className={styles.sectionTitle}>System Overview</h2>
        <div className={styles.statsGrid}>
          <div className={styles.stat}>
            <span className={styles.statLabel}>ROS Bridge</span>
            <span className={`${styles.statValue} ${connected ? styles.online : styles.offline}`}>
              {connected ? 'Connected' : 'Disconnected'}
            </span>
          </div>
          <div className={styles.stat}>
            <span className={styles.statLabel}>Navigation</span>
            <span className={styles.statValue}>
              {status?.navigation?.status || 'Unknown'}
            </span>
          </div>
          <div className={styles.stat}>
            <span className={styles.statLabel}>SLAM</span>
            <span className={styles.statValue}>
              {status?.slam?.status || 'Unknown'}
            </span>
          </div>
        </div>
      </div>
    </div>
  );
}
