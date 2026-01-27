import { useStatusWs } from '../hooks/useStatusWs';
import styles from './SystemStatus.module.css';

export function SystemStatus() {
  const { status, connected } = useStatusWs();

  const formatTime = (isoString: string | undefined) => {
    if (!isoString) return '-';
    const date = new Date(isoString);
    return date.toLocaleString('zh-TW');
  };

  return (
    <div className={styles.container}>
      <h2 className={styles.title}>System Status</h2>

      {/* WebSocket Connection */}
      <div className={styles.section}>
        <h3 className={styles.sectionTitle}>WebSocket Connection</h3>
        <div className={styles.statusRow}>
          <span className={styles.label}>Status</span>
          <span className={`${styles.badge} ${connected ? styles.online : styles.offline}`}>
            {connected ? 'CONNECTED' : 'DISCONNECTED'}
          </span>
        </div>
      </div>

      {/* SLAM */}
      <div className={styles.section}>
        <h3 className={styles.sectionTitle}>SLAM Mapping</h3>
        <div className={styles.statusRow}>
          <span className={styles.label}>Status</span>
          <span className={`${styles.badge} ${
            status?.slam?.status === 'mapping' ? styles.running :
            status?.slam?.status === 'saving' ? styles.warning :
            styles.idle
          }`}>
            {status?.slam?.status?.toUpperCase() || 'IDLE'}
          </span>
        </div>
        {status?.crash_info?.slam && (
          <div className={styles.crashInfo}>
            <span className={styles.crashLabel}>Last Crash</span>
            <span className={styles.crashDetail}>
              Exit code: {status.crash_info.slam.exit_code}
            </span>
            <span className={styles.crashTime}>
              {formatTime(status.crash_info.slam.time)}
            </span>
          </div>
        )}
      </div>

      {/* Navigation */}
      <div className={styles.section}>
        <h3 className={styles.sectionTitle}>Navigation</h3>
        <div className={styles.statusRow}>
          <span className={styles.label}>Nav2 Stack</span>
          <span className={`${styles.badge} ${status?.navigation?.nav_running ? styles.running : styles.stopped}`}>
            {status?.navigation?.nav_running ? 'RUNNING' : 'STOPPED'}
          </span>
        </div>
        <div className={styles.statusRow}>
          <span className={styles.label}>Goal Status</span>
          <span className={`${styles.badge} ${
            status?.navigation?.is_complete ? styles.idle : styles.running
          }`}>
            {status?.navigation?.is_complete ? 'IDLE' : 'NAVIGATING'}
          </span>
        </div>
        {status?.navigation?.distance_remaining !== null &&
         status?.navigation?.distance_remaining !== undefined && (
          <div className={styles.statusRow}>
            <span className={styles.label}>Distance Remaining</span>
            <span className={styles.value}>
              {status.navigation.distance_remaining.toFixed(2)} m
            </span>
          </div>
        )}
        {status?.crash_info?.navigation && (
          <div className={styles.crashInfo}>
            <span className={styles.crashLabel}>Last Crash</span>
            <span className={styles.crashDetail}>
              Exit code: {status.crash_info.navigation.exit_code}
            </span>
            <span className={styles.crashTime}>
              {formatTime(status.crash_info.navigation.time)}
            </span>
          </div>
        )}
      </div>

      {/* Last Update */}
      <div className={styles.footer}>
        {connected ? 'Real-time updates via WebSocket' : 'Waiting for connection...'}
      </div>
    </div>
  );
}
