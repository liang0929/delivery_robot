import { useNavigate } from 'react-router-dom';
import styles from './RoleSelect.module.css';

export function RoleSelect() {
  const navigate = useNavigate();

  return (
    <div className={styles.container}>
      <div className={styles.content}>
        <h1 className={styles.title}>Robot Control</h1>
        <p className={styles.subtitle}>Select your role to continue</p>

        <div className={styles.roleButtons}>
          <button
            className={`${styles.roleButton} ${styles.waiter}`}
            onClick={() => navigate('/waiter')}
          >
            <span className={styles.roleIcon}>Server</span>
            <span className={styles.roleName}>Waiter</span>
            <span className={styles.roleDesc}>Table selection and delivery</span>
          </button>

          <button
            className={`${styles.roleButton} ${styles.admin}`}
            onClick={() => navigate('/admin')}
          >
            <span className={styles.roleIcon}>Admin</span>
            <span className={styles.roleName}>Administrator</span>
            <span className={styles.roleDesc}>System settings and mapping</span>
          </button>
        </div>
      </div>
    </div>
  );
}
