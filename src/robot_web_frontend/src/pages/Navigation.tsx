import { NavigationPanel } from '../components/navigation/NavigationPanel';
import styles from './Navigation.module.css';

export function Navigation() {
  return (
    <div className={styles.container}>
      <h2 className={styles.title}>Autonomous Navigation</h2>
      <NavigationPanel />
      <div className={styles.instructions}>
        <h4>How to Navigate</h4>
        <ol>
          <li>Make sure autonomous navigation is running (ros2 launch nav2 autonomous_navigation.launch.py)</li>
          <li>Click on the map to select a destination</li>
          <li>Click "Navigate to Goal" to start navigation</li>
          <li>The robot will automatically plan and follow a path</li>
          <li>Click "Cancel" to stop navigation at any time</li>
        </ol>
      </div>
    </div>
  );
}
