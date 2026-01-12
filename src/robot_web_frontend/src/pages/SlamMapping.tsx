import { VirtualJoystick } from '../components/joystick/VirtualJoystick';
import { MapView } from '../components/map/MapView';
import { SlamControlPanel } from '../components/slam/SlamControlPanel';
import styles from './SlamMapping.module.css';

export function SlamMapping() {
  return (
    <div className={styles.container}>
      <h2 className={styles.title}>SLAM Mapping</h2>
      <div className={styles.content}>
        <div className={styles.mapSection}>
          <MapView />
        </div>
        <div className={styles.controlSection}>
          <SlamControlPanel />
          <VirtualJoystick />
        </div>
      </div>
      <div className={styles.instructions}>
        <h4>How to Create a Map</h4>
        <ol>
          <li>Click "Start Mapping" to begin SLAM</li>
          <li>Drive the robot around using the joystick</li>
          <li>The map will be displayed in real-time</li>
          <li>Enter a name and click "Save Map" when done</li>
          <li>Click "Stop Mapping" to end the session</li>
        </ol>
      </div>
    </div>
  );
}
