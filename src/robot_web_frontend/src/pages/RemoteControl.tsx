import { VirtualJoystick } from '../components/joystick/VirtualJoystick';
import { MapView } from '../components/map/MapView';
import { RobotCorePanel } from '../components/robot/RobotCorePanel';
import styles from './RemoteControl.module.css';

export function RemoteControl() {
  return (
    <div className={styles.container}>
      <h2 className={styles.title}>Remote Control</h2>
      <div className={styles.content}>
        <div className={styles.mapSection}>
          <MapView />
        </div>
        <div className={styles.controlSection}>
          <RobotCorePanel />
          <VirtualJoystick />
          <div className={styles.instructions}>
            <h4>Instructions</h4>
            <ul>
              <li>Start robot core first</li>
              <li>Drag the joystick to move</li>
              <li>Up/Down: Forward/Backward</li>
              <li>Left/Right: Turn left/right</li>
              <li>Release to stop</li>
            </ul>
          </div>
        </div>
      </div>
    </div>
  );
}
