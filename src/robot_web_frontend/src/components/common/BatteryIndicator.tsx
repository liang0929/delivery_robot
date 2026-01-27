import { useState, useEffect } from 'react';
import { rosbridgeService } from '../../services/rosbridge.service';
import styles from './BatteryIndicator.module.css';

// 電壓範圍（3S 鋰電池）
const VOLTAGE_MIN = 21.0; // 每節 7V
const VOLTAGE_MAX = 25.2; // 每節 8.4V
const VOLTAGE_LOW = 22.0; // 低電量警告

export function BatteryIndicator() {
  const [voltage, setVoltage] = useState<number | null>(null);

  useEffect(() => {
    rosbridgeService.subscribeToVoltage((v) => {
      setVoltage(v);
    });

    return () => {
      rosbridgeService.unsubscribeFromVoltage();
    };
  }, []);

  if (voltage === null) {
    return (
      <div className={styles.container}>
        <div className={styles.battery}>
          <div className={styles.level} style={{ width: '0%' }} />
        </div>
        <span className={styles.text}>--V</span>
      </div>
    );
  }

  // 計算電量百分比
  const percentage = Math.max(
    0,
    Math.min(100, ((voltage - VOLTAGE_MIN) / (VOLTAGE_MAX - VOLTAGE_MIN)) * 100)
  );

  const isLow = voltage < VOLTAGE_LOW;

  // 根據電量選擇顏色
  let colorClass = styles.high;
  if (percentage < 20) {
    colorClass = styles.critical;
  } else if (percentage < 50) {
    colorClass = styles.low;
  }

  return (
    <div className={`${styles.container} ${isLow ? styles.warning : ''}`}>
      <div className={styles.battery}>
        <div
          className={`${styles.level} ${colorClass}`}
          style={{ width: `${percentage}%` }}
        />
      </div>
      <span className={styles.text}>{voltage.toFixed(1)}V</span>
    </div>
  );
}
