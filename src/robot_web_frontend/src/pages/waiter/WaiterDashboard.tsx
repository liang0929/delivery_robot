import { useState, useEffect } from 'react';
import { TableSelector } from '../../components/delivery/TableSelector';
import { TaskQueue } from '../../components/delivery/TaskQueue';
import { TaskProgress } from '../../components/delivery/TaskProgress';
import { ArrivalConfirm } from '../../components/delivery/ArrivalConfirm';
import { useDeliveryStore } from '../../stores/useDeliveryStore';
import { apiService } from '../../services/api.service';
import styles from './WaiterDashboard.module.css';

export function WaiterDashboard() {
  const [selectedMap, setSelectedMap] = useState<string>('');
  const [maps, setMaps] = useState<string[]>([]);
  const [isLoadingMaps, setIsLoadingMaps] = useState(true);
  const { status } = useDeliveryStore();

  useEffect(() => {
    const loadMaps = async () => {
      try {
        const response = await apiService.getMaps();
        const mapNames = response.maps.map((m) => m.name);
        setMaps(mapNames);

        // 選擇預設地圖或第一個
        if (response.default && mapNames.includes(response.default)) {
          setSelectedMap(response.default);
        } else if (mapNames.length > 0) {
          setSelectedMap(mapNames[0]);
        }
      } catch (error) {
        console.error('Failed to load maps:', error);
      } finally {
        setIsLoadingMaps(false);
      }
    };

    loadMaps();
  }, []);

  if (isLoadingMaps) {
    return (
      <div className={styles.loading}>
        <p>Loading...</p>
      </div>
    );
  }

  if (maps.length === 0) {
    return (
      <div className={styles.noMaps}>
        <h2>No Maps Available</h2>
        <p>Please create a map using SLAM Mapping first.</p>
      </div>
    );
  }

  const isDelivering = status !== 'idle';

  return (
    <div className={styles.container}>
      {/* 地圖選擇器（只在空閒時顯示） */}
      {!isDelivering && maps.length > 1 && (
        <div className={styles.mapSelector}>
          <label>Map:</label>
          <select
            value={selectedMap}
            onChange={(e) => setSelectedMap(e.target.value)}
          >
            {maps.map((map) => (
              <option key={map} value={map}>
                {map}
              </option>
            ))}
          </select>
        </div>
      )}

      <div className={styles.content}>
        {/* 左側：桌號選擇或任務進度 */}
        <div className={styles.leftPanel}>
          {isDelivering ? (
            <TaskProgress />
          ) : (
            <TableSelector mapName={selectedMap} />
          )}
        </div>

        {/* 右側：任務隊列 */}
        <div className={styles.rightPanel}>
          <TaskQueue />
        </div>
      </div>

      {/* 到達確認彈窗 */}
      <ArrivalConfirm />
    </div>
  );
}
