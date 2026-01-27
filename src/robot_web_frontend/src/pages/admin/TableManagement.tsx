import { useState, useEffect } from 'react';
import { TableEditor } from '../../components/map/TableEditor';
import { apiService } from '../../services/api.service';
import styles from './TableManagement.module.css';

export function TableManagement() {
  const [selectedMap, setSelectedMap] = useState<string>('');
  const [maps, setMaps] = useState<string[]>([]);
  const [isLoading, setIsLoading] = useState(true);

  useEffect(() => {
    const loadMaps = async () => {
      try {
        const response = await apiService.getMaps();
        const mapNames = response.maps.map((m) => m.name);
        setMaps(mapNames);

        if (response.default && mapNames.includes(response.default)) {
          setSelectedMap(response.default);
        } else if (mapNames.length > 0) {
          setSelectedMap(mapNames[0]);
        }
      } catch (error) {
        console.error('Failed to load maps:', error);
      } finally {
        setIsLoading(false);
      }
    };

    loadMaps();
  }, []);

  if (isLoading) {
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

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h1 className={styles.title}>Table Setup</h1>

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
      </div>

      {selectedMap && (
        <div className={styles.editorContainer}>
          <TableEditor mapName={selectedMap} />
        </div>
      )}
    </div>
  );
}
