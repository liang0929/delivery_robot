import { useState, useEffect } from 'react';
import { StaticMapView } from './StaticMapView';
import { useTableStore } from '../../stores/useTableStore';
import type { Table, TableCreate } from '../../types/table.types';
import styles from './TableEditor.module.css';

interface TableEditorProps {
  mapName: string;
}

type EditorMode = 'view' | 'add' | 'edit';

export function TableEditor({ mapName }: TableEditorProps) {
  const {
    tables,
    isLoading,
    error,
    fetchTables,
    createTable,
    updateTable,
    deleteTable,
  } = useTableStore();

  const [mode, setMode] = useState<EditorMode>('view');
  const [selectedTable, setSelectedTable] = useState<Table | null>(null);
  const [pendingPosition, setPendingPosition] = useState<{ x: number; y: number; yaw: number } | null>(null);

  // Form state
  const [formNumber, setFormNumber] = useState<number>(1);
  const [formName, setFormName] = useState<string>('');
  const [formYaw, setFormYaw] = useState<number>(0);
  const [formActive, setFormActive] = useState<boolean>(true);

  useEffect(() => {
    if (mapName) {
      fetchTables(mapName);
    }
  }, [mapName, fetchTables]);

  // 計算下一個可用的桌號
  const getNextTableNumber = () => {
    if (tables.length === 0) return 1;
    const numbers = tables.map((t) => t.number);
    return Math.max(...numbers) + 1;
  };

  const handleMapClick = (x: number, y: number, yaw: number) => {
    if (mode === 'add') {
      setPendingPosition({ x, y, yaw });
      setFormNumber(getNextTableNumber());
      setFormName('');
      setFormYaw(0);
      setFormActive(true);
    }
  };

  const handleTableClick = (table: Table) => {
    setSelectedTable(table);
    setFormNumber(table.number);
    setFormName(table.name || '');
    setFormYaw(table.yaw_deg);
    setFormActive(table.isActive);
    setMode('edit');
  };

  const handleCreateTable = async () => {
    if (!pendingPosition) return;

    const data: TableCreate = {
      number: formNumber,
      name: formName || undefined,
      x: pendingPosition.x,
      y: pendingPosition.y,
      yaw_deg: formYaw,
      isActive: formActive,
    };

    try {
      await createTable(data);
      setPendingPosition(null);
      setMode('view');
    } catch (err) {
      console.error('Failed to create table:', err);
    }
  };

  const handleUpdateTable = async () => {
    if (!selectedTable) return;

    try {
      await updateTable(selectedTable.id, {
        number: formNumber,
        name: formName || undefined,
        yaw_deg: formYaw,
        isActive: formActive,
      });
      setSelectedTable(null);
      setMode('view');
    } catch (err) {
      console.error('Failed to update table:', err);
    }
  };

  const handleDeleteTable = async () => {
    if (!selectedTable) return;

    if (!confirm(`Delete Table ${selectedTable.number}?`)) return;

    try {
      await deleteTable(selectedTable.id);
      setSelectedTable(null);
      setMode('view');
    } catch (err) {
      console.error('Failed to delete table:', err);
    }
  };

  const handleCancel = () => {
    setPendingPosition(null);
    setSelectedTable(null);
    setMode('view');
  };

  return (
    <div className={styles.container}>
      <div className={styles.toolbar}>
        <div className={styles.modeButtons}>
          <button
            className={`${styles.modeButton} ${mode === 'view' ? styles.active : ''}`}
            onClick={() => { setMode('view'); setPendingPosition(null); setSelectedTable(null); }}
          >
            View
          </button>
          <button
            className={`${styles.modeButton} ${mode === 'add' ? styles.active : ''}`}
            onClick={() => { setMode('add'); setSelectedTable(null); }}
          >
            + Add Table
          </button>
        </div>

        {mode === 'add' && !pendingPosition && (
          <span className={styles.hint}>Click on the map to place a table</span>
        )}
      </div>

      <div className={styles.content}>
        <div className={styles.mapContainer}>
          <StaticMapView
            mapName={mapName}
            tables={tables}
            selectedTableId={selectedTable?.id}
            mode={mode === 'add' ? 'add_table' : mode === 'edit' ? 'edit_table' : 'view'}
            onMapClick={handleMapClick}
            onTableClick={handleTableClick}
          />
        </div>

        <div className={styles.sidebar}>
          {/* Table Form */}
          {(pendingPosition || selectedTable) && (
            <div className={styles.form}>
              <h3 className={styles.formTitle}>
                {pendingPosition ? 'New Table' : `Edit Table ${selectedTable?.number}`}
              </h3>

              <div className={styles.formField}>
                <label>Table Number</label>
                <input
                  type="number"
                  value={formNumber}
                  onChange={(e) => setFormNumber(parseInt(e.target.value) || 1)}
                  min={1}
                />
              </div>

              <div className={styles.formField}>
                <label>Name (optional)</label>
                <input
                  type="text"
                  value={formName}
                  onChange={(e) => setFormName(e.target.value)}
                  placeholder="e.g., VIP Table"
                />
              </div>

              <div className={styles.formField}>
                <label>Approach Angle (deg)</label>
                <input
                  type="number"
                  value={formYaw}
                  onChange={(e) => setFormYaw(parseFloat(e.target.value) || 0)}
                  step={15}
                />
              </div>

              <div className={styles.formField}>
                <label className={styles.checkbox}>
                  <input
                    type="checkbox"
                    checked={formActive}
                    onChange={(e) => setFormActive(e.target.checked)}
                  />
                  Active (available for delivery)
                </label>
              </div>

              <div className={styles.formField}>
                <label>Position</label>
                <div className={styles.positionInfo}>
                  X: {(pendingPosition?.x ?? selectedTable?.x ?? 0).toFixed(2)}m,
                  Y: {(pendingPosition?.y ?? selectedTable?.y ?? 0).toFixed(2)}m
                </div>
              </div>

              <div className={styles.formActions}>
                {pendingPosition ? (
                  <button
                    className={styles.primaryButton}
                    onClick={handleCreateTable}
                    disabled={isLoading}
                  >
                    {isLoading ? 'Creating...' : 'Create Table'}
                  </button>
                ) : (
                  <>
                    <button
                      className={styles.primaryButton}
                      onClick={handleUpdateTable}
                      disabled={isLoading}
                    >
                      {isLoading ? 'Saving...' : 'Save Changes'}
                    </button>
                    <button
                      className={styles.dangerButton}
                      onClick={handleDeleteTable}
                      disabled={isLoading}
                    >
                      Delete
                    </button>
                  </>
                )}
                <button className={styles.secondaryButton} onClick={handleCancel}>
                  Cancel
                </button>
              </div>
            </div>
          )}

          {/* Table List */}
          <div className={styles.tableList}>
            <h3 className={styles.listTitle}>Tables ({tables.length})</h3>
            {tables.length === 0 ? (
              <p className={styles.emptyList}>No tables configured</p>
            ) : (
              <ul className={styles.list}>
                {tables
                  .sort((a, b) => a.number - b.number)
                  .map((table) => (
                    <li
                      key={table.id}
                      className={`${styles.listItem} ${!table.isActive ? styles.inactive : ''} ${selectedTable?.id === table.id ? styles.selected : ''}`}
                      onClick={() => handleTableClick(table)}
                    >
                      <span className={styles.listNumber}>{table.number}</span>
                      <span className={styles.listName}>
                        {table.name || `Table ${table.number}`}
                      </span>
                      {!table.isActive && (
                        <span className={styles.inactiveBadge}>Inactive</span>
                      )}
                    </li>
                  ))}
              </ul>
            )}
          </div>

          {error && <div className={styles.error}>{error}</div>}
        </div>
      </div>
    </div>
  );
}
