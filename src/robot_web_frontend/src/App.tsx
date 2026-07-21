// 單頁應用外殼：三個分頁（建圖 / 設定點位 / 自動導航）+ 全域狀態列與提示。
//
// 分頁採「只掛載當前頁」策略：離開建圖頁時 MappingPage 卸載，
// useManualDrive 的 cleanup 會保證送出 stop。

import { useCallback, useEffect, useState } from 'react';
import { StatusBar } from './components/StatusBar';
import { Toasts } from './components/Toasts';
import { MappingPage } from './pages/MappingPage';
import { PointsPage } from './pages/PointsPage';
import { NavigationPage } from './pages/NavigationPage';
import { useMapList } from './hooks/useMapList';
import { useRobotStore } from './store/useRobotStore';
import styles from './App.module.css';

type Tab = 'mapping' | 'points' | 'navigation';

const TABS: { id: Tab; label: string }[] = [
  { id: 'mapping', label: '建圖' },
  { id: 'points', label: '設定點位' },
  { id: 'navigation', label: '自動導航' },
];

/** 瀏覽器分頁是否可見；隱藏時暫停輪詢 */
function usePageVisible(): boolean {
  const [visible, setVisible] = useState(
    typeof document === 'undefined' || document.visibilityState !== 'hidden',
  );
  useEffect(() => {
    const onChange = () => setVisible(document.visibilityState !== 'hidden');
    document.addEventListener('visibilitychange', onChange);
    return () => document.removeEventListener('visibilitychange', onChange);
  }, []);
  return visible;
}

export default function App() {
  const [tab, setTab] = useState<Tab>('mapping');
  const [selectedMap, setSelectedMap] = useState<string | null>(null);
  const visible = usePageVisible();

  const startSocket = useRobotStore((s) => s.startSocket);
  const stopSocket = useRobotStore((s) => s.stopSocket);
  const { maps, reload: reloadMaps } = useMapList();

  useEffect(() => {
    startSocket();
    return () => stopSocket();
  }, [startSocket, stopSocket]);

  // 地圖清單載入後，若尚未選過就自動選第一張
  useEffect(() => {
    setSelectedMap((prev) => (prev === null && maps.length > 0 ? maps[0] : prev));
  }, [maps]);

  const handleSelectMap = useCallback(
    (name: string) => setSelectedMap(name === '' ? null : name),
    [],
  );

  const handleMapSaved = useCallback(
    (name: string) => {
      reloadMaps();
      setSelectedMap(name);
    },
    [reloadMaps],
  );

  return (
    <div className={styles.app}>
      <header className={styles.header}>
        <span className={styles.brand}>AMR 控制台</span>
        <nav className={styles.tabs}>
          {TABS.map((t) => (
            <button
              key={t.id}
              type="button"
              className={`${styles.tab} ${tab === t.id ? styles.tabActive : ''}`}
              onClick={() => setTab(t.id)}
            >
              {t.label}
            </button>
          ))}
        </nav>
      </header>

      <StatusBar />

      <main className={styles.body}>
        {tab === 'mapping' && (
          <MappingPage active={visible} onMapSaved={handleMapSaved} />
        )}
        {tab === 'points' && (
          <PointsPage
            maps={maps}
            selectedMap={selectedMap}
            onSelectMap={handleSelectMap}
            onReloadMaps={reloadMaps}
          />
        )}
        {tab === 'navigation' && (
          <NavigationPage
            maps={maps}
            selectedMap={selectedMap}
            onSelectMap={handleSelectMap}
            onReloadMaps={reloadMaps}
          />
        )}
      </main>

      <Toasts />
    </div>
  );
}
