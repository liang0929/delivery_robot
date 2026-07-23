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
  const visible = usePageVisible();

  const startSocket = useRobotStore((s) => s.startSocket);
  const stopSocket = useRobotStore((s) => s.stopSocket);
  const loadMaps = useRobotStore((s) => s.loadMaps);
  const selectMap = useRobotStore((s) => s.selectMap);

  useEffect(() => {
    startSocket();
    return () => stopSocket();
  }, [startSocket, stopSocket]);

  useEffect(() => {
    void loadMaps();
  }, [loadMaps]);

  const handleMapSaved = useCallback(
    (name: string) => {
      void loadMaps();
      selectMap(name);
    },
    [loadMaps, selectMap],
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
        {tab === 'points' && <PointsPage />}
        {tab === 'navigation' && <NavigationPage />}
      </main>

      <Toasts />
    </div>
  );
}
