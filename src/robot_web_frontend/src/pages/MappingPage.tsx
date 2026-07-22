// 建圖分頁：切 explore 模式 → 遙控跑圖 → 觀察 live map 成長 → 存檔。

import { useState } from 'react';
import { MapCanvas } from '../components/MapCanvas';
import { DPad } from '../components/DPad';
import { useLiveMap } from '../hooks/useMapSource';
import { useManualDrive } from '../hooks/useManualDrive';
import { useAction } from '../hooks/useAction';
import { useRobotStore } from '../store/useRobotStore';
import { saveMap, switchMode } from '../api/robot.api';
import page from './Page.module.css';

interface MappingPageProps {
  /** 分頁是否為當前顯示中；非顯示時停止輪詢並確保遙控已停 */
  active: boolean;
  onMapSaved: (name: string) => void;
}

const NAME_PATTERN = /^[A-Za-z0-9_-]{1,64}$/;

export function MappingPage({ active, onMapSaved }: MappingPageProps) {
  const opMode = useRobotStore((s) => s.info?.op_mode ?? null);
  const location = useRobotStore((s) => s.info?.location ?? null);

  const [mapName, setMapName] = useState('');
  const { busy, run } = useAction();

  const isExplore = opMode === 'explore';
  // 只有在本分頁顯示且處於 explore 模式時才輪詢 live map
  const live = useLiveMap(active && isExplore);
  const drive = useManualDrive(active && isExplore);

  // explore 模式沒有定位問題，回應的 localized 欄位不適用，直接捨棄
  const handleSwitch = () =>
    void run(
      '切換建圖模式',
      () => switchMode('explore').then(() => undefined),
      '已切換到建圖模式',
    );

  const handleSave = () => {
    const name = mapName.trim();
    if (!NAME_PATTERN.test(name)) return;
    void run('儲存地圖', async () => {
      await saveMap(name);
      onMapSaved(name);
    }, `地圖「${name}」已儲存`);
  };

  const nameValid = NAME_PATTERN.test(mapName.trim());

  return (
    <div className={page.page}>
      <div className={page.mapArea}>
        <div className={page.mapBox}>
          <MapCanvas
            meta={live.meta}
            image={live.image}
            robot={location}
            badge={
              isExplore
                ? live.meta
                  ? `即時地圖 ${live.meta.width}×${live.meta.height} px @ ${live.meta.resolution} m/px`
                  : '等待地圖資料…'
                : undefined
            }
            emptyHint={
              isExplore
                ? live.error
                  ? `即時地圖尚未就緒：${live.error}`
                  : '正在等待建圖資料…'
                : '請先切換到建圖模式'
            }
          />
        </div>
      </div>

      <div className={page.side}>
        <section className="panel">
          <h2 className="panelTitle">1. 建圖模式</h2>
          <div className="row">
            <button
              type="button"
              className={`btn ${isExplore ? '' : 'primary'}`}
              onClick={handleSwitch}
              disabled={busy !== null}
            >
              {isExplore ? '重新啟動建圖' : '切換到建圖模式'}
            </button>
            {isExplore && <span className="tag">explore 執行中</span>}
          </div>
          <p className="hintText" style={{ marginTop: 8 }}>
            切換後 SLAM 會重新開始建圖，地圖以約 1.25 Hz 更新。
          </p>
        </section>

        <section className="panel">
          <h2 className="panelTitle">2. 遙控跑圖</h2>
          <DPad drive={drive} />
        </section>

        <section className="panel">
          <h2 className="panelTitle">3. 儲存地圖</h2>
          <div className="field">
            <label className="fieldLabel" htmlFor="map-name">
              地圖名稱（英數、底線、連字號）
            </label>
            <input
              id="map-name"
              className="input"
              value={mapName}
              placeholder="office_1f"
              onChange={(e) => setMapName(e.target.value)}
              onKeyDown={(e) => {
                if (e.key === 'Enter' && nameValid) handleSave();
              }}
            />
          </div>
          <div className="row" style={{ marginTop: 10 }}>
            <button
              type="button"
              className="btn primary"
              onClick={handleSave}
              disabled={!nameValid || !isExplore || busy !== null}
            >
              {busy === '儲存地圖' ? '儲存中…' : '儲存目前地圖'}
            </button>
          </div>
          {!isExplore && (
            <p className="hintText" style={{ marginTop: 8 }}>
              需在建圖模式下才能存檔。
            </p>
          )}
        </section>
      </div>
    </div>
  );
}
