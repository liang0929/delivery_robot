// 自動導航分頁：切 navigate 模式 → 重定位 → 選點位或點地圖導航 → 停止。

import { useCallback, useEffect, useState } from 'react';
import { MapCanvas, type MapInteraction } from '../components/MapCanvas';
import { MapPicker, mapEmptyHint } from '../components/MapPicker';
import { useStoredMap } from '../hooks/useMapSource';
import { useMapEntities } from '../hooks/useMapEntities';
import { useAction } from '../hooks/useAction';
import {
  moveToLocation,
  moveToPoint,
  relocateToLocation,
  relocateToPoint,
  stop as stopRobot,
  switchMode,
} from '../api/robot.api';
import { pushToast, useRobotStore } from '../store/useRobotStore';
import { pixelToApiLocation, type PixelPoint } from '../lib/coords';
import type { RobotPoint } from '../api/types';
import page from './Page.module.css';

type Tool = 'none' | 'relocate' | 'goto';

const TOOL_HINT: Record<Tool, string> = {
  none: '選擇下方工具，或直接從點位清單操作。',
  relocate: '在地圖上按下決定初始位置，拖曳決定朝向後放開。',
  goto: '在地圖上按下決定目標位置，拖曳決定抵達朝向後放開。',
};

export function NavigationPage() {
  const info = useRobotStore((s) => s.info);
  const robotLocation = info?.location ?? null;
  const opMode = info?.op_mode ?? null;
  const status = info?.status ?? null;

  const maps = useRobotStore((s) => s.maps);
  const selectedMap = useRobotStore((s) => s.selectedMap);
  const selectMap = useRobotStore((s) => s.selectMap);
  const loadMaps = useRobotStore((s) => s.loadMaps);

  const { busy, run } = useAction();
  const { image, meta, loading, error } = useStoredMap(selectedMap);

  const { points, walls, error: entitiesError } = useMapEntities(selectedMap);
  const [selectedPointId, setSelectedPointId] = useState<string | null>(null);
  const [tool, setTool] = useState<Tool>('none');
  /**
   * 切到導航模式後 AMCL 是否已定位。
   * null = 尚未嘗試切換；false = 自動定位失敗，需要手動指定位置。
   */
  const [localized, setLocalized] = useState<boolean | null>(null);
  const [localizeDetail, setLocalizeDetail] = useState<string | null>(null);

  useEffect(() => {
    setSelectedPointId(null);
    setTool('none');
  }, [selectedMap]);

  const isNavigate = opMode === 'navigate';
  const selectedPoint = points.find((p) => p.id === selectedPointId) ?? null;

  const handlePose = useCallback(
    (px: PixelPoint, deg: number) => {
      if (!meta) return;
      const location = pixelToApiLocation(px, deg, meta);
      if (tool === 'relocate') {
        void run(
          '設定初始位姿',
          async () => {
            await relocateToLocation(location);
            setTool('none');
            // 手動指定後視為已定位，清掉自動定位失敗的提示
            setLocalized(true);
            setLocalizeDetail(null);
          },
          '已送出重定位',
        );
      } else if (tool === 'goto') {
        void run(
          '導航到指定位置',
          async () => {
            await moveToLocation(location);
            setTool('none');
          },
          '已送出導航目標',
        );
      }
    },
    [meta, tool, run],
  );

  const handleSwitchMode = () => {
    if (!selectedMap) return;
    void run('切換導航模式', async () => {
      const res = await switchMode('navigate', selectedMap);
      // 後端會先嘗試以地圖原點自動定位；成功與否決定使用者要不要手動指定
      const ok = res.localized !== false;
      setLocalized(ok);
      setLocalizeDetail(ok ? null : (res.detail ?? null));
      if (ok) {
        pushToast('success', `已切換到導航模式（${selectedMap}）`);
      } else {
        pushToast(
          'error',
          '導航模式已啟動，但尚未完成定位',
          '請在下方「重定位」指定機器人在地圖上的實際位置',
        );
      }
    });
  };

  const handleRelocateToPoint = () => {
    if (!selectedPoint) return;
    void run(
      '以點位重定位',
      async () => {
        await relocateToPoint(selectedPoint.id);
        // 手動指定後視為已定位，清掉自動定位失敗的提示
        setLocalized(true);
        setLocalizeDetail(null);
      },
      `已以「${selectedPoint.name}」重定位`,
    );
  };

  const handleGoToPoint = (point: RobotPoint) => {
    void run(
      '導航到點位',
      () => moveToPoint(point.id).then(() => undefined),
      `已送出導航：${point.name}`,
    );
  };

  const handleStop = () => void run('停止', () => stopRobot(), '已送出停止指令');

  const interaction: MapInteraction = tool === 'none' ? 'none' : 'pose';

  return (
    <div className={page.page}>
      <div className={page.mapArea}>
        <div className={page.mapBox}>
          <MapCanvas
            meta={meta}
            image={image}
            points={points}
            walls={walls}
            robot={robotLocation}
            selectedId={selectedPointId}
            interaction={interaction}
            onPose={handlePose}
            badge={selectedMap ? `${selectedMap} · ${TOOL_HINT[tool]}` : undefined}
            emptyHint={mapEmptyHint(selectedMap, loading, error)}
          />
        </div>
      </div>

      <div className={page.side}>
        <section className="panel">
          <h2 className="panelTitle">1. 地圖與模式</h2>
          <MapPicker
            maps={maps}
            selectedMap={selectedMap}
            onSelectMap={selectMap}
            onReloadMaps={() => void loadMaps()}
          />
          <div className="row" style={{ marginTop: 10 }}>
            <button
              type="button"
              className={`btn ${isNavigate ? '' : 'primary'}`}
              onClick={handleSwitchMode}
              disabled={!selectedMap || busy !== null}
            >
              {isNavigate ? '重新載入導航模式' : '切換到導航模式'}
            </button>
            {isNavigate && <span className="tag">navigate 執行中</span>}
          </div>
          {entitiesError && (
            <p className="hintText" style={{ marginTop: 8, color: '#ff9a9a' }}>
              點位資料載入失敗：{entitiesError}
            </p>
          )}
        </section>

        <section className="panel">
          <h2 className="panelTitle">2. 重定位</h2>
          {localized === false && (
            <div className={page.warnBanner}>
              <strong>尚未完成定位，導航無法執行</strong>
              <p>
                系統已嘗試以地圖原點（建圖起點）自動定位但未成功。
                請用下方任一方式指定機器人在地圖上的<b>實際位置</b>。
              </p>
              {localizeDetail && <p className={page.warnDetail}>{localizeDetail}</p>}
            </div>
          )}
          <div className="row">
            <button
              type="button"
              className={`btn ${tool === 'relocate' ? 'active' : ''}`}
              disabled={!meta || busy !== null}
              onClick={() =>
                setTool((prev) => (prev === 'relocate' ? 'none' : 'relocate'))
              }
            >
              在地圖上設定初始位姿
            </button>
            <button
              type="button"
              className="btn"
              disabled={!selectedPoint || busy !== null}
              onClick={handleRelocateToPoint}
            >
              以選取點位重定位
            </button>
          </div>
          <p className="hintText" style={{ marginTop: 8 }}>
            {selectedPoint
              ? `已選取點位：${selectedPoint.name}`
              : '從下方清單選一個點位，即可用它的位姿重定位。'}
          </p>
        </section>

        <section className="panel">
          <h2 className="panelTitle">3. 導航</h2>
          <div className="row">
            <button
              type="button"
              className={`btn ${tool === 'goto' ? 'active' : ''}`}
              disabled={!meta || !isNavigate || busy !== null}
              onClick={() => setTool((prev) => (prev === 'goto' ? 'none' : 'goto'))}
            >
              點地圖導航
            </button>
            <button
              type="button"
              className="btn primary"
              disabled={!selectedPoint || !isNavigate || busy !== null}
              onClick={() => selectedPoint && handleGoToPoint(selectedPoint)}
            >
              前往選取點位
            </button>
            {/* 停止是安全控制項，任何情況下都不得停用——包含其他請求進行中、
                或模式尚未切換完成。使用者必須永遠能夠中止機器人。 */}
            <button
              type="button"
              className="btn danger"
              onClick={handleStop}
            >
              停止
            </button>
          </div>
          {!isNavigate && (
            <p className="hintText" style={{ marginTop: 8 }}>
              需先切換到導航模式才能下達導航指令。
            </p>
          )}
          {status === 'moving' && (
            <p className="hintText" style={{ marginTop: 8, color: '#3ddc84' }}>
              導航中…
            </p>
          )}
        </section>

        <section className="panel">
          <h2 className="panelTitle">
            點位 <span className="tag">{points.length}</span>
          </h2>
          {points.length === 0 ? (
            <p className="hintText">這張地圖尚未設定點位。</p>
          ) : (
            <ul className="list">
              {points.map((p) => (
                <li
                  key={p.id}
                  className={`listItem ${p.id === selectedPointId ? 'selected' : ''}`}
                  onClick={() =>
                    setSelectedPointId(p.id === selectedPointId ? null : p.id)
                  }
                >
                  <div className="listItemMain">
                    <div className="listItemName">
                      {p.name}{' '}
                      {p.type === 'charge' && (
                        <span className="tag charge">charge</span>
                      )}
                    </div>
                    <div className="listItemMeta">
                      {p.location.x}, {p.location.y} cm @{' '}
                      {Math.round(p.location.orientation)}°
                    </div>
                  </div>
                  <button
                    type="button"
                    className="btn small primary"
                    disabled={!isNavigate || busy !== null}
                    onClick={(e) => {
                      e.stopPropagation();
                      handleGoToPoint(p);
                    }}
                  >
                    前往
                  </button>
                </li>
              ))}
            </ul>
          )}
        </section>
      </div>
    </div>
  );
}
