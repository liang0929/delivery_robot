// 自動導航分頁：切 navigate 模式 → 重定位 → 選點位或點地圖導航 → 停止。

import { useCallback, useEffect, useRef, useState } from 'react';
import { MapCanvas, type MapInteraction } from '../components/MapCanvas';
import { useStoredMap } from '../hooks/useMapSource';
import { useAction } from '../hooks/useAction';
import { describeError, isAbortError } from '../api/client';
import {
  listPoints,
  listVirtualWalls,
  moveToLocation,
  moveToPoint,
  relocateToLocation,
  relocateToPoint,
  stop as stopRobot,
  switchMode,
} from '../api/robot.api';
import { useRobotStore } from '../store/useRobotStore';
import { pixelToApiLocation, type PixelPoint } from '../lib/coords';
import type { RobotPoint, VirtualWall } from '../api/types';
import page from './Page.module.css';

interface NavigationPageProps {
  maps: string[];
  selectedMap: string | null;
  onSelectMap: (name: string) => void;
  onReloadMaps: () => void;
}

type Tool = 'none' | 'relocate' | 'goto';

const TOOL_HINT: Record<Tool, string> = {
  none: '選擇下方工具，或直接從點位清單操作。',
  relocate: '在地圖上按下決定初始位置，拖曳決定朝向後放開。',
  goto: '在地圖上按下決定目標位置，拖曳決定抵達朝向後放開。',
};

export function NavigationPage({
  maps,
  selectedMap,
  onSelectMap,
  onReloadMaps,
}: NavigationPageProps) {
  const info = useRobotStore((s) => s.info);
  const robotLocation = info?.location ?? null;
  const opMode = info?.op_mode ?? null;
  const status = info?.status ?? null;

  const { busy, run } = useAction();
  const { image, meta, loading, error } = useStoredMap(selectedMap);

  const [points, setPoints] = useState<RobotPoint[]>([]);
  const [walls, setWalls] = useState<VirtualWall[]>([]);
  const [entitiesError, setEntitiesError] = useState<string | null>(null);
  const [selectedPointId, setSelectedPointId] = useState<string | null>(null);
  const [tool, setTool] = useState<Tool>('none');

  const requestId = useRef(0);

  useEffect(() => {
    if (!selectedMap) {
      requestId.current += 1;
      setPoints([]);
      setWalls([]);
      setEntitiesError(null);
      return;
    }
    const id = ++requestId.current;
    const controller = new AbortController();

    (async () => {
      try {
        const [nextPoints, nextWalls] = await Promise.all([
          listPoints(selectedMap, { signal: controller.signal }),
          listVirtualWalls(selectedMap, { signal: controller.signal }),
        ]);
        if (id !== requestId.current) return;
        setPoints(nextPoints);
        setWalls(nextWalls);
        setEntitiesError(null);
      } catch (err) {
        if (isAbortError(err) || id !== requestId.current) return;
        setPoints([]);
        setWalls([]);
        setEntitiesError(describeError(err));
      }
    })();

    return () => controller.abort();
  }, [selectedMap]);

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
    void run(
      '切換導航模式',
      () => switchMode('navigate', selectedMap),
      `已切換到導航模式（${selectedMap}）`,
    );
  };

  const handleRelocateToPoint = () => {
    if (!selectedPoint) return;
    void run(
      '以點位重定位',
      () => relocateToPoint(selectedPoint.id).then(() => undefined),
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
            emptyHint={
              !selectedMap
                ? '請先在右側選擇一張地圖'
                : loading
                  ? '地圖載入中…'
                  : (error ?? '地圖載入失敗')
            }
          />
        </div>
      </div>

      <div className={page.side}>
        <section className="panel">
          <h2 className="panelTitle">1. 地圖與模式</h2>
          <div className="row">
            <select
              className="select"
              style={{ flex: 1 }}
              value={selectedMap ?? ''}
              onChange={(e) => onSelectMap(e.target.value)}
            >
              <option value="">— 選擇地圖 —</option>
              {maps.map((m) => (
                <option key={m} value={m}>
                  {m}
                </option>
              ))}
            </select>
            <button type="button" className="btn small" onClick={onReloadMaps}>
              重新整理
            </button>
          </div>
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
            <button
              type="button"
              className="btn danger"
              disabled={busy !== null}
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
