// 設定點位分頁：在已存檔地圖上建立 / 編輯 / 刪除點位與虛擬牆。
//
// 後端是暫存交易模型（規格 §4.2、§8）：所有變更先進記憶體，
// 必須按「提交變更」才會寫入磁碟並套用。UI 以黃色橫幅明示未提交狀態。
//
// 資料 / async CRUD 邏輯集中在 usePointsEditor；本檔只保留 UI 狀態
// （tool / draftPoint / draftWall / selectedId）與版面組裝。

import { useEffect, useState } from 'react';
import { MapCanvas, type MapInteraction } from '../components/MapCanvas';
import { MapPicker, mapEmptyHint } from '../components/MapPicker';
import { DirtyBanner } from '../components/points/DirtyBanner';
import { DraftPointPanel } from '../components/points/DraftPointPanel';
import { DraftWallPanel } from '../components/points/DraftWallPanel';
import { ToolPanel } from '../components/points/ToolPanel';
import { PointList } from '../components/points/PointList';
import { WallList } from '../components/points/WallList';
import { CommitPanel } from '../components/points/CommitPanel';
import { useStoredMap } from '../hooks/useMapSource';
import { usePointsEditor, type DraftPoint, type DraftWall, type Tool } from '../hooks/usePointsEditor';
import { useRobotStore } from '../store/useRobotStore';
import page from './Page.module.css';

const TOOL_HINT: Record<Tool, string> = {
  none: '從清單選取項目，或選擇上方工具開始編輯。',
  'add-point': '在地圖上按下決定位置，拖曳決定朝向後放開。',
  'add-wall': '在地圖上拖出一條線段作為虛擬牆。',
  'move-point': '在地圖上按下並拖曳，設定選取點位的新位姿。',
};

export function PointsPage() {
  const robotLocation = useRobotStore((s) => s.info?.location ?? null);
  const maps = useRobotStore((s) => s.maps);
  const selectedMap = useRobotStore((s) => s.selectedMap);
  const selectMap = useRobotStore((s) => s.selectMap);
  const loadMaps = useRobotStore((s) => s.loadMaps);

  const [mapVersion, setMapVersion] = useState(0);
  const { image, meta, loading, error } = useStoredMap(selectedMap, mapVersion);

  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [tool, setTool] = useState<Tool>('none');
  const [draftPoint, setDraftPoint] = useState<DraftPoint | null>(null);
  const [draftWall, setDraftWall] = useState<DraftWall | null>(null);

  // 切換地圖時把編輯狀態歸零，避免草稿套到別張地圖上
  useEffect(() => {
    setSelectedId(null);
    setTool('none');
    setDraftPoint(null);
    setDraftWall(null);
  }, [selectedMap]);

  const {
    points,
    walls,
    dirty,
    entitiesError,
    busy,
    selectedPoint,
    handlePose,
    handleSegment,
    createPoint,
    renamePoint,
    updateOrientation,
    deletePoint,
    createWall,
    deleteWall,
    commit,
    discard,
  } = usePointsEditor({
    selectedMap,
    meta,
    tool,
    draftPoint,
    draftWall,
    selectedId,
    setDraftPoint,
    setDraftWall,
    setSelectedId,
    setTool,
    onCommitted: () => setMapVersion((v) => v + 1),
  });

  // --------------------------------------------------------------- 地圖互動
  const interaction: MapInteraction =
    tool === 'add-wall' ? 'segment' : tool === 'none' ? 'none' : 'pose';

  const toggleTool = (next: Tool) => {
    setTool((prev) => (prev === next ? 'none' : next));
    setDraftPoint(null);
    setDraftWall(null);
  };

  return (
    <div className={page.page}>
      <div className={page.mapArea}>
        <DirtyBanner dirty={dirty} busy={busy} onCommit={commit} onDiscard={discard} />

        <div className={page.mapBox}>
          <MapCanvas
            meta={meta}
            image={image}
            points={points}
            walls={walls}
            robot={robotLocation}
            selectedId={selectedId}
            draftPose={draftPoint}
            interaction={interaction}
            onPose={handlePose}
            onSegment={handleSegment}
            badge={selectedMap ? `${selectedMap} · ${TOOL_HINT[tool]}` : undefined}
            emptyHint={mapEmptyHint(selectedMap, loading, error)}
          />
        </div>
      </div>

      <div className={page.side}>
        <section className="panel">
          <h2 className="panelTitle">地圖</h2>
          <MapPicker
            maps={maps}
            selectedMap={selectedMap}
            onSelectMap={selectMap}
            onReloadMaps={() => void loadMaps()}
          />
          {entitiesError && (
            <p className="hintText" style={{ marginTop: 8, color: '#ff9a9a' }}>
              點位資料載入失敗：{entitiesError}
            </p>
          )}
        </section>

        <ToolPanel
          tool={tool}
          toolHint={TOOL_HINT[tool]}
          selectedMap={selectedMap}
          meta={meta}
          selectedPoint={selectedPoint}
          onToggleTool={toggleTool}
        />

        {draftPoint && (
          <DraftPointPanel
            draftPoint={draftPoint}
            busy={busy}
            onChange={setDraftPoint}
            onCreate={createPoint}
            onCancel={() => setDraftPoint(null)}
          />
        )}

        {draftWall && (
          <DraftWallPanel
            draftWall={draftWall}
            busy={busy}
            onChange={setDraftWall}
            onCreate={createWall}
            onCancel={() => setDraftWall(null)}
          />
        )}

        <section className="panel">
          <h2 className="panelTitle">
            點位 <span className="tag">{points.length}</span>
          </h2>
          <PointList
            points={points}
            selectedId={selectedId}
            busy={busy}
            onSelect={(id) => setSelectedId(id === selectedId ? null : id)}
            onSave={renamePoint}
            onOrientation={updateOrientation}
            onDelete={deletePoint}
          />
        </section>

        <section className="panel">
          <h2 className="panelTitle">
            虛擬牆 <span className="tag">{walls.length}</span>
          </h2>
          <WallList
            walls={walls}
            selectedId={selectedId}
            busy={busy}
            onSelect={(id) => setSelectedId(id === selectedId ? null : id)}
            onDelete={deleteWall}
          />
        </section>

        <CommitPanel busy={busy} onCommit={commit} onDiscard={discard} />
      </div>
    </div>
  );
}
