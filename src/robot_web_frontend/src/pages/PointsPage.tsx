// 設定點位分頁：在已存檔地圖上建立 / 編輯 / 刪除點位與虛擬牆。
//
// 後端是暫存交易模型（規格 §4.2、§8）：所有變更先進記憶體，
// 必須按「提交變更」才會寫入磁碟並套用。UI 以黃色橫幅明示未提交狀態。

import { useCallback, useEffect, useState } from 'react';
import { MapCanvas, type MapInteraction } from '../components/MapCanvas';
import { MapPicker, mapEmptyHint } from '../components/MapPicker';
import { useStoredMap } from '../hooks/useMapSource';
import { useMapEntities } from '../hooks/useMapEntities';
import { useAction } from '../hooks/useAction';
import {
  commitEdits,
  createPoint,
  createVirtualWall,
  deletePoint,
  deleteVirtualWall,
  discardEdits,
  updatePoint,
} from '../api/robot.api';
import { useRobotStore } from '../store/useRobotStore';
import {
  normalizeDegrees,
  pixelToApi,
  pixelToApiLocation,
  type PixelPoint,
} from '../lib/coords';
import type {
  ApiLocation,
  ApiPosition,
  PointType,
  RobotPoint,
  VirtualWall,
} from '../api/types';
import page from './Page.module.css';

type Tool = 'none' | 'add-point' | 'add-wall' | 'move-point';

interface DraftPoint {
  location: ApiLocation;
  name: string;
  type: PointType;
}

interface DraftWall {
  start: ApiPosition;
  end: ApiPosition;
  name: string;
}

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
  const { busy, run } = useAction();

  const [mapVersion, setMapVersion] = useState(0);
  const { image, meta, loading, error } = useStoredMap(selectedMap, mapVersion);

  const {
    points,
    walls,
    error: entitiesError,
    reload: reloadEntities,
    setPoints,
    setWalls,
  } = useMapEntities(selectedMap);

  const [selectedId, setSelectedId] = useState<string | null>(null);
  const [tool, setTool] = useState<Tool>('none');
  const [draftPoint, setDraftPoint] = useState<DraftPoint | null>(null);
  const [draftWall, setDraftWall] = useState<DraftWall | null>(null);
  const [dirty, setDirty] = useState(false);

  // 切換地圖時把編輯狀態歸零，避免草稿套到別張地圖上
  useEffect(() => {
    setSelectedId(null);
    setTool('none');
    setDraftPoint(null);
    setDraftWall(null);
  }, [selectedMap]);

  const selectedPoint = points.find((p) => p.id === selectedId) ?? null;

  // --------------------------------------------------------------- 地圖互動
  const interaction: MapInteraction =
    tool === 'add-wall' ? 'segment' : tool === 'none' ? 'none' : 'pose';

  const handlePose = useCallback(
    (px: PixelPoint, deg: number) => {
      if (!meta) return;
      const location = pixelToApiLocation(px, deg, meta);
      if (tool === 'add-point') {
        setDraftPoint({ location, name: '', type: 'point' });
        setSelectedId(null);
      } else if (tool === 'move-point' && selectedPoint) {
        void run(
          '更新點位位置',
          async () => {
            const updated = await updatePoint(selectedPoint, { location });
            setPoints((prev) =>
              prev.map((p) => (p.id === selectedPoint.id ? updated : p)),
            );
            setDirty(true);
            setTool('none');
          },
          '點位位置已更新（尚未提交）',
        );
      }
    },
    [meta, tool, selectedPoint, run],
  );

  const handleSegment = useCallback(
    (start: PixelPoint, end: PixelPoint) => {
      if (!meta) return;
      setDraftWall({
        start: pixelToApi(start, meta),
        end: pixelToApi(end, meta),
        name: '',
      });
    },
    [meta],
  );

  // --------------------------------------------------------------- 動作處理
  const handleCreatePoint = () => {
    if (!draftPoint || !selectedMap) return;
    const name = draftPoint.name.trim();
    if (!name) return;
    void run(
      '建立點位',
      async () => {
        const created = await createPoint({
          map: selectedMap,
          name,
          type: draftPoint.type,
          location: draftPoint.location,
        });
        setPoints((prev) => [...prev, created]);
        setDraftPoint(null);
        setTool('none');
        setDirty(true);
        reloadEntities();
      },
      `點位「${name}」已建立（尚未提交）`,
    );
  };

  const handleRenamePoint = (point: RobotPoint, name: string, type: PointType) => {
    const trimmed = name.trim();
    if (!trimmed) return;
    void run(
      '更新點位',
      async () => {
        const updated = await updatePoint(point, { name: trimmed, type });
        setPoints((prev) => prev.map((p) => (p.id === point.id ? updated : p)));
        setDirty(true);
      },
      '點位已更新（尚未提交）',
    );
  };

  const handleUpdateOrientation = (point: RobotPoint, orientation: number) => {
    const location: ApiLocation = {
      ...point.location,
      orientation: normalizeDegrees(orientation),
    };
    void run(
      '更新朝向',
      async () => {
        const updated = await updatePoint(point, { location });
        setPoints((prev) => prev.map((p) => (p.id === point.id ? updated : p)));
        setDirty(true);
      },
      '朝向已更新（尚未提交）',
    );
  };

  const handleDeletePoint = (point: RobotPoint) => {
    void run(
      '刪除點位',
      async () => {
        await deletePoint(point.id, selectedMap ?? undefined);
        setPoints((prev) => prev.filter((p) => p.id !== point.id));
        if (selectedId === point.id) setSelectedId(null);
        setDirty(true);
      },
      `點位「${point.name}」已刪除（尚未提交）`,
    );
  };

  const handleCreateWall = () => {
    if (!draftWall || !selectedMap) return;
    const name = draftWall.name.trim() || `wall_${walls.length + 1}`;
    void run(
      '建立虛擬牆',
      async () => {
        const created = await createVirtualWall({
          map: selectedMap,
          name,
          start_position: draftWall.start,
          end_position: draftWall.end,
        });
        setWalls((prev) => [...prev, created]);
        setDraftWall(null);
        setTool('none');
        setDirty(true);
        reloadEntities();
      },
      `虛擬牆「${name}」已建立（尚未提交）`,
    );
  };

  const handleDeleteWall = (wall: VirtualWall) => {
    void run(
      '刪除虛擬牆',
      async () => {
        await deleteVirtualWall(wall.id, selectedMap ?? undefined);
        setWalls((prev) => prev.filter((w) => w.id !== wall.id));
        if (selectedId === wall.id) setSelectedId(null);
        setDirty(true);
      },
      `虛擬牆「${wall.name}」已刪除（尚未提交）`,
    );
  };

  const handleCommit = () =>
    void run(
      '提交變更',
      async () => {
        await commitEdits();
        setDirty(false);
        setMapVersion((v) => v + 1);
        reloadEntities();
      },
      '變更已提交並套用',
    );

  const handleDiscard = () =>
    void run(
      '捨棄變更',
      async () => {
        await discardEdits();
        setDirty(false);
        setDraftPoint(null);
        setDraftWall(null);
        setTool('none');
        reloadEntities();
      },
      '已捨棄未提交的變更',
    );

  // ------------------------------------------------------------------ render
  const toggleTool = (next: Tool) => {
    setTool((prev) => (prev === next ? 'none' : next));
    setDraftPoint(null);
    setDraftWall(null);
  };

  return (
    <div className={page.page}>
      <div className={page.mapArea}>
        {dirty ? (
          <div className="dirtyBanner">
            <strong>有未提交的變更</strong>
            <span>點位 / 虛擬牆的修改仍只存在記憶體，尚未寫入磁碟。</span>
            <span style={{ flex: 1 }} />
            <button
              type="button"
              className="btn primary"
              onClick={handleCommit}
              disabled={busy !== null}
            >
              提交變更
            </button>
            <button
              type="button"
              className="btn danger"
              onClick={handleDiscard}
              disabled={busy !== null}
            >
              捨棄變更
            </button>
          </div>
        ) : (
          <div className="cleanBanner">目前沒有未提交的變更。</div>
        )}

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

        <section className="panel">
          <h2 className="panelTitle">編輯工具</h2>
          <div className="row">
            <button
              type="button"
              className={`btn ${tool === 'add-point' ? 'active' : ''}`}
              disabled={!selectedMap || !meta}
              onClick={() => toggleTool('add-point')}
            >
              新增點位
            </button>
            <button
              type="button"
              className={`btn ${tool === 'add-wall' ? 'active' : ''}`}
              disabled={!selectedMap || !meta}
              onClick={() => toggleTool('add-wall')}
            >
              新增虛擬牆
            </button>
            <button
              type="button"
              className={`btn ${tool === 'move-point' ? 'active' : ''}`}
              disabled={!selectedPoint || !meta}
              onClick={() => toggleTool('move-point')}
            >
              重設選取點位置
            </button>
          </div>
          <p className="hintText" style={{ marginTop: 8 }}>
            {TOOL_HINT[tool]}
          </p>
        </section>

        {draftPoint && (
          <section className="panel">
            <h2 className="panelTitle">新點位</h2>
            <div className="field">
              <label className="fieldLabel">名稱</label>
              <input
                className="input"
                autoFocus
                value={draftPoint.name}
                onChange={(e) =>
                  setDraftPoint({ ...draftPoint, name: e.target.value })
                }
                onKeyDown={(e) => {
                  if (e.key === 'Enter') handleCreatePoint();
                }}
              />
            </div>
            <div className="row" style={{ marginTop: 8 }}>
              <div className="field" style={{ flex: 1 }}>
                <label className="fieldLabel">類型</label>
                <select
                  className="select"
                  value={draftPoint.type}
                  onChange={(e) =>
                    setDraftPoint({
                      ...draftPoint,
                      type: e.target.value as PointType,
                    })
                  }
                >
                  <option value="point">一般點位 (point)</option>
                  <option value="charge">充電座 (charge)</option>
                </select>
              </div>
              <div className="field" style={{ width: 110 }}>
                <label className="fieldLabel">朝向 (°)</label>
                <input
                  className="input"
                  type="number"
                  value={Math.round(draftPoint.location.orientation)}
                  onChange={(e) =>
                    setDraftPoint({
                      ...draftPoint,
                      location: {
                        ...draftPoint.location,
                        orientation: normalizeDegrees(Number(e.target.value) || 0),
                      },
                    })
                  }
                />
              </div>
            </div>
            <p className="hintText" style={{ marginTop: 8 }}>
              座標 {draftPoint.location.x}, {draftPoint.location.y} cm
            </p>
            <div className="row" style={{ marginTop: 10 }}>
              <button
                type="button"
                className="btn primary"
                onClick={handleCreatePoint}
                disabled={!draftPoint.name.trim() || busy !== null}
              >
                建立
              </button>
              <button
                type="button"
                className="btn"
                onClick={() => setDraftPoint(null)}
              >
                取消
              </button>
            </div>
          </section>
        )}

        {draftWall && (
          <section className="panel">
            <h2 className="panelTitle">新虛擬牆</h2>
            <div className="field">
              <label className="fieldLabel">名稱（留空自動命名）</label>
              <input
                className="input"
                autoFocus
                value={draftWall.name}
                onChange={(e) =>
                  setDraftWall({ ...draftWall, name: e.target.value })
                }
                onKeyDown={(e) => {
                  if (e.key === 'Enter') handleCreateWall();
                }}
              />
            </div>
            <p className="hintText" style={{ marginTop: 8 }}>
              ({draftWall.start.x}, {draftWall.start.y}) → ({draftWall.end.x},{' '}
              {draftWall.end.y}) cm
            </p>
            <div className="row" style={{ marginTop: 10 }}>
              <button
                type="button"
                className="btn primary"
                onClick={handleCreateWall}
                disabled={busy !== null}
              >
                建立
              </button>
              <button
                type="button"
                className="btn"
                onClick={() => setDraftWall(null)}
              >
                取消
              </button>
            </div>
          </section>
        )}

        <section className="panel">
          <h2 className="panelTitle">
            點位 <span className="tag">{points.length}</span>
          </h2>
          {points.length === 0 ? (
            <p className="hintText">尚未建立任何點位。</p>
          ) : (
            <ul className="list">
              {points.map((p) => (
                <PointRow
                  key={p.id}
                  point={p}
                  selected={p.id === selectedId}
                  busy={busy !== null}
                  onSelect={() => setSelectedId(p.id === selectedId ? null : p.id)}
                  onSave={handleRenamePoint}
                  onOrientation={handleUpdateOrientation}
                  onDelete={handleDeletePoint}
                />
              ))}
            </ul>
          )}
        </section>

        <section className="panel">
          <h2 className="panelTitle">
            虛擬牆 <span className="tag">{walls.length}</span>
          </h2>
          {walls.length === 0 ? (
            <p className="hintText">尚未建立任何虛擬牆。</p>
          ) : (
            <ul className="list">
              {walls.map((w) => (
                <li
                  key={w.id}
                  className={`listItem ${w.id === selectedId ? 'selected' : ''}`}
                  onClick={() => setSelectedId(w.id === selectedId ? null : w.id)}
                >
                  <div className="listItemMain">
                    <div className="listItemName">{w.name}</div>
                    <div className="listItemMeta">
                      ({w.start_position.x}, {w.start_position.y}) → (
                      {w.end_position.x}, {w.end_position.y})
                    </div>
                  </div>
                  <button
                    type="button"
                    className="btn small danger"
                    disabled={busy !== null}
                    onClick={(e) => {
                      e.stopPropagation();
                      handleDeleteWall(w);
                    }}
                  >
                    刪除
                  </button>
                </li>
              ))}
            </ul>
          )}
        </section>

        <section className="panel">
          <h2 className="panelTitle">變更交易</h2>
          <p className="hintText">
            後端採暫存交易模型：未提交的變更只存在記憶體，磁碟永遠是已提交狀態。
          </p>
          <div className="row" style={{ marginTop: 10 }}>
            <button
              type="button"
              className="btn primary"
              onClick={handleCommit}
              disabled={busy !== null}
            >
              提交變更 (commit)
            </button>
            <button
              type="button"
              className="btn danger"
              onClick={handleDiscard}
              disabled={busy !== null}
            >
              捨棄變更 (discard)
            </button>
          </div>
        </section>
      </div>
    </div>
  );
}

interface PointRowProps {
  point: RobotPoint;
  selected: boolean;
  busy: boolean;
  onSelect: () => void;
  onSave: (point: RobotPoint, name: string, type: PointType) => void;
  onOrientation: (point: RobotPoint, orientation: number) => void;
  onDelete: (point: RobotPoint) => void;
}

function PointRow({
  point,
  selected,
  busy,
  onSelect,
  onSave,
  onOrientation,
  onDelete,
}: PointRowProps) {
  const [editing, setEditing] = useState(false);
  const [name, setName] = useState(point.name);
  const [type, setType] = useState<PointType>(point.type);
  const [orientation, setOrientation] = useState(
    String(Math.round(point.location.orientation)),
  );

  // 外部資料重新載入後同步回編輯欄位
  useEffect(() => {
    setName(point.name);
    setType(point.type);
    setOrientation(String(Math.round(point.location.orientation)));
  }, [point.name, point.type, point.location.orientation]);

  if (!editing) {
    return (
      <li
        className={`listItem ${selected ? 'selected' : ''}`}
        onClick={onSelect}
      >
        <div className="listItemMain">
          <div className="listItemName">
            {point.name}{' '}
            {point.type === 'charge' && <span className="tag charge">charge</span>}
          </div>
          <div className="listItemMeta">
            {point.location.x}, {point.location.y} cm @{' '}
            {Math.round(point.location.orientation)}°
          </div>
        </div>
        <button
          type="button"
          className="btn small"
          onClick={(e) => {
            e.stopPropagation();
            setEditing(true);
          }}
        >
          編輯
        </button>
        <button
          type="button"
          className="btn small danger"
          disabled={busy}
          onClick={(e) => {
            e.stopPropagation();
            onDelete(point);
          }}
        >
          刪除
        </button>
      </li>
    );
  }

  const commitRow = () => {
    if (name.trim() !== point.name || type !== point.type) {
      onSave(point, name, type);
    }
    const deg = Number(orientation);
    if (
      Number.isFinite(deg) &&
      Math.round(deg) !== Math.round(point.location.orientation)
    ) {
      onOrientation(point, deg);
    }
    setEditing(false);
  };

  return (
    <li className={`listItem ${selected ? 'selected' : ''}`}>
      <div className="listItemMain" style={{ display: 'grid', gap: 6 }}>
        <input
          className="input"
          value={name}
          autoFocus
          onChange={(e) => setName(e.target.value)}
        />
        <div className="row">
          <select
            className="select"
            style={{ flex: 1 }}
            value={type}
            onChange={(e) => setType(e.target.value as PointType)}
          >
            <option value="point">point</option>
            <option value="charge">charge</option>
          </select>
          <input
            className="input"
            style={{ width: 80 }}
            type="number"
            value={orientation}
            onChange={(e) => setOrientation(e.target.value)}
          />
        </div>
      </div>
      <button
        type="button"
        className="btn small primary"
        disabled={busy || !name.trim()}
        onClick={commitRow}
      >
        儲存
      </button>
      <button
        type="button"
        className="btn small"
        onClick={() => {
          setName(point.name);
          setType(point.type);
          setOrientation(String(Math.round(point.location.orientation)));
          setEditing(false);
        }}
      >
        取消
      </button>
    </li>
  );
}
