// 設定點位分頁的編輯邏輯：points/walls 資料、未提交（dirty）狀態、
// 以及全部 CRUD / 交易 async handler。
//
// 後端是暫存交易模型（規格 §4.2、§8）：所有變更先進記憶體，
// 必須按「提交變更」才會寫入磁碟並套用。
//
// tool / draftPoint / draftWall / selectedId 屬於 UI 狀態，留在呼叫端
// （PointsPage）管理；本 hook 只透過參數讀取、透過 setter 更新它們，
// 藉此讓「新增點位後清空草稿」「刪除選取中的項目後清空選取」等既有時序不變。

import { useCallback, useState } from 'react';
import type { Dispatch, SetStateAction } from 'react';
import { useMapEntities } from './useMapEntities';
import { useAction } from './useAction';
import {
  commitEdits as apiCommitEdits,
  createPoint as apiCreatePoint,
  createVirtualWall as apiCreateVirtualWall,
  deletePoint as apiDeletePoint,
  deleteVirtualWall as apiDeleteVirtualWall,
  discardEdits as apiDiscardEdits,
  updatePoint as apiUpdatePoint,
} from '../api/robot.api';
import {
  normalizeDegrees,
  pixelToApi,
  pixelToApiLocation,
  type PixelPoint,
} from '../lib/coords';
import type {
  ApiLocation,
  ApiPosition,
  MapMetadata,
  PointType,
  RobotPoint,
  VirtualWall,
} from '../api/types';

export type Tool = 'none' | 'add-point' | 'add-wall' | 'move-point';

export interface DraftPoint {
  location: ApiLocation;
  name: string;
  type: PointType;
}

export interface DraftWall {
  start: ApiPosition;
  end: ApiPosition;
  name: string;
}

export interface UsePointsEditorParams {
  selectedMap: string | null;
  meta: MapMetadata | null;
  tool: Tool;
  draftPoint: DraftPoint | null;
  draftWall: DraftWall | null;
  selectedId: string | null;
  setDraftPoint: Dispatch<SetStateAction<DraftPoint | null>>;
  setDraftWall: Dispatch<SetStateAction<DraftWall | null>>;
  setSelectedId: Dispatch<SetStateAction<string | null>>;
  setTool: Dispatch<SetStateAction<Tool>>;
  /** 提交成功後呼叫（例如讓 Page 端遞增 mapVersion 觸發地圖重新載入） */
  onCommitted?: () => void;
}

export interface UsePointsEditorResult {
  points: RobotPoint[];
  walls: VirtualWall[];
  dirty: boolean;
  entitiesError: string | null;
  busy: string | null;
  selectedPoint: RobotPoint | null;
  handlePose: (px: PixelPoint, deg: number) => void;
  handleSegment: (start: PixelPoint, end: PixelPoint) => void;
  createPoint: () => void;
  renamePoint: (point: RobotPoint, name: string, type: PointType) => void;
  updateOrientation: (point: RobotPoint, orientation: number) => void;
  deletePoint: (point: RobotPoint) => void;
  createWall: () => void;
  deleteWall: (wall: VirtualWall) => void;
  commit: () => void;
  discard: () => void;
}

export function usePointsEditor(
  params: UsePointsEditorParams,
): UsePointsEditorResult {
  const {
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
    onCommitted,
  } = params;

  const { busy, run } = useAction();

  const {
    points,
    walls,
    error: entitiesError,
    reload: reloadEntities,
    setPoints,
    setWalls,
  } = useMapEntities(selectedMap);

  const [dirty, setDirty] = useState(false);

  const selectedPoint = points.find((p) => p.id === selectedId) ?? null;

  // --------------------------------------------------------------- 地圖互動
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
            const updated = await apiUpdatePoint(selectedPoint, { location });
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
    [meta, tool, selectedPoint, run, setDraftPoint, setSelectedId, setTool, setPoints],
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
    [meta, setDraftWall],
  );

  // --------------------------------------------------------------- 動作處理
  const createPoint = () => {
    if (!draftPoint || !selectedMap) return;
    const name = draftPoint.name.trim();
    if (!name) return;
    void run(
      '建立點位',
      async () => {
        const created = await apiCreatePoint({
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

  const renamePoint = (point: RobotPoint, name: string, type: PointType) => {
    const trimmed = name.trim();
    if (!trimmed) return;
    void run(
      '更新點位',
      async () => {
        const updated = await apiUpdatePoint(point, { name: trimmed, type });
        setPoints((prev) => prev.map((p) => (p.id === point.id ? updated : p)));
        setDirty(true);
      },
      '點位已更新（尚未提交）',
    );
  };

  const updateOrientation = (point: RobotPoint, orientation: number) => {
    const location: ApiLocation = {
      ...point.location,
      orientation: normalizeDegrees(orientation),
    };
    void run(
      '更新朝向',
      async () => {
        const updated = await apiUpdatePoint(point, { location });
        setPoints((prev) => prev.map((p) => (p.id === point.id ? updated : p)));
        setDirty(true);
      },
      '朝向已更新（尚未提交）',
    );
  };

  const deletePoint = (point: RobotPoint) => {
    void run(
      '刪除點位',
      async () => {
        await apiDeletePoint(point.id, selectedMap ?? undefined);
        setPoints((prev) => prev.filter((p) => p.id !== point.id));
        if (selectedId === point.id) setSelectedId(null);
        setDirty(true);
      },
      `點位「${point.name}」已刪除（尚未提交）`,
    );
  };

  const createWall = () => {
    if (!draftWall || !selectedMap) return;
    const name = draftWall.name.trim() || `wall_${walls.length + 1}`;
    void run(
      '建立虛擬牆',
      async () => {
        const created = await apiCreateVirtualWall({
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

  const deleteWall = (wall: VirtualWall) => {
    void run(
      '刪除虛擬牆',
      async () => {
        await apiDeleteVirtualWall(wall.id, selectedMap ?? undefined);
        setWalls((prev) => prev.filter((w) => w.id !== wall.id));
        if (selectedId === wall.id) setSelectedId(null);
        setDirty(true);
      },
      `虛擬牆「${wall.name}」已刪除（尚未提交）`,
    );
  };

  const commit = () =>
    void run(
      '提交變更',
      async () => {
        await apiCommitEdits();
        setDirty(false);
        onCommitted?.();
        reloadEntities();
      },
      '變更已提交並套用',
    );

  const discard = () =>
    void run(
      '捨棄變更',
      async () => {
        await apiDiscardEdits();
        setDirty(false);
        setDraftPoint(null);
        setDraftWall(null);
        setTool('none');
        reloadEntities();
      },
      '已捨棄未提交的變更',
    );

  return {
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
  };
}
