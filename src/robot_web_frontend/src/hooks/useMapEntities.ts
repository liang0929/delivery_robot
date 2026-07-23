// 地圖點位 / 虛擬牆載入（含卸載保護、地圖切換與手動重載的競態防護）。

import { useCallback, useEffect, useRef, useState } from 'react';
import type { Dispatch, SetStateAction } from 'react';
import { listPoints, listVirtualWalls } from '../api/robot.api';
import { describeError, isAbortError } from '../api/client';
import type { RobotPoint, VirtualWall } from '../api/types';

export interface MapEntitiesState {
  points: RobotPoint[];
  walls: VirtualWall[];
  loading: boolean;
  error: string | null;
  reload: () => void;
  setPoints: Dispatch<SetStateAction<RobotPoint[]>>;
  setWalls: Dispatch<SetStateAction<VirtualWall[]>>;
}

/**
 * 依 selectedMap 抓取 points / virtual walls。selectedMap 為空時清空並跳過抓取。
 * reloadKey 供呼叫端以外部計數器觸發重抓（例如跟著另一個 version state 連動）；
 * 一般情況下用回傳的 reload() 即可。
 */
export function useMapEntities(
  selectedMap: string | null,
  reloadKey = 0,
): MapEntitiesState {
  const [points, setPoints] = useState<RobotPoint[]>([]);
  const [walls, setWalls] = useState<VirtualWall[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [version, setVersion] = useState(0);
  const requestId = useRef(0);

  useEffect(() => {
    if (!selectedMap) {
      requestId.current += 1;
      setPoints([]);
      setWalls([]);
      setError(null);
      setLoading(false);
      return;
    }
    const id = ++requestId.current;
    const controller = new AbortController();
    setLoading(true);

    (async () => {
      try {
        const [nextPoints, nextWalls] = await Promise.all([
          listPoints(selectedMap, { signal: controller.signal }),
          listVirtualWalls(selectedMap, { signal: controller.signal }),
        ]);
        if (id !== requestId.current) return;
        setPoints(nextPoints);
        setWalls(nextWalls);
        setError(null);
      } catch (err) {
        if (isAbortError(err) || id !== requestId.current) return;
        setPoints([]);
        setWalls([]);
        setError(describeError(err));
      } finally {
        if (id === requestId.current) setLoading(false);
      }
    })();

    return () => controller.abort();
  }, [selectedMap, version, reloadKey]);

  const reload = useCallback(() => setVersion((v) => v + 1), []);

  return { points, walls, loading, error, reload, setPoints, setWalls };
}
