// 地圖清單載入（含卸載保護與重載觸發）。

import { useCallback, useEffect, useRef, useState } from 'react';
import { listMaps } from '../api/robot.api';
import { describeError, isAbortError } from '../api/client';

export interface MapListState {
  maps: string[];
  loading: boolean;
  error: string | null;
  reload: () => void;
}

export function useMapList(): MapListState {
  const [maps, setMaps] = useState<string[]>([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [version, setVersion] = useState(0);
  const requestId = useRef(0);

  useEffect(() => {
    const id = ++requestId.current;
    const controller = new AbortController();
    setLoading(true);

    listMaps({ signal: controller.signal })
      .then((list) => {
        if (id !== requestId.current) return;
        setMaps(list);
        setError(null);
      })
      .catch((err: unknown) => {
        if (isAbortError(err) || id !== requestId.current) return;
        setError(describeError(err));
      })
      .finally(() => {
        if (id === requestId.current) setLoading(false);
      });

    return () => controller.abort();
  }, [version]);

  const reload = useCallback(() => setVersion((v) => v + 1), []);

  return { maps, loading, error, reload };
}
