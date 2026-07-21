// 地圖影像 / metadata 載入。
//
// 兩種來源：
//   useStoredMap(name)  — 已存檔地圖，切換 name 時舊請求會被 abort，且用 requestId
//                          雙重防護，確保晚回來的舊回應不會蓋掉新地圖
//   useLiveMap(active)  — 建圖中的即時地圖，序列化輪詢（前一輪完成才排下一輪），
//                          不會堆積請求
//
// 兩者都會在卸載時 abort、revoke object URL。

import { useEffect, useRef, useState } from 'react';
import {
  fetchLiveMapImageUrl,
  fetchMapImageUrl,
  getLiveMapMetadata,
  getMapMetadata,
} from '../api/robot.api';
import { describeError, isAbortError } from '../api/client';
import { ROBOT_CONFIG } from '../config/robot.config';
import type { MapMetadata } from '../api/types';

export interface MapSource {
  image: HTMLImageElement | null;
  meta: MapMetadata | null;
  loading: boolean;
  error: string | null;
}

const EMPTY: MapSource = { image: null, meta: null, loading: false, error: null };

function loadImage(url: string, signal?: AbortSignal): Promise<HTMLImageElement> {
  return new Promise((resolve, reject) => {
    const img = new Image();
    const cleanup = () => {
      img.onload = null;
      img.onerror = null;
      signal?.removeEventListener('abort', onAbort);
    };
    const onAbort = () => {
      cleanup();
      img.src = '';
      reject(new DOMException('aborted', 'AbortError'));
    };
    img.onload = () => {
      cleanup();
      resolve(img);
    };
    img.onerror = () => {
      cleanup();
      reject(new Error('地圖影像解碼失敗'));
    };
    signal?.addEventListener('abort', onAbort);
    img.src = url;
  });
}

/** 取回 object URL、解碼成 Image，然後立刻 revoke（Image 已持有解碼結果） */
async function fetchImage(
  urlPromise: Promise<string>,
  signal: AbortSignal,
): Promise<HTMLImageElement> {
  const objectUrl = await urlPromise;
  try {
    return await loadImage(objectUrl, signal);
  } finally {
    URL.revokeObjectURL(objectUrl);
  }
}

/**
 * 已存檔地圖。`reloadKey` 改變時會重新抓取（例如提交編輯後）。
 */
export function useStoredMap(name: string | null, reloadKey = 0): MapSource {
  const [state, setState] = useState<MapSource>(EMPTY);
  // 每次請求遞增；只有最新一輪的結果會被寫入 state
  const requestId = useRef(0);

  useEffect(() => {
    if (!name) {
      requestId.current += 1;
      setState(EMPTY);
      return;
    }

    const id = ++requestId.current;
    const controller = new AbortController();
    setState((prev) => ({ ...prev, loading: true, error: null }));

    (async () => {
      try {
        const [meta, image] = await Promise.all([
          getMapMetadata(name, { signal: controller.signal }),
          fetchImage(
            fetchMapImageUrl(name, { signal: controller.signal }),
            controller.signal,
          ),
        ]);
        if (id !== requestId.current) return; // 已被更新的請求取代
        setState({ image, meta, loading: false, error: null });
      } catch (err) {
        if (isAbortError(err) || id !== requestId.current) return;
        setState({ image: null, meta: null, loading: false, error: describeError(err) });
      }
    })();

    return () => controller.abort();
  }, [name, reloadKey]);

  return state;
}

/**
 * 建圖中的即時地圖，約 1.25 Hz 輪詢。
 * 序列化排程：一輪完成（成功或失敗）後才安排下一輪。
 */
export function useLiveMap(active: boolean): MapSource {
  const [state, setState] = useState<MapSource>(EMPTY);

  useEffect(() => {
    if (!active) {
      setState(EMPTY);
      return;
    }

    let cancelled = false;
    let timer: ReturnType<typeof setTimeout> | null = null;
    const controller = new AbortController();

    const tick = async () => {
      try {
        const [meta, image] = await Promise.all([
          getLiveMapMetadata({ signal: controller.signal }),
          fetchImage(
            fetchLiveMapImageUrl({ signal: controller.signal }),
            controller.signal,
          ),
        ]);
        if (cancelled) return;
        setState({ image, meta, loading: false, error: null });
      } catch (err) {
        if (cancelled || isAbortError(err)) return;
        // 建圖剛啟動時 live map 可能還沒 ready，保留上一張並只記錄錯誤
        setState((prev) => ({ ...prev, loading: false, error: describeError(err) }));
      } finally {
        if (!cancelled) {
          timer = setTimeout(tick, ROBOT_CONFIG.LIVE_MAP_POLL_MS);
        }
      }
    };

    setState((prev) => ({ ...prev, loading: true }));
    void tick();

    return () => {
      cancelled = true;
      if (timer !== null) clearTimeout(timer);
      controller.abort();
    };
  }, [active]);

  return state;
}
