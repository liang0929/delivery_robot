// Winstec Robot API v1.1 端點封裝（docs/winstec_api_v1.1.md §5、§7）
// 所有座標皆為 API 單位（公分整數 + 度）；換算請走 lib/coords.ts。

import { absoluteUrl, LONG_TIMEOUT_MS, request, requestBlobUrl } from './client';
import type {
  ApiLocation,
  ApiPosition,
  ManualDirection,
  MapMetadata,
  OpMode,
  PointType,
  RobotInfo,
  RobotPoint,
  VirtualWall,
} from './types';

type Sig = { signal?: AbortSignal };

// ------------------------------------------------------------ 機器人資訊與移動

export const getInfo = (o: Sig = {}) =>
  request<RobotInfo>('/info', { signal: o.signal });

export const moveToLocation = (location: ApiLocation, o: Sig = {}) =>
  request<void>('/move', {
    method: 'POST',
    body: { type: 'location', location },
    signal: o.signal,
  });

export const moveToPoint = (pointId: string, o: Sig = {}) =>
  request<RobotPoint>(`/move/${encodeURIComponent(pointId)}`, {
    method: 'POST',
    signal: o.signal,
  });

export const manualMove = (direction: ManualDirection, o: Sig = {}) =>
  request<void>('/manual/move', {
    method: 'POST',
    body: { direction },
    signal: o.signal,
  });

export const stop = (o: Sig = {}) =>
  request<void>('/stop', { method: 'POST', signal: o.signal });

export const relocateToLocation = (location: ApiLocation, o: Sig = {}) =>
  request<{ location: ApiLocation }>('/relocate/location', {
    method: 'POST',
    body: { location },
    signal: o.signal,
  });

export const relocateToPoint = (pointId: string, o: Sig = {}) =>
  request<RobotPoint>(`/relocate/${encodeURIComponent(pointId)}`, {
    method: 'POST',
    signal: o.signal,
  });

// ---------------------------------------------------------------------- Points

export interface CreatePointBody {
  map?: string;
  name: string;
  type: PointType;
  /** 省略時後端使用機器人當前位置 */
  location?: ApiLocation;
}

export const createPoint = (body: CreatePointBody, o: Sig = {}) =>
  request<RobotPoint>('/points', { method: 'POST', body, signal: o.signal });

export const listPoints = (map?: string, o: Sig = {}) =>
  request<{ points: RobotPoint[] }>('/points', {
    query: { map },
    signal: o.signal,
  }).then((r) => r?.points ?? []);

export interface UpdatePointBody {
  name?: string;
  type?: PointType;
  location?: ApiLocation;
}

export const updatePoint = (
  pointId: string,
  body: UpdatePointBody,
  o: Sig = {},
) =>
  request<RobotPoint>(`/points/${encodeURIComponent(pointId)}`, {
    method: 'PATCH',
    body,
    signal: o.signal,
  });

export const deletePoint = (pointId: string, map?: string, o: Sig = {}) =>
  request<void>(`/points/${encodeURIComponent(pointId)}`, {
    method: 'DELETE',
    query: { map },
    signal: o.signal,
  });

// --------------------------------------------------------------- Virtual Walls

export interface CreateVirtualWallBody {
  map?: string;
  name: string;
  start_position: ApiPosition;
  end_position: ApiPosition;
}

export const createVirtualWall = (body: CreateVirtualWallBody, o: Sig = {}) =>
  request<VirtualWall>('/virtual-walls', {
    method: 'POST',
    body,
    signal: o.signal,
  });

export const listVirtualWalls = (map?: string, o: Sig = {}) =>
  request<{ virtual_walls: VirtualWall[] }>('/virtual-walls', {
    query: { map },
    signal: o.signal,
  }).then((r) => r?.virtual_walls ?? []);

export const deleteVirtualWall = (
  virtualWallId: string,
  map?: string,
  o: Sig = {},
) =>
  request<void>(`/virtual-walls/${encodeURIComponent(virtualWallId)}`, {
    method: 'DELETE',
    query: { map },
    signal: o.signal,
  });

// ------------------------------------------------------------------ 編輯交易

export const commitEdits = (o: Sig = {}) =>
  request<void>('/edits/commit', { method: 'POST', signal: o.signal });

export const discardEdits = (o: Sig = {}) =>
  request<void>('/edits/discard', { method: 'POST', signal: o.signal });

// ------------------------------------------------------- 擴充端點：模式與地圖

/** 切換模式要拉起整個 Nav2 或 SLAM，正常就需要數十秒，故用長逾時 */
export const switchMode = (mode: OpMode, map?: string, o: Sig = {}) =>
  request<void>('/mode', {
    method: 'POST',
    body: map ? { mode, map } : { mode },
    signal: o.signal,
    timeoutMs: LONG_TIMEOUT_MS,
  });

/** 後端可能回 `{maps:[...]}`、字串陣列或物件陣列，統一成字串陣列 */
export const listMaps = (o: Sig = {}) =>
  request<unknown>('/maps', { signal: o.signal }).then(normalizeMapList);

function normalizeMapList(raw: unknown): string[] {
  const arr = Array.isArray(raw)
    ? raw
    : Array.isArray((raw as { maps?: unknown })?.maps)
      ? ((raw as { maps: unknown[] }).maps)
      : [];
  return arr
    .map((m) =>
      typeof m === 'string' ? m : String((m as { name?: unknown })?.name ?? ''),
    )
    .filter((n): n is string => n.length > 0);
}

export const saveMap = (name: string, o: Sig = {}) =>
  request<void>('/maps', { method: 'POST', body: { name }, signal: o.signal });

export const deleteMap = (name: string, o: Sig = {}) =>
  request<void>(`/maps/${encodeURIComponent(name)}`, {
    method: 'DELETE',
    signal: o.signal,
  });

export const getMapMetadata = (name: string, o: Sig = {}) =>
  request<MapMetadata>(`/maps/${encodeURIComponent(name)}/metadata`, {
    signal: o.signal,
  });

export const getLiveMapMetadata = (o: Sig = {}) =>
  request<MapMetadata>('/maps/live/metadata', { signal: o.signal });

/** 已存檔地圖 PNG → object URL（呼叫端須 revoke） */
export const fetchMapImageUrl = (name: string, o: Sig = {}) =>
  requestBlobUrl(`/maps/${encodeURIComponent(name)}/image`, {
    signal: o.signal,
  });

/**
 * 建圖中的即時地圖 PNG → object URL。
 * 帶 cache-busting 參數避免瀏覽器快取住同一張。
 */
export const fetchLiveMapImageUrl = (o: Sig = {}) =>
  requestBlobUrl('/maps/live/image', {
    query: { _t: Date.now() },
    signal: o.signal,
  });

export const mapImageHref = (name: string) =>
  absoluteUrl(`/maps/${encodeURIComponent(name)}/image`);
