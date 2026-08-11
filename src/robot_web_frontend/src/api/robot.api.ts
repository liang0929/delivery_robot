// Winstec Robot API v1.1 端點封裝（docs/winstec_api_v1.1.md §5、§7）
// 所有座標皆為 API 單位（公分整數 + 度）；換算請走 lib/coords.ts。

import { absoluteUrl, LONG_TIMEOUT_MS, request, requestBlobUrl } from './client';
import type {
  ApiLocation,
  ApiPosition,
  DockPoseRecord,
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

/** 後端 POST 可能不回完整物件，缺值時用送出的 body 補齊回傳完整 RobotPoint */
export const createPoint = async (
  body: CreatePointBody,
  o: Sig = {},
): Promise<RobotPoint> => {
  const res = await request<RobotPoint | undefined>('/points', {
    method: 'POST',
    body,
    signal: o.signal,
  });
  return (
    res ?? {
      id: `tmp_${Date.now()}`,
      map: body.map ?? '',
      name: body.name,
      type: body.type,
      location: body.location ?? { x: 0, y: 0, orientation: 0 },
    }
  );
};

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

/** 後端 PATCH 可能不回完整物件，缺值時用送出前的點位 + body 補齊回傳完整 RobotPoint */
export const updatePoint = async (
  point: RobotPoint,
  body: UpdatePointBody,
  o: Sig = {},
): Promise<RobotPoint> => {
  const res = await request<RobotPoint | undefined>(
    `/points/${encodeURIComponent(point.id)}`,
    { method: 'PATCH', body, signal: o.signal },
  );
  return res ?? { ...point, ...body };
};

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

/** 後端 POST 可能不回完整物件，缺值時用送出的 body 補齊回傳完整 VirtualWall */
export const createVirtualWall = async (
  body: CreateVirtualWallBody,
  o: Sig = {},
): Promise<VirtualWall> => {
  const res = await request<VirtualWall | undefined>('/virtual-walls', {
    method: 'POST',
    body,
    signal: o.signal,
  });
  return (
    res ?? {
      id: `tmp_${Date.now()}`,
      map: body.map ?? '',
      name: body.name,
      start_position: body.start_position,
      end_position: body.end_position,
    }
  );
};

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

export interface ModeResult {
  mode: string;
  map: string | null;
  /** 切到 navigate 時才有：AMCL 是否已完成定位。undefined 代表 explore 模式 */
  localized?: boolean;
  /** 未定位時的具體原因，可直接顯示給使用者 */
  detail?: string;
}

/** 切換模式要拉起整個 Nav2 或 SLAM，正常就需要數十秒，故用長逾時 */
export const switchMode = (mode: OpMode, map?: string, o: Sig = {}) =>
  request<ModeResult>('/mode', {
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

// ------------------------------------------------------------- 擴充端點：充電座

/**
 * 記錄「車已與充電座完全對接」時的位姿，寫進 opennav_docking 的 dock database。
 *
 * `frame` 預設 `map`（正式值）。傳 `odom` 是測試模式：odom 每次重開機歸零，
 * 回應會帶 `test_only: true`，UI 必須照樣標示，否則使用者會把它當正式座標。
 */
export const recordDockPose = (
  frame: 'map' | 'odom' = 'map',
  o: Sig = {},
) =>
  request<DockPoseRecord>('/dock/record_pose', {
    method: 'POST',
    query: { frame },
    signal: o.signal,
  });
