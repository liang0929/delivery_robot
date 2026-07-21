// 座標換算集中處。
//
// 三個座標系：
//   1. API 座標    — 公分整數 (x, y)，orientation 為度（CCW，0 = +x）
//   2. ROS world   — 公尺浮點，與地圖 origin 同一參考系
//   3. 影像像素    — 地圖 PNG 的 (col, row)，row 0 在最上方（y 軸向下）
//
// 影像與 ROS world 的關係（ROS map_server 慣例）：
//   world_x = origin_x + col * resolution
//   world_y = origin_y + (height - row) * resolution
//
// 除了本檔案，其他地方不應該出現 resolution / origin / 100 這類魔術換算。

import type { ApiLocation, ApiPosition, MapMetadata } from '../api/types';

/** ROS 公尺 → API 公分整數 */
export const COORD_SCALE = 100;

export interface PixelPoint {
  /** 影像欄（x，像素） */
  col: number;
  /** 影像列（y，像素） */
  row: number;
}

export interface WorldPoint {
  /** 公尺 */
  x: number;
  /** 公尺 */
  y: number;
}

// ---------------------------------------------------------------- 公尺 ↔ 公分

export const metersToCm = (m: number): number => Math.round(m * COORD_SCALE);
export const cmToMeters = (cm: number): number => cm / COORD_SCALE;

// ------------------------------------------------------------ world ↔ 影像像素

export function worldToPixel(world: WorldPoint, meta: MapMetadata): PixelPoint {
  const [ox, oy] = meta.origin;
  return {
    col: (world.x - ox) / meta.resolution,
    row: meta.height - (world.y - oy) / meta.resolution,
  };
}

export function pixelToWorld(px: PixelPoint, meta: MapMetadata): WorldPoint {
  const [ox, oy] = meta.origin;
  return {
    x: ox + px.col * meta.resolution,
    y: oy + (meta.height - px.row) * meta.resolution,
  };
}

// -------------------------------------------------------------- API ↔ 影像像素

/** API 公分座標 → 影像像素 */
export function apiToPixel(
  pos: ApiPosition | ApiLocation,
  meta: MapMetadata,
): PixelPoint {
  return worldToPixel({ x: cmToMeters(pos.x), y: cmToMeters(pos.y) }, meta);
}

/** 影像像素 → API 公分座標（整數） */
export function pixelToApi(px: PixelPoint, meta: MapMetadata): ApiPosition {
  const world = pixelToWorld(px, meta);
  return { x: metersToCm(world.x), y: metersToCm(world.y) };
}

/** 影像像素 + 朝向 → API Location */
export function pixelToApiLocation(
  px: PixelPoint,
  orientationDeg: number,
  meta: MapMetadata,
): ApiLocation {
  const { x, y } = pixelToApi(px, meta);
  return { x, y, orientation: normalizeDegrees(orientationDeg) };
}

// ------------------------------------------------------------------------ 角度

/** 正規化到 [0, 360) */
export function normalizeDegrees(deg: number): number {
  const r = deg % 360;
  return r < 0 ? r + 360 : r;
}

/**
 * API 角度（數學慣例，CCW，y 向上）→ canvas 旋轉角（弧度，y 向下）。
 * 因為影像 y 軸翻轉，所以取負號。
 */
export function apiDegToCanvasRad(deg: number): number {
  return (-deg * Math.PI) / 180;
}

/**
 * 由兩個影像像素點推得朝向（度，API 慣例）。
 * 用於「點一下設位置，拖曳決定朝向」的互動。
 */
export function pixelDeltaToApiDeg(from: PixelPoint, to: PixelPoint): number {
  const dCol = to.col - from.col;
  const dRow = to.row - from.row;
  // 影像 row 向下 = world y 向下，故取 -dRow
  return normalizeDegrees((Math.atan2(-dRow, dCol) * 180) / Math.PI);
}

// -------------------------------------------------------- canvas 顯示座標轉換

/**
 * canvas 以 `contain` 方式縮放地圖影像時的擺放參數。
 * 用來把滑鼠事件的 client 座標換回影像像素。
 */
export interface ViewTransform {
  /** 影像像素 → canvas CSS 像素的縮放倍率 */
  scale: number;
  /** 影像左上角在 canvas 中的 CSS 位移 */
  offsetX: number;
  offsetY: number;
}

export function computeViewTransform(
  meta: MapMetadata,
  viewWidth: number,
  viewHeight: number,
): ViewTransform {
  if (meta.width <= 0 || meta.height <= 0 || viewWidth <= 0 || viewHeight <= 0) {
    return { scale: 1, offsetX: 0, offsetY: 0 };
  }
  const scale = Math.min(viewWidth / meta.width, viewHeight / meta.height);
  return {
    scale,
    offsetX: (viewWidth - meta.width * scale) / 2,
    offsetY: (viewHeight - meta.height * scale) / 2,
  };
}

/** canvas CSS 座標 → 影像像素 */
export function viewToPixel(
  viewX: number,
  viewY: number,
  t: ViewTransform,
): PixelPoint {
  return {
    col: (viewX - t.offsetX) / t.scale,
    row: (viewY - t.offsetY) / t.scale,
  };
}

/** 影像像素 → canvas CSS 座標 */
export function pixelToView(
  px: PixelPoint,
  t: ViewTransform,
): { x: number; y: number } {
  return {
    x: px.col * t.scale + t.offsetX,
    y: px.row * t.scale + t.offsetY,
  };
}

/** 該像素是否落在地圖影像範圍內 */
export function isPixelInsideMap(px: PixelPoint, meta: MapMetadata): boolean {
  return (
    px.col >= 0 && px.col <= meta.width && px.row >= 0 && px.row <= meta.height
  );
}
