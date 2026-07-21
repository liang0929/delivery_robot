// 地圖畫布。
//
// 效能設計（上一版踩過的坑）：
//   1. 地圖點陣先畫進「離屏 canvas」快取（bitmapRef），之後只做 blit。
//   2. 底圖與疊加層是**兩個獨立的 <canvas>**：
//      - 底圖只在 image / meta / 容器尺寸變更時重畫
//      - 機器人位置每秒更新只會重畫疊加層，不會碰到點陣
//   3. 所有座標換算都呼叫 lib/coords.ts，這裡不自己算 resolution / origin。

import {
  useCallback,
  useEffect,
  useLayoutEffect,
  useMemo,
  useRef,
  useState,
} from 'react';
import type { PointerEvent as ReactPointerEvent } from 'react';
import {
  apiDegToCanvasRad,
  apiToPixel,
  computeViewTransform,
  isPixelInsideMap,
  pixelDeltaToApiDeg,
  pixelToView,
  viewToPixel,
  type PixelPoint,
  type ViewTransform,
} from '../lib/coords';
import type {
  ApiLocation,
  MapMetadata,
  RobotPoint,
  VirtualWall,
} from '../api/types';
import styles from './MapCanvas.module.css';

export type MapInteraction = 'none' | 'click' | 'pose' | 'segment';

export interface MapCanvasProps {
  meta: MapMetadata | null;
  image: HTMLImageElement | null;
  points?: RobotPoint[];
  walls?: VirtualWall[];
  /** 機器人即時位姿（API 單位） */
  robot?: ApiLocation | null;
  /** 目前選取的 point / wall id，會高亮 */
  selectedId?: string | null;
  /** 尚未送出的暫定位姿（例如剛點下的新點位） */
  draftPose?: { location: ApiLocation } | null;
  interaction?: MapInteraction;
  /** interaction='click' 時，單擊地圖 */
  onPick?: (px: PixelPoint) => void;
  /** interaction='pose' 時，點擊設位置、拖曳決定朝向（度） */
  onPose?: (px: PixelPoint, orientationDeg: number) => void;
  /** interaction='segment' 時，拖出一條線段 */
  onSegment?: (start: PixelPoint, end: PixelPoint) => void;
  /** 沒有地圖時顯示的提示 */
  emptyHint?: string;
  /** 左上角小標籤 */
  badge?: string;
}

const COLOR = {
  point: '#4da3ff',
  charge: '#ffc14d',
  selected: '#ffffff',
  wall: '#ff5f6d',
  robot: '#3ddc84',
  draft: '#c084fc',
};

interface DragState {
  startPx: PixelPoint;
  currentPx: PixelPoint;
  pointerId: number;
}

export function MapCanvas(props: MapCanvasProps) {
  const {
    meta,
    image,
    points,
    walls,
    robot,
    selectedId = null,
    draftPose = null,
    interaction = 'none',
    onPick,
    onPose,
    onSegment,
    emptyHint = '尚未載入地圖',
    badge,
  } = props;

  const wrapRef = useRef<HTMLDivElement>(null);
  const baseRef = useRef<HTMLCanvasElement>(null);
  const overlayRef = useRef<HTMLCanvasElement>(null);
  const bitmapRef = useRef<HTMLCanvasElement | null>(null);

  const [size, setSize] = useState({ width: 0, height: 0 });
  const [bitmapVersion, setBitmapVersion] = useState(0);
  const [drag, setDrag] = useState<DragState | null>(null);

  // ------------------------------------------------------------ 容器尺寸追蹤
  useLayoutEffect(() => {
    const el = wrapRef.current;
    if (!el) return;
    const update = () => {
      const rect = el.getBoundingClientRect();
      setSize((prev) =>
        prev.width === rect.width && prev.height === rect.height
          ? prev
          : { width: rect.width, height: rect.height },
      );
    };
    update();
    const ro = new ResizeObserver(update);
    ro.observe(el);
    return () => ro.disconnect();
  }, []);

  // ------------------------------------------------- 地圖點陣 → 離屏 canvas 快取
  useEffect(() => {
    if (!image || image.naturalWidth === 0) {
      bitmapRef.current = null;
      setBitmapVersion((v) => v + 1);
      return;
    }
    const off = document.createElement('canvas');
    off.width = image.naturalWidth;
    off.height = image.naturalHeight;
    const ctx = off.getContext('2d');
    if (!ctx) return;
    ctx.drawImage(image, 0, 0);
    bitmapRef.current = off;
    setBitmapVersion((v) => v + 1);
  }, [image]);

  const transform: ViewTransform | null = useMemo(() => {
    if (!meta || size.width === 0 || size.height === 0) return null;
    return computeViewTransform(meta, size.width, size.height);
  }, [meta, size.width, size.height]);

  // ------------------------------------------------------------------ 底圖繪製
  // 依賴刻意只有 bitmapVersion / transform / size：機器人動不會進來。
  useEffect(() => {
    const canvas = baseRef.current;
    if (!canvas) return;
    const ctx = prepareCanvas(canvas, size.width, size.height);
    if (!ctx) return;
    ctx.clearRect(0, 0, size.width, size.height);

    const bitmap = bitmapRef.current;
    if (!bitmap || !transform) return;

    ctx.imageSmoothingEnabled = false;
    ctx.drawImage(
      bitmap,
      transform.offsetX,
      transform.offsetY,
      bitmap.width * transform.scale,
      bitmap.height * transform.scale,
    );
  }, [bitmapVersion, transform, size.width, size.height]);

  // ---------------------------------------------------------------- 疊加層繪製
  useEffect(() => {
    const canvas = overlayRef.current;
    if (!canvas) return;
    const ctx = prepareCanvas(canvas, size.width, size.height);
    if (!ctx) return;
    ctx.clearRect(0, 0, size.width, size.height);
    if (!meta || !transform) return;

    // 虛擬牆
    for (const wall of walls ?? []) {
      const a = pixelToView(apiToPixel(wall.start_position, meta), transform);
      const b = pixelToView(apiToPixel(wall.end_position, meta), transform);
      const active = wall.id === selectedId;
      ctx.strokeStyle = active ? COLOR.selected : COLOR.wall;
      ctx.lineWidth = active ? 5 : 3;
      ctx.lineCap = 'round';
      ctx.beginPath();
      ctx.moveTo(a.x, a.y);
      ctx.lineTo(b.x, b.y);
      ctx.stroke();
    }

    // 點位
    for (const point of points ?? []) {
      const view = pixelToView(apiToPixel(point.location, meta), transform);
      const color = point.type === 'charge' ? COLOR.charge : COLOR.point;
      drawPose(
        ctx,
        view.x,
        view.y,
        point.location.orientation,
        point.id === selectedId ? COLOR.selected : color,
        point.id === selectedId ? 9 : 7,
      );
      drawLabel(ctx, view.x, view.y - 14, point.name);
    }

    // 未送出的暫定點位
    if (draftPose) {
      const view = pixelToView(apiToPixel(draftPose.location, meta), transform);
      ctx.setLineDash([4, 3]);
      drawPose(ctx, view.x, view.y, draftPose.location.orientation, COLOR.draft, 9);
      ctx.setLineDash([]);
    }

    // 拖曳預覽
    if (drag) {
      const a = pixelToView(drag.startPx, transform);
      const b = pixelToView(drag.currentPx, transform);
      if (interaction === 'segment') {
        ctx.strokeStyle = COLOR.draft;
        ctx.lineWidth = 3;
        ctx.setLineDash([6, 4]);
        ctx.beginPath();
        ctx.moveTo(a.x, a.y);
        ctx.lineTo(b.x, b.y);
        ctx.stroke();
        ctx.setLineDash([]);
      } else if (interaction === 'pose') {
        drawPose(
          ctx,
          a.x,
          a.y,
          pixelDeltaToApiDeg(drag.startPx, drag.currentPx),
          COLOR.draft,
          9,
        );
      }
    }

    // 機器人（畫最上層）
    if (robot) {
      const view = pixelToView(apiToPixel(robot, meta), transform);
      drawRobot(ctx, view.x, view.y, robot.orientation);
    }
  }, [
    meta,
    transform,
    points,
    walls,
    robot,
    selectedId,
    draftPose,
    drag,
    interaction,
    size.width,
    size.height,
  ]);

  // ------------------------------------------------------------------ 指標互動
  const toPixel = useCallback(
    (ev: ReactPointerEvent<HTMLCanvasElement>): PixelPoint | null => {
      if (!transform || !meta) return null;
      const rect = ev.currentTarget.getBoundingClientRect();
      const px = viewToPixel(
        ev.clientX - rect.left,
        ev.clientY - rect.top,
        transform,
      );
      return isPixelInsideMap(px, meta) ? px : null;
    },
    [transform, meta],
  );

  const handlePointerDown = useCallback(
    (ev: ReactPointerEvent<HTMLCanvasElement>) => {
      if (interaction === 'none') return;
      const px = toPixel(ev);
      if (!px) return;
      if (interaction === 'click') {
        onPick?.(px);
        return;
      }
      ev.currentTarget.setPointerCapture(ev.pointerId);
      setDrag({ startPx: px, currentPx: px, pointerId: ev.pointerId });
    },
    [interaction, onPick, toPixel],
  );

  const handlePointerMove = useCallback(
    (ev: ReactPointerEvent<HTMLCanvasElement>) => {
      if (!drag || ev.pointerId !== drag.pointerId || !transform) return;
      const rect = ev.currentTarget.getBoundingClientRect();
      const px = viewToPixel(
        ev.clientX - rect.left,
        ev.clientY - rect.top,
        transform,
      );
      setDrag((prev) => (prev ? { ...prev, currentPx: px } : prev));
    },
    [drag, transform],
  );

  const finishDrag = useCallback(
    (ev: ReactPointerEvent<HTMLCanvasElement>) => {
      if (!drag || ev.pointerId !== drag.pointerId) return;
      if (ev.currentTarget.hasPointerCapture(ev.pointerId)) {
        ev.currentTarget.releasePointerCapture(ev.pointerId);
      }
      const { startPx, currentPx } = drag;
      setDrag(null);
      if (interaction === 'pose') {
        onPose?.(startPx, pixelDeltaToApiDeg(startPx, currentPx));
      } else if (interaction === 'segment') {
        const moved =
          Math.hypot(currentPx.col - startPx.col, currentPx.row - startPx.row) > 2;
        if (moved) onSegment?.(startPx, currentPx);
      }
    },
    [drag, interaction, onPose, onSegment],
  );

  // 指標離開視窗 / 被系統取消時，把拖曳狀態清掉
  useEffect(() => {
    if (!drag) return;
    const cancel = () => setDrag(null);
    window.addEventListener('pointercancel', cancel);
    window.addEventListener('blur', cancel);
    return () => {
      window.removeEventListener('pointercancel', cancel);
      window.removeEventListener('blur', cancel);
    };
  }, [drag]);

  const hasMap = Boolean(meta && bitmapRef.current);

  return (
    <div ref={wrapRef} className={styles.wrap}>
      <canvas ref={baseRef} className={styles.layer} />
      <canvas
        ref={overlayRef}
        className={`${styles.layer} ${styles.overlay} ${
          interaction === 'none' ? '' : styles.interactive
        }`}
        onPointerDown={handlePointerDown}
        onPointerMove={handlePointerMove}
        onPointerUp={finishDrag}
        onPointerCancel={() => setDrag(null)}
      />
      {badge && <div className={styles.badge}>{badge}</div>}
      {!hasMap && <div className={styles.hint}>{emptyHint}</div>}
    </div>
  );
}

// ------------------------------------------------------------------ 繪圖工具

/** 設定 DPR 對應的實際像素尺寸並回傳已縮放的 context */
function prepareCanvas(
  canvas: HTMLCanvasElement,
  cssWidth: number,
  cssHeight: number,
): CanvasRenderingContext2D | null {
  const dpr = window.devicePixelRatio || 1;
  const w = Math.max(1, Math.round(cssWidth * dpr));
  const h = Math.max(1, Math.round(cssHeight * dpr));
  if (canvas.width !== w) canvas.width = w;
  if (canvas.height !== h) canvas.height = h;
  const ctx = canvas.getContext('2d');
  if (!ctx) return null;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return ctx;
}

function drawPose(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  orientationDeg: number,
  color: string,
  radius: number,
) {
  const rad = apiDegToCanvasRad(orientationDeg);
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 2;

  ctx.beginPath();
  ctx.arc(x, y, radius, 0, Math.PI * 2);
  ctx.stroke();

  const len = radius + 12;
  ctx.beginPath();
  ctx.moveTo(x, y);
  ctx.lineTo(x + Math.cos(rad) * len, y + Math.sin(rad) * len);
  ctx.stroke();

  ctx.beginPath();
  ctx.arc(x, y, 2.5, 0, Math.PI * 2);
  ctx.fill();
  ctx.restore();
}

function drawRobot(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  orientationDeg: number,
) {
  const rad = apiDegToCanvasRad(orientationDeg);
  ctx.save();
  ctx.translate(x, y);
  ctx.rotate(rad);
  ctx.fillStyle = COLOR.robot;
  ctx.strokeStyle = '#0b0f14';
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(13, 0);
  ctx.lineTo(-8, 8);
  ctx.lineTo(-8, -8);
  ctx.closePath();
  ctx.fill();
  ctx.stroke();
  ctx.restore();
}

function drawLabel(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  text: string,
) {
  if (!text) return;
  ctx.save();
  ctx.font = '11px system-ui, sans-serif';
  ctx.textAlign = 'center';
  ctx.textBaseline = 'bottom';
  const width = ctx.measureText(text).width;
  ctx.fillStyle = 'rgba(11, 15, 20, 0.75)';
  ctx.fillRect(x - width / 2 - 4, y - 14, width + 8, 15);
  ctx.fillStyle = '#dbe6f5';
  ctx.fillText(text, x, y);
  ctx.restore();
}
