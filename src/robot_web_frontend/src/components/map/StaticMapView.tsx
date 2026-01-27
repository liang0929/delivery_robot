import { useRef, useEffect, useCallback, useState } from 'react';
import { apiService, MapMetadata } from '../../services/api.service';
import type { Table } from '../../types/table.types';
import styles from './MapView.module.css';

export type StaticMapInteractionMode = 'view' | 'add_table' | 'edit_table';

const CANVAS_WIDTH = 600;
const CANVAS_HEIGHT = 400;
const FIT_PADDING = 20;

interface StaticMapViewProps {
  mapName: string;
  tables?: Table[];
  selectedTableId?: string | null;
  mode?: StaticMapInteractionMode;
  onMapClick?: (x: number, y: number, yaw: number) => void;
  onTableClick?: (table: Table) => void;
}

export function StaticMapView({
  mapName,
  tables = [],
  selectedTableId = null,
  mode = 'view',
  onMapClick,
  onTableClick,
}: StaticMapViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [mapImage, setMapImage] = useState<HTMLImageElement | null>(null);
  const [metadata, setMetadata] = useState<MapMetadata | null>(null);
  const [scale, setScale] = useState(1);
  const [offset, setOffset] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
  const [isLoading, setIsLoading] = useState(true);
  const [error, setError] = useState<string | null>(null);

  // 載入地圖圖片和 metadata
  useEffect(() => {
    if (!mapName) return;

    setIsLoading(true);
    setError(null);

    const loadMap = async () => {
      try {
        // 載入 metadata
        const meta = await apiService.getMapMetadata(mapName);
        setMetadata(meta);

        // 載入圖片
        const img = new Image();
        img.crossOrigin = 'anonymous';
        img.onload = () => {
          setMapImage(img);
          setIsLoading(false);

          // 自動適配
          const availableWidth = CANVAS_WIDTH - FIT_PADDING * 2;
          const availableHeight = CANVAS_HEIGHT - FIT_PADDING * 2;
          const scaleX = availableWidth / img.width;
          const scaleY = availableHeight / img.height;
          const fitScale = Math.min(scaleX, scaleY);
          const scaledWidth = img.width * fitScale;
          const scaledHeight = img.height * fitScale;
          const fitOffsetX = (CANVAS_WIDTH - scaledWidth) / 2;
          const fitOffsetY = (CANVAS_HEIGHT - scaledHeight) / 2;

          setScale(fitScale);
          setOffset({ x: fitOffsetX, y: fitOffsetY });
        };
        img.onerror = () => {
          setError('Failed to load map image');
          setIsLoading(false);
        };
        img.src = apiService.getMapImageUrl(mapName);
      } catch (err) {
        setError('Failed to load map metadata');
        setIsLoading(false);
      }
    };

    loadMap();
  }, [mapName]);

  // 地圖座標轉 canvas 座標
  const mapToCanvas = useCallback((mapX: number, mapY: number) => {
    if (!metadata || !mapImage) return { x: 0, y: 0 };
    const { resolution, origin } = metadata;
    const canvasX = (mapX - origin[0]) / resolution * scale + offset.x;
    const canvasY = (mapImage.height - (mapY - origin[1]) / resolution) * scale + offset.y;
    return { x: canvasX, y: canvasY };
  }, [metadata, mapImage, scale, offset]);

  // Canvas 座標轉地圖座標
  const canvasToMap = useCallback((canvasX: number, canvasY: number) => {
    if (!metadata || !mapImage) return { x: 0, y: 0 };
    const { resolution, origin } = metadata;
    const mapX = (canvasX - offset.x) / scale * resolution + origin[0];
    const mapY = (mapImage.height - (canvasY - offset.y) / scale) * resolution + origin[1];
    return { x: mapX, y: mapY };
  }, [metadata, mapImage, scale, offset]);

  // 繪製地圖
  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas || !mapImage) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // 清除畫布
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // 繪製地圖
    ctx.save();
    ctx.translate(offset.x, offset.y);
    ctx.scale(scale, scale);
    ctx.imageSmoothingEnabled = false;

    // PGM 地圖需要翻轉 Y 軸
    ctx.translate(0, mapImage.height);
    ctx.scale(1, -1);
    ctx.drawImage(mapImage, 0, 0);
    ctx.restore();

    // 繪製桌位
    tables.forEach((table) => {
      const tableCanvas = mapToCanvas(table.x, table.y);
      const isSelected = table.id === selectedTableId;
      const isActive = table.isActive;

      // 桌位標記（圓角矩形）
      const size = isSelected ? 24 : 20;
      const halfSize = size / 2;

      ctx.beginPath();
      ctx.roundRect(
        tableCanvas.x - halfSize,
        tableCanvas.y - halfSize,
        size,
        size,
        4
      );

      if (!isActive) {
        ctx.fillStyle = 'rgba(100, 100, 100, 0.6)';
        ctx.strokeStyle = '#666';
      } else if (isSelected) {
        ctx.fillStyle = 'rgba(0, 255, 136, 0.9)';
        ctx.strokeStyle = '#00ff88';
      } else {
        ctx.fillStyle = 'rgba(0, 200, 100, 0.7)';
        ctx.strokeStyle = '#00cc66';
      }

      ctx.fill();
      ctx.lineWidth = isSelected ? 3 : 2;
      ctx.stroke();

      // 桌號
      ctx.fillStyle = isActive ? '#000' : '#888';
      ctx.font = 'bold 12px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText(table.number.toString(), tableCanvas.x, tableCanvas.y);

      // 方向指示
      const yawRad = table.yaw_deg * Math.PI / 180;
      const arrowLength = 16;
      const arrowX = tableCanvas.x + Math.cos(-yawRad + Math.PI / 2) * arrowLength;
      const arrowY = tableCanvas.y + Math.sin(-yawRad + Math.PI / 2) * arrowLength;
      ctx.beginPath();
      ctx.moveTo(tableCanvas.x, tableCanvas.y);
      ctx.lineTo(arrowX, arrowY);
      ctx.strokeStyle = isActive ? (isSelected ? '#00ff88' : '#00cc66') : '#666';
      ctx.lineWidth = 2;
      ctx.stroke();
    });
  }, [mapImage, scale, offset, tables, selectedTableId, mapToCanvas]);

  // 繪製效果
  useEffect(() => {
    draw();
  }, [draw]);

  // 找到點擊的桌位
  const findClickedTable = useCallback((canvasX: number, canvasY: number): Table | null => {
    for (const table of tables) {
      const tableCanvas = mapToCanvas(table.x, table.y);
      const size = 24;
      const halfSize = size / 2;

      if (
        canvasX >= tableCanvas.x - halfSize &&
        canvasX <= tableCanvas.x + halfSize &&
        canvasY >= tableCanvas.y - halfSize &&
        canvasY <= tableCanvas.y + halfSize
      ) {
        return table;
      }
    }
    return null;
  }, [tables, mapToCanvas]);

  // 滾輪縮放
  const handleWheel = useCallback((e: React.WheelEvent) => {
    e.preventDefault();
    const delta = e.deltaY > 0 ? 0.9 : 1.1;
    setScale((s) => Math.max(0.5, Math.min(20, s * delta)));
  }, []);

  // 拖曳平移
  const handleMouseDown = useCallback((e: React.MouseEvent) => {
    if (e.button === 0 && mode === 'view') {
      setIsDragging(true);
      setDragStart({ x: e.clientX - offset.x, y: e.clientY - offset.y });
    }
  }, [offset, mode]);

  const handleMouseMove = useCallback((e: React.MouseEvent) => {
    if (isDragging) {
      setOffset({ x: e.clientX - dragStart.x, y: e.clientY - dragStart.y });
    }
  }, [isDragging, dragStart]);

  const handleMouseUp = useCallback(() => {
    setIsDragging(false);
  }, []);

  // 點擊處理
  const handleClick = useCallback((e: React.MouseEvent) => {
    const canvas = canvasRef.current;
    if (!canvas || !mapImage || !metadata) return;

    const rect = canvas.getBoundingClientRect();
    const canvasX = e.clientX - rect.left;
    const canvasY = e.clientY - rect.top;

    // 檢查是否點擊桌位
    if (mode === 'edit_table' && onTableClick) {
      const clickedTable = findClickedTable(canvasX, canvasY);
      if (clickedTable) {
        onTableClick(clickedTable);
        return;
      }
    }

    // 處理地圖點擊（添加桌位）
    if (mode === 'add_table' && onMapClick) {
      const mapPos = canvasToMap(canvasX, canvasY);
      onMapClick(mapPos.x, mapPos.y, 0);
    }
  }, [mapImage, metadata, mode, onMapClick, onTableClick, canvasToMap, findClickedTable]);

  // 自動適配
  const fitToView = useCallback(() => {
    if (!mapImage) return;

    const availableWidth = CANVAS_WIDTH - FIT_PADDING * 2;
    const availableHeight = CANVAS_HEIGHT - FIT_PADDING * 2;
    const scaleX = availableWidth / mapImage.width;
    const scaleY = availableHeight / mapImage.height;
    const fitScale = Math.min(scaleX, scaleY);
    const scaledWidth = mapImage.width * fitScale;
    const scaledHeight = mapImage.height * fitScale;
    const fitOffsetX = (CANVAS_WIDTH - scaledWidth) / 2;
    const fitOffsetY = (CANVAS_HEIGHT - scaledHeight) / 2;

    setScale(fitScale);
    setOffset({ x: fitOffsetX, y: fitOffsetY });
  }, [mapImage]);

  if (isLoading) {
    return (
      <div className={styles.container}>
        <div className={styles.loading}>Loading map...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className={styles.container}>
        <div className={styles.error}>{error}</div>
      </div>
    );
  }

  return (
    <div className={styles.container}>
      <canvas
        ref={canvasRef}
        width={CANVAS_WIDTH}
        height={CANVAS_HEIGHT}
        className={styles.canvas}
        onWheel={handleWheel}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        onMouseLeave={handleMouseUp}
        onClick={handleClick}
      />
      <div className={styles.controls}>
        <button onClick={() => setScale((s) => Math.min(20, s * 1.2))}>+</button>
        <button onClick={() => setScale((s) => Math.max(0.5, s / 1.2))}>-</button>
        <button onClick={fitToView}>Fit</button>
      </div>
      <div className={styles.info}>
        {metadata && <span>Map: {metadata.width}x{metadata.height}</span>}
        <span>Tables: {tables.length}</span>
      </div>
    </div>
  );
}
