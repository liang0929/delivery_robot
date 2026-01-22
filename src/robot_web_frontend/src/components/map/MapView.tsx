import { useRef, useEffect, useCallback, useState } from 'react';
import { rosbridgeService, OccupancyGridData } from '../../services/rosbridge.service';
import { Waypoint } from '../../services/api.service';
import styles from './MapView.module.css';

export type MapInteractionMode = 'navigate' | 'add_waypoint';

interface MapViewProps {
  onClickGoal?: (x: number, y: number, yaw: number) => void;
  showGoalSelector?: boolean;
  waypoints?: Waypoint[];
  selectedWaypointId?: string | null;
  mode?: MapInteractionMode;
}

interface RobotPose {
  x: number;
  y: number;
  yaw: number;
}

export function MapView({
  onClickGoal,
  showGoalSelector = false,
  waypoints = [],
  selectedWaypointId = null,
  mode = 'navigate',
}: MapViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [mapData, setMapData] = useState<OccupancyGridData | null>(null);
  const [robotPose, setRobotPose] = useState<RobotPose>({ x: 0, y: 0, yaw: 0 });
  const [scale, setScale] = useState(4);
  const [offset, setOffset] = useState({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState(false);
  const [dragStart, setDragStart] = useState({ x: 0, y: 0 });
  const [goalMarker, setGoalMarker] = useState<{ x: number; y: number } | null>(null);

  // Subscribe to map and odom with retry mechanism
  useEffect(() => {
    let retryInterval: number | null = null;
    let subscribed = false;

    const trySubscribe = () => {
      if (!rosbridgeService.isConnected()) {
        return; // 等待下次重試
      }

      if (!subscribed) {
        rosbridgeService.subscribeToMap((map) => {
          setMapData(map);
        });

        rosbridgeService.subscribeToOdom((odom) => {
          const { position, orientation } = odom.pose.pose;
          const yaw = Math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
          );
          setRobotPose({ x: position.x, y: position.y, yaw });
        });

        subscribed = true;
        // 訂閱成功後停止重試
        if (retryInterval) {
          clearInterval(retryInterval);
          retryInterval = null;
        }
      }
    };

    // 立即嘗試訂閱
    trySubscribe();
    // 每秒重試直到成功
    retryInterval = window.setInterval(trySubscribe, 1000);

    return () => {
      if (retryInterval) {
        clearInterval(retryInterval);
      }
      rosbridgeService.unsubscribeFromMap();
      rosbridgeService.unsubscribeFromOdom();
    };
  }, []);

  // Convert map coordinates to canvas coordinates
  const mapToCanvas = useCallback((mapX: number, mapY: number) => {
    if (!mapData) return { x: 0, y: 0 };
    const { resolution, origin, height } = mapData.info;
    const canvasX = (mapX - origin.position.x) / resolution * scale + offset.x;
    const canvasY = (height - (mapY - origin.position.y) / resolution) * scale + offset.y;
    return { x: canvasX, y: canvasY };
  }, [mapData, scale, offset]);

  // Convert canvas coordinates to map coordinates
  const canvasToMap = useCallback((canvasX: number, canvasY: number) => {
    if (!mapData) return { x: 0, y: 0 };
    const { resolution, origin, height } = mapData.info;
    const mapX = (canvasX - offset.x) / scale * resolution + origin.position.x;
    const mapY = (height - (canvasY - offset.y) / scale) * resolution + origin.position.y;
    return { x: mapX, y: mapY };
  }, [mapData, scale, offset]);

  // Draw map
  const draw = useCallback(() => {
    const canvas = canvasRef.current;
    if (!canvas || !mapData) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const { width, height } = mapData.info;
    const data = mapData.data;

    // Clear canvas
    ctx.fillStyle = '#1a1a1a';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // Draw map
    const imageData = ctx.createImageData(width, height);
    for (let i = 0; i < data.length; i++) {
      const value = data[i];
      const idx = i * 4;

      if (value === -1) {
        // Unknown - dark gray
        imageData.data[idx] = 50;
        imageData.data[idx + 1] = 50;
        imageData.data[idx + 2] = 50;
      } else if (value === 0) {
        // Free - white
        imageData.data[idx] = 255;
        imageData.data[idx + 1] = 255;
        imageData.data[idx + 2] = 255;
      } else {
        // Occupied - black
        imageData.data[idx] = 0;
        imageData.data[idx + 1] = 0;
        imageData.data[idx + 2] = 0;
      }
      imageData.data[idx + 3] = 255;
    }

    // Create temp canvas for map
    const tempCanvas = document.createElement('canvas');
    tempCanvas.width = width;
    tempCanvas.height = height;
    const tempCtx = tempCanvas.getContext('2d')!;
    tempCtx.putImageData(imageData, 0, 0);

    // Draw map with scale and offset
    ctx.save();
    ctx.translate(offset.x, offset.y);
    ctx.scale(scale, scale);
    ctx.imageSmoothingEnabled = false;

    // Flip Y axis for ROS coordinate system
    ctx.translate(0, height);
    ctx.scale(1, -1);
    ctx.drawImage(tempCanvas, 0, 0);
    ctx.restore();

    // Draw robot
    const robotCanvas = mapToCanvas(robotPose.x, robotPose.y);
    ctx.save();
    ctx.translate(robotCanvas.x, robotCanvas.y);
    ctx.rotate(-robotPose.yaw + Math.PI / 2);

    // Robot body (triangle)
    ctx.beginPath();
    ctx.moveTo(0, -15);
    ctx.lineTo(-10, 10);
    ctx.lineTo(10, 10);
    ctx.closePath();
    ctx.fillStyle = '#00ff88';
    ctx.fill();
    ctx.strokeStyle = '#00aa55';
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.restore();

    // Draw goal marker
    if (goalMarker) {
      const goalCanvas = mapToCanvas(goalMarker.x, goalMarker.y);
      ctx.beginPath();
      ctx.arc(goalCanvas.x, goalCanvas.y, 10, 0, Math.PI * 2);
      ctx.fillStyle = 'rgba(255, 100, 100, 0.7)';
      ctx.fill();
      ctx.strokeStyle = '#ff4444';
      ctx.lineWidth = 2;
      ctx.stroke();

      // Draw X
      ctx.beginPath();
      ctx.moveTo(goalCanvas.x - 6, goalCanvas.y - 6);
      ctx.lineTo(goalCanvas.x + 6, goalCanvas.y + 6);
      ctx.moveTo(goalCanvas.x + 6, goalCanvas.y - 6);
      ctx.lineTo(goalCanvas.x - 6, goalCanvas.y + 6);
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 2;
      ctx.stroke();
    }

    // Draw waypoints
    waypoints.forEach((wp, index) => {
      const wpCanvas = mapToCanvas(wp.x, wp.y);
      const isSelected = wp.id === selectedWaypointId;

      // Waypoint marker (circle with number)
      ctx.beginPath();
      ctx.arc(wpCanvas.x, wpCanvas.y, isSelected ? 14 : 12, 0, Math.PI * 2);
      ctx.fillStyle = isSelected ? 'rgba(102, 170, 255, 0.9)' : 'rgba(255, 170, 102, 0.8)';
      ctx.fill();
      ctx.strokeStyle = isSelected ? '#66aaff' : '#ffaa66';
      ctx.lineWidth = isSelected ? 3 : 2;
      ctx.stroke();

      // Waypoint number
      ctx.fillStyle = '#000';
      ctx.font = 'bold 10px sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.fillText((index + 1).toString(), wpCanvas.x, wpCanvas.y);

      // Direction indicator
      const yawRad = wp.yaw_deg * Math.PI / 180;
      const arrowLength = 18;
      const arrowX = wpCanvas.x + Math.cos(-yawRad + Math.PI / 2) * arrowLength;
      const arrowY = wpCanvas.y + Math.sin(-yawRad + Math.PI / 2) * arrowLength;
      ctx.beginPath();
      ctx.moveTo(wpCanvas.x, wpCanvas.y);
      ctx.lineTo(arrowX, arrowY);
      ctx.strokeStyle = isSelected ? '#66aaff' : '#ffaa66';
      ctx.lineWidth = 2;
      ctx.stroke();
    });
  }, [mapData, robotPose, scale, offset, goalMarker, mapToCanvas, waypoints, selectedWaypointId]);

  // Draw effect
  useEffect(() => {
    draw();
  }, [draw]);

  // Zoom handler
  const handleWheel = useCallback((e: React.WheelEvent) => {
    e.preventDefault();
    const delta = e.deltaY > 0 ? 0.9 : 1.1;
    setScale((s) => Math.max(0.5, Math.min(20, s * delta)));
  }, []);

  // Pan handlers
  const handleMouseDown = useCallback((e: React.MouseEvent) => {
    if (e.button === 0 && !showGoalSelector) {
      setIsDragging(true);
      setDragStart({ x: e.clientX - offset.x, y: e.clientY - offset.y });
    }
  }, [offset, showGoalSelector]);

  const handleMouseMove = useCallback((e: React.MouseEvent) => {
    if (isDragging) {
      setOffset({ x: e.clientX - dragStart.x, y: e.clientY - dragStart.y });
    }
  }, [isDragging, dragStart]);

  const handleMouseUp = useCallback(() => {
    setIsDragging(false);
  }, []);

  // Goal/Waypoint selection handler
  const handleClick = useCallback((e: React.MouseEvent) => {
    if (!showGoalSelector || !onClickGoal || !mapData) return;

    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const canvasX = e.clientX - rect.left;
    const canvasY = e.clientY - rect.top;

    const mapPos = canvasToMap(canvasX, canvasY);

    // Only show goal marker in navigate mode
    if (mode === 'navigate') {
      setGoalMarker(mapPos);
    }

    // Calculate yaw towards robot's current direction (or 0)
    const yaw = 0;
    onClickGoal(mapPos.x, mapPos.y, yaw);
  }, [showGoalSelector, onClickGoal, mapData, canvasToMap, mode]);

  return (
    <div className={styles.container}>
      <canvas
        ref={canvasRef}
        width={600}
        height={400}
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
        <button onClick={() => { setScale(4); setOffset({ x: 0, y: 0 }); }}>Reset</button>
      </div>
      <div className={styles.info}>
        <span>Robot: ({robotPose.x.toFixed(2)}, {robotPose.y.toFixed(2)})</span>
        {mapData && <span>Map: {mapData.info.width}x{mapData.info.height}</span>}
      </div>
    </div>
  );
}
