// PointsPage：側欄點位清單中的單一列，支援原地編輯名稱 / 類型 / 朝向。

import { useEffect, useState } from 'react';
import type { PointType, RobotPoint } from '../../api/types';

export interface PointRowProps {
  point: RobotPoint;
  selected: boolean;
  busy: boolean;
  onSelect: () => void;
  onSave: (point: RobotPoint, name: string, type: PointType) => void;
  onOrientation: (point: RobotPoint, orientation: number) => void;
  onDelete: (point: RobotPoint) => void;
}

export function PointRow({
  point,
  selected,
  busy,
  onSelect,
  onSave,
  onOrientation,
  onDelete,
}: PointRowProps) {
  const [editing, setEditing] = useState(false);
  const [name, setName] = useState(point.name);
  const [type, setType] = useState<PointType>(point.type);
  const [orientation, setOrientation] = useState(
    String(Math.round(point.location.orientation)),
  );

  // 外部資料重新載入後同步回編輯欄位
  useEffect(() => {
    setName(point.name);
    setType(point.type);
    setOrientation(String(Math.round(point.location.orientation)));
  }, [point.name, point.type, point.location.orientation]);

  if (!editing) {
    return (
      <li
        className={`listItem ${selected ? 'selected' : ''}`}
        onClick={onSelect}
      >
        <div className="listItemMain">
          <div className="listItemName">
            {point.name}{' '}
            {point.type === 'charge' && <span className="tag charge">charge</span>}
          </div>
          <div className="listItemMeta">
            {point.location.x}, {point.location.y} cm @{' '}
            {Math.round(point.location.orientation)}°
          </div>
        </div>
        <button
          type="button"
          className="btn small"
          onClick={(e) => {
            e.stopPropagation();
            setEditing(true);
          }}
        >
          編輯
        </button>
        <button
          type="button"
          className="btn small danger"
          disabled={busy}
          onClick={(e) => {
            e.stopPropagation();
            onDelete(point);
          }}
        >
          刪除
        </button>
      </li>
    );
  }

  const commitRow = () => {
    if (name.trim() !== point.name || type !== point.type) {
      onSave(point, name, type);
    }
    const deg = Number(orientation);
    if (
      Number.isFinite(deg) &&
      Math.round(deg) !== Math.round(point.location.orientation)
    ) {
      onOrientation(point, deg);
    }
    setEditing(false);
  };

  return (
    <li className={`listItem ${selected ? 'selected' : ''}`}>
      <div className="listItemMain" style={{ display: 'grid', gap: 6 }}>
        <input
          className="input"
          value={name}
          autoFocus
          onChange={(e) => setName(e.target.value)}
        />
        <div className="row">
          <select
            className="select"
            style={{ flex: 1 }}
            value={type}
            onChange={(e) => setType(e.target.value as PointType)}
          >
            <option value="point">point</option>
            <option value="charge">charge</option>
          </select>
          <input
            className="input"
            style={{ width: 80 }}
            type="number"
            value={orientation}
            onChange={(e) => setOrientation(e.target.value)}
          />
        </div>
      </div>
      <button
        type="button"
        className="btn small primary"
        disabled={busy || !name.trim()}
        onClick={commitRow}
      >
        儲存
      </button>
      <button
        type="button"
        className="btn small"
        onClick={() => {
          setName(point.name);
          setType(point.type);
          setOrientation(String(Math.round(point.location.orientation)));
          setEditing(false);
        }}
      >
        取消
      </button>
    </li>
  );
}
