// PointsPage：側欄的點位清單（含每列的原地編輯 / 刪除）。

import { PointRow } from './PointRow';
import type { PointType, RobotPoint } from '../../api/types';

export interface PointListProps {
  points: RobotPoint[];
  selectedId: string | null;
  busy: string | null;
  onSelect: (id: string) => void;
  onSave: (point: RobotPoint, name: string, type: PointType) => void;
  onOrientation: (point: RobotPoint, orientation: number) => void;
  onDelete: (point: RobotPoint) => void;
}

export function PointList({
  points,
  selectedId,
  busy,
  onSelect,
  onSave,
  onOrientation,
  onDelete,
}: PointListProps) {
  if (points.length === 0) {
    return <p className="hintText">尚未建立任何點位。</p>;
  }

  return (
    <ul className="list">
      {points.map((p) => (
        <PointRow
          key={p.id}
          point={p}
          selected={p.id === selectedId}
          busy={busy !== null}
          onSelect={() => onSelect(p.id)}
          onSave={onSave}
          onOrientation={onOrientation}
          onDelete={onDelete}
        />
      ))}
    </ul>
  );
}
