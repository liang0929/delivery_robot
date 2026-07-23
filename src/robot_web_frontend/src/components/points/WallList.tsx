// PointsPage：側欄的虛擬牆清單（含刪除）。

import type { VirtualWall } from '../../api/types';

export interface WallListProps {
  walls: VirtualWall[];
  selectedId: string | null;
  busy: string | null;
  onSelect: (id: string) => void;
  onDelete: (wall: VirtualWall) => void;
}

export function WallList({ walls, selectedId, busy, onSelect, onDelete }: WallListProps) {
  if (walls.length === 0) {
    return <p className="hintText">尚未建立任何虛擬牆。</p>;
  }

  return (
    <ul className="list">
      {walls.map((w) => (
        <li
          key={w.id}
          className={`listItem ${w.id === selectedId ? 'selected' : ''}`}
          onClick={() => onSelect(w.id)}
        >
          <div className="listItemMain">
            <div className="listItemName">{w.name}</div>
            <div className="listItemMeta">
              ({w.start_position.x}, {w.start_position.y}) → (
              {w.end_position.x}, {w.end_position.y})
            </div>
          </div>
          <button
            type="button"
            className="btn small danger"
            disabled={busy !== null}
            onClick={(e) => {
              e.stopPropagation();
              onDelete(w);
            }}
          >
            刪除
          </button>
        </li>
      ))}
    </ul>
  );
}
