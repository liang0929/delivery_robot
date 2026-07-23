// PointsPage：地圖上拖出一條線段後，用來填寫虛擬牆名稱的草稿面板。

import type { DraftWall } from '../../hooks/usePointsEditor';

export interface DraftWallPanelProps {
  draftWall: DraftWall;
  busy: string | null;
  onChange: (next: DraftWall) => void;
  onCreate: () => void;
  onCancel: () => void;
}

export function DraftWallPanel({
  draftWall,
  busy,
  onChange,
  onCreate,
  onCancel,
}: DraftWallPanelProps) {
  return (
    <section className="panel">
      <h2 className="panelTitle">新虛擬牆</h2>
      <div className="field">
        <label className="fieldLabel">名稱（留空自動命名）</label>
        <input
          className="input"
          autoFocus
          value={draftWall.name}
          onChange={(e) => onChange({ ...draftWall, name: e.target.value })}
          onKeyDown={(e) => {
            if (e.key === 'Enter') onCreate();
          }}
        />
      </div>
      <p className="hintText" style={{ marginTop: 8 }}>
        ({draftWall.start.x}, {draftWall.start.y}) → ({draftWall.end.x},{' '}
        {draftWall.end.y}) cm
      </p>
      <div className="row" style={{ marginTop: 10 }}>
        <button
          type="button"
          className="btn primary"
          onClick={onCreate}
          disabled={busy !== null}
        >
          建立
        </button>
        <button type="button" className="btn" onClick={onCancel}>
          取消
        </button>
      </div>
    </section>
  );
}
