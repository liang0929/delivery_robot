// PointsPage：地圖上按下新增點位工具後，用來填寫名稱 / 類型 / 朝向的草稿面板。

import { normalizeDegrees } from '../../lib/coords';
import type { DraftPoint } from '../../hooks/usePointsEditor';
import type { PointType } from '../../api/types';

export interface DraftPointPanelProps {
  draftPoint: DraftPoint;
  busy: string | null;
  onChange: (next: DraftPoint) => void;
  onCreate: () => void;
  onCancel: () => void;
}

export function DraftPointPanel({
  draftPoint,
  busy,
  onChange,
  onCreate,
  onCancel,
}: DraftPointPanelProps) {
  return (
    <section className="panel">
      <h2 className="panelTitle">新點位</h2>
      <div className="field">
        <label className="fieldLabel">名稱</label>
        <input
          className="input"
          autoFocus
          value={draftPoint.name}
          onChange={(e) => onChange({ ...draftPoint, name: e.target.value })}
          onKeyDown={(e) => {
            if (e.key === 'Enter') onCreate();
          }}
        />
      </div>
      <div className="row" style={{ marginTop: 8 }}>
        <div className="field" style={{ flex: 1 }}>
          <label className="fieldLabel">類型</label>
          <select
            className="select"
            value={draftPoint.type}
            onChange={(e) =>
              onChange({
                ...draftPoint,
                type: e.target.value as PointType,
              })
            }
          >
            <option value="point">一般點位 (point)</option>
            <option value="charge">充電座 (charge)</option>
          </select>
        </div>
        <div className="field" style={{ width: 110 }}>
          <label className="fieldLabel">朝向 (°)</label>
          <input
            className="input"
            type="number"
            value={Math.round(draftPoint.location.orientation)}
            onChange={(e) =>
              onChange({
                ...draftPoint,
                location: {
                  ...draftPoint.location,
                  orientation: normalizeDegrees(Number(e.target.value) || 0),
                },
              })
            }
          />
        </div>
      </div>
      <p className="hintText" style={{ marginTop: 8 }}>
        座標 {draftPoint.location.x}, {draftPoint.location.y} cm
      </p>
      <div className="row" style={{ marginTop: 10 }}>
        <button
          type="button"
          className="btn primary"
          onClick={onCreate}
          disabled={!draftPoint.name.trim() || busy !== null}
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
