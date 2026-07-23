// PointsPage：切換「新增點位 / 新增虛擬牆 / 重設選取點位置」工具的按鈕列。

import type { MapMetadata, RobotPoint } from '../../api/types';
import type { Tool } from '../../hooks/usePointsEditor';

export interface ToolPanelProps {
  tool: Tool;
  toolHint: string;
  selectedMap: string | null;
  meta: MapMetadata | null;
  selectedPoint: RobotPoint | null;
  onToggleTool: (next: Tool) => void;
}

export function ToolPanel({
  tool,
  toolHint,
  selectedMap,
  meta,
  selectedPoint,
  onToggleTool,
}: ToolPanelProps) {
  return (
    <section className="panel">
      <h2 className="panelTitle">編輯工具</h2>
      <div className="row">
        <button
          type="button"
          className={`btn ${tool === 'add-point' ? 'active' : ''}`}
          disabled={!selectedMap || !meta}
          onClick={() => onToggleTool('add-point')}
        >
          新增點位
        </button>
        <button
          type="button"
          className={`btn ${tool === 'add-wall' ? 'active' : ''}`}
          disabled={!selectedMap || !meta}
          onClick={() => onToggleTool('add-wall')}
        >
          新增虛擬牆
        </button>
        <button
          type="button"
          className={`btn ${tool === 'move-point' ? 'active' : ''}`}
          disabled={!selectedPoint || !meta}
          onClick={() => onToggleTool('move-point')}
        >
          重設選取點位置
        </button>
      </div>
      <p className="hintText" style={{ marginTop: 8 }}>
        {toolHint}
      </p>
    </section>
  );
}
