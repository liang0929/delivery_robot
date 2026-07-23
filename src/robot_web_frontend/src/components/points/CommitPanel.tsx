// PointsPage：側欄底部的交易面板（提交 / 捨棄），與頂部 DirtyBanner 功能相同，
// 提供另一個隨手可及的入口。

export interface CommitPanelProps {
  busy: string | null;
  onCommit: () => void;
  onDiscard: () => void;
}

export function CommitPanel({ busy, onCommit, onDiscard }: CommitPanelProps) {
  return (
    <section className="panel">
      <h2 className="panelTitle">變更交易</h2>
      <p className="hintText">
        後端採暫存交易模型：未提交的變更只存在記憶體，磁碟永遠是已提交狀態。
      </p>
      <div className="row" style={{ marginTop: 10 }}>
        <button
          type="button"
          className="btn primary"
          onClick={onCommit}
          disabled={busy !== null}
        >
          提交變更 (commit)
        </button>
        <button
          type="button"
          className="btn danger"
          onClick={onDiscard}
          disabled={busy !== null}
        >
          捨棄變更 (discard)
        </button>
      </div>
    </section>
  );
}
