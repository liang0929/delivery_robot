// PointsPage 頂部橫幅：顯示是否有未提交的點位 / 虛擬牆變更。

export interface DirtyBannerProps {
  dirty: boolean;
  busy: string | null;
  onCommit: () => void;
  onDiscard: () => void;
}

export function DirtyBanner({ dirty, busy, onCommit, onDiscard }: DirtyBannerProps) {
  if (!dirty) {
    return <div className="cleanBanner">目前沒有未提交的變更。</div>;
  }

  return (
    <div className="dirtyBanner">
      <strong>有未提交的變更</strong>
      <span>點位 / 虛擬牆的修改仍只存在記憶體，尚未寫入磁碟。</span>
      <span style={{ flex: 1 }} />
      <button
        type="button"
        className="btn primary"
        onClick={onCommit}
        disabled={busy !== null}
      >
        提交變更
      </button>
      <button
        type="button"
        className="btn danger"
        onClick={onDiscard}
        disabled={busy !== null}
      >
        捨棄變更
      </button>
    </div>
  );
}
