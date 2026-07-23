// 地圖選擇器：下拉選單 + 重新整理按鈕。
// PointsPage / NavigationPage 共用（MappingPage 用 live map，不經此元件）。

export interface MapPickerProps {
  maps: string[];
  selectedMap: string | null;
  onSelectMap: (name: string) => void;
  onReloadMaps: () => void;
}

export function MapPicker({
  maps,
  selectedMap,
  onSelectMap,
  onReloadMaps,
}: MapPickerProps) {
  return (
    <div className="row">
      <select
        className="select"
        style={{ flex: 1 }}
        value={selectedMap ?? ''}
        onChange={(e) => onSelectMap(e.target.value)}
      >
        <option value="">— 選擇地圖 —</option>
        {maps.map((m) => (
          <option key={m} value={m}>
            {m}
          </option>
        ))}
      </select>
      <button type="button" className="btn small" onClick={onReloadMaps}>
        重新整理
      </button>
    </div>
  );
}

/** 地圖區塊空狀態提示：未選地圖 > 載入中 > 錯誤（PointsPage / NavigationPage 共用） */
export function mapEmptyHint(
  selectedMap: string | null,
  loading: boolean,
  error: string | null,
): string {
  return !selectedMap
    ? '請先在右側選擇一張地圖'
    : loading
      ? '地圖載入中…'
      : (error ?? '地圖載入失敗');
}
