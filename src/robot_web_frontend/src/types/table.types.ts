/**
 * 桌位相關類型定義
 */

export interface Table {
  id: string;
  number: number;        // 顯示桌號 (1, 2, 3...)
  name?: string;         // 自訂名稱（可選）
  x: number;             // 地圖座標
  y: number;
  yaw_deg: number;       // 接近角度
  isActive: boolean;     // 是否啟用
  created_at: string;
  updated_at: string;
}

export interface TableCreate {
  number: number;
  name?: string;
  x: number;
  y: number;
  yaw_deg: number;
  isActive?: boolean;
}

export interface TableUpdate {
  number?: number;
  name?: string;
  x?: number;
  y?: number;
  yaw_deg?: number;
  isActive?: boolean;
}
