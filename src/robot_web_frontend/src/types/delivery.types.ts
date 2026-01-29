/**
 * 送餐任務相關類型定義
 */

export type DeliveryStatus =
  | 'idle'
  | 'delivering'
  | 'at_table'
  | 'returning'
  | 'stuck';

export type StopStatus =
  | 'pending'
  | 'in_progress'
  | 'arrived'
  | 'completed'
  | 'skipped';

export interface DeliveryStop {
  tableId: string;
  tableNumber: number;
  tableName?: string;
  status: StopStatus;
}

export interface Position {
  x: number;
  y: number;
  yaw: number;
}

export interface DeliveryTask {
  id: string;
  stops: DeliveryStop[];
  status: DeliveryStatus;
  currentStopIndex: number;
  startPosition: Position;
  createdAt: string;
}

export interface DeliveryStartRequest {
  tableIds: string[];
  startPosition: Position;
  mapName?: string;
}

export interface DeliveryStatusResponse {
  task: DeliveryTask | null;
  distanceRemaining: number | null;
  isStuck?: boolean;
}
