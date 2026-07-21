// Winstec Robot API v1.1 資料模型（docs/winstec_api_v1.1.md §3）
// 注意：API 座標單位一律為「公分整數」，orientation 為「度」。

/** §7.1 Operating Mode */
export type OpMode = 'explore' | 'navigate';

/** §7.2 Robot Status */
export type RobotStatus =
  | 'init'
  | 'idle'
  | 'relocating'
  | 'moving'
  | 'go_charging'
  | 'switching_mode';

/** §7.3 Point Type */
export type PointType = 'point' | 'charge';

/** §7.4 Location Object — x/y 為公分整數，orientation 為度（0–360） */
export interface ApiLocation {
  x: number;
  y: number;
  orientation: number;
}

/** §7.5 Position Object — 虛擬牆端點，無 orientation */
export interface ApiPosition {
  x: number;
  y: number;
}

/** §7.6 Event Codes */
export type EventCode =
  | 'COMPLETE'
  | 'STUCK'
  | 'ABORT'
  | 'CHG_STA_NOT_FOUND'
  | 'SHUTDOWN';

export interface RobotInfo {
  op_mode: OpMode;
  status: RobotStatus;
  battery: number;
  location: ApiLocation;
}

export interface RobotPoint {
  id: string;
  map: string;
  name: string;
  type: PointType;
  location: ApiLocation;
}

export interface VirtualWall {
  id: string;
  map: string;
  name: string;
  start_position: ApiPosition;
  end_position: ApiPosition;
}

/** POST /manual/move 的方向值 */
export type ManualDirection = 'stop' | 'forward' | 'backward' | 'left' | 'right';

/** 地圖 metadata（擴充端點 GET /maps/{name}/metadata） */
export interface MapMetadata {
  /** 公尺 / 像素 */
  resolution: number;
  /** ROS map origin，公尺 [x, y, theta] */
  origin: [number, number, number];
  /** 影像寬（像素） */
  width: number;
  /** 影像高（像素） */
  height: number;
}

export interface MapSummary {
  name: string;
}

/** §6 錯誤回應：{ "event": { "code": "..." } } */
export interface ApiErrorBody {
  event?: { code?: string };
}

/** WebSocket 訊息 */
export interface RobotInfoMessage extends RobotInfo {
  event: 'robot_info';
}

export type RobotEventName =
  | 'go_point'
  | 'go_charging'
  | 'switch_mode'
  | 'relocate'
  | 'power';

export interface RobotEventMessage {
  event: RobotEventName;
  code: EventCode | string;
}

export type RobotSocketMessage = RobotInfoMessage | RobotEventMessage;

export function isRobotInfoMessage(
  msg: RobotSocketMessage,
): msg is RobotInfoMessage {
  return msg.event === 'robot_info';
}
