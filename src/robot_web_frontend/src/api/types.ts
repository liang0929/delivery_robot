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

/**
 * 🟡 擴充：低電壓保護狀態，來自 battery_guard 的 /battery/state。
 * - `ok`       電壓正常
 * - `warning`  低於警告門檻，尚可行走
 * - `shutdown` 低電壓停機鎖存，需充電後重啟 battery_guard 才會解除
 * - `unknown`  沒有 battery_guard，或還沒收到資料
 */
export type BatteryState = 'ok' | 'warning' | 'shutdown' | 'unknown';

export interface RobotInfo {
  op_mode: OpMode;
  status: RobotStatus;
  battery: number;
  /** 電池母線電壓（V）。後端取不到 /motor/voltage 時為 null */
  voltage: number | null;
  /** 🟡 擴充：低電壓保護狀態。舊版後端不送此欄位，store 會補 'unknown' */
  battery_state: BatteryState;
  /** 🟡 擴充：停機鎖存中（機器人不會動）。舊版後端不送此欄位，store 會補 false */
  battery_stop_latched: boolean;
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

/** 尚未送出的暫定位姿（例如剛點下的新點位、拖曳中的預覽） */
export interface DraftPose {
  location: ApiLocation;
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

/**
 * §6 錯誤回應：{ "event": { "code": "..." } }
 *
 * `detail` 是擴充端點才會帶的人話說明（規格沒有）。錯誤碼是給程式判斷用的
 * 固定字串，但像「查不到 map→base_link」這種失敗，操作者需要知道的是
 * 「Nav2 沒起來還是沒定位」——沒有 detail 就只能顯示一串大寫英文。
 */
export interface ApiErrorBody {
  event?: { code?: string; detail?: string };
}

/** 平面位姿（ROS 單位：公尺 / 弧度，`yaw_deg` 是顯示用衍生值） */
export interface Pose2D {
  x_m: number;
  y_m: number;
  yaw_rad: number;
  yaw_deg: number;
}

/** POST /dock/record_pose 🟡 的回應 */
export interface DockPoseRecord {
  dock_id: string;
  /** 記錄所在的 frame；`map` 以外都是測試值 */
  frame: string;
  /** 寫進 dock_database.yaml 的值（已含 yaw + 180° 換算） */
  pose: Pose2D;
  /** 換算前查到的 base_link 位姿 */
  base_link: Pose2D;
  contact_offset_m: number;
  /** true＝非 map frame 的測試記錄，重開機後失效，不可當正式值 */
  test_only: boolean;
  recorded_at: string;
  database_path: string;
}

/**
 * WebSocket 訊息。
 *
 * `battery_state` / `battery_stop_latched` 在型別上是 optional：舊版後端
 * （battery_guard 上線前）不會送這兩個鍵，store 收下時補預設值。
 */
export interface RobotInfoMessage
  extends Omit<RobotInfo, 'battery_state' | 'battery_stop_latched'> {
  event: 'robot_info';
  battery_state?: BatteryState;
  battery_stop_latched?: boolean;
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
