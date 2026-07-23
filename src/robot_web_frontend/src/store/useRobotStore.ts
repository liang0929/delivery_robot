// 全域狀態：WebSocket 連線、robot_info、事件 / 錯誤 toast、地圖清單與選取。
//
// robot_info 每秒一次；為了避免整棵樹每秒重繪，元件請用 selector
// 只訂閱自己需要的欄位（例如只要 location 的地圖圖層）。

import { create } from 'zustand';
import { RobotSocket, type ConnectionState } from '../ws/RobotSocket';
import { describeError } from '../api/client';
import { listMaps } from '../api/robot.api';
import { isRobotInfoMessage } from '../api/types';
import type { RobotInfo, RobotSocketMessage } from '../api/types';

export type ToastLevel = 'info' | 'success' | 'error';

export interface Toast {
  id: number;
  level: ToastLevel;
  title: string;
  detail?: string;
}

interface RobotState {
  connection: ConnectionState;
  /** 最後一次收到的 robot_info；重連期間保留舊值 */
  info: RobotInfo | null;
  /** 最後一次收到 robot_info 的時間戳（ms） */
  infoAt: number | null;
  toasts: Toast[];

  /** 已存檔地圖清單 */
  maps: string[];
  /** 目前選取的地圖；null 代表尚未選取 */
  selectedMap: string | null;
  mapsLoading: boolean;
  mapsError: string | null;

  startSocket: () => void;
  stopSocket: () => void;
  pushToast: (level: ToastLevel, title: string, detail?: string) => void;
  dismissToast: (id: number) => void;
  /** 重新拉取地圖清單；清單載入後若尚未選過就自動選第一張 */
  loadMaps: () => Promise<void>;
  /** 選取地圖；空字串視為取消選取 */
  selectMap: (name: string) => void;
}

let socket: RobotSocket | null = null;
let socketRefCount = 0;
let toastSeq = 0;
let mapsRequestId = 0;

const MAX_TOASTS = 5;

/** 事件 code → toast 等級 */
function levelForCode(code: string): ToastLevel {
  const upper = code.toUpperCase();
  if (upper === 'COMPLETE') return 'success';
  if (upper === 'STUCK' || upper === 'ABORT' || upper === 'CHG_STA_NOT_FOUND') {
    return 'error';
  }
  return 'info';
}

const EVENT_LABEL: Record<string, string> = {
  go_point: '導航',
  go_charging: '前往充電',
  switch_mode: '模式切換',
  relocate: '重定位',
  power: '電源',
};

function sameInfo(a: RobotInfo | null, b: RobotInfo): boolean {
  return (
    a !== null &&
    a.op_mode === b.op_mode &&
    a.status === b.status &&
    a.battery === b.battery &&
    a.voltage === b.voltage &&
    a.location.x === b.location.x &&
    a.location.y === b.location.y &&
    a.location.orientation === b.location.orientation
  );
}

export const useRobotStore = create<RobotState>((set, get) => ({
  connection: 'closed',
  info: null,
  infoAt: null,
  toasts: [],

  maps: [],
  selectedMap: null,
  mapsLoading: false,
  mapsError: null,

  startSocket: () => {
    socketRefCount += 1;
    if (socket) return;
    socket = new RobotSocket({
      onStateChange: (connection) => set({ connection }),
      onMessage: (msg: RobotSocketMessage) => {
        if (isRobotInfoMessage(msg)) {
          const { op_mode, status, battery, voltage, location } = msg;
          const next: RobotInfo = {
            op_mode,
            status,
            battery,
            voltage: voltage ?? null,
            location,
          };
          // 內容沒變就沿用舊物件：機器人靜止時不會每秒觸發一次重繪
          if (sameInfo(get().info, next)) {
            set({ infoAt: Date.now() });
            return;
          }
          set({ info: next, infoAt: Date.now() });
          return;
        }
        const label = EVENT_LABEL[msg.event] ?? msg.event;
        const code = String(msg.code ?? '');
        get().pushToast(levelForCode(code), label, code);
      },
    });
    socket.start();
  },

  stopSocket: () => {
    socketRefCount = Math.max(0, socketRefCount - 1);
    if (socketRefCount > 0) return;
    socket?.stop();
    socket = null;
  },

  pushToast: (level, title, detail) => {
    toastSeq += 1;
    const toast: Toast = { id: toastSeq, level, title, detail };
    set((s) => ({ toasts: [...s.toasts, toast].slice(-MAX_TOASTS) }));
  },

  dismissToast: (id) =>
    set((s) => ({ toasts: s.toasts.filter((t) => t.id !== id) })),

  loadMaps: async () => {
    const id = ++mapsRequestId;
    set({ mapsLoading: true, mapsError: null });
    try {
      const list = await listMaps();
      if (id !== mapsRequestId) return;
      set((s) => ({
        maps: list,
        mapsLoading: false,
        // 地圖清單載入後，若尚未選過就自動選第一張
        selectedMap:
          s.selectedMap === null && list.length > 0 ? list[0] : s.selectedMap,
      }));
    } catch (err) {
      if (id !== mapsRequestId) return;
      set({ mapsLoading: false, mapsError: describeError(err) });
    }
  },

  selectMap: (name) => set({ selectedMap: name === '' ? null : name }),
}));

/** 非 React 環境（例如 catch 區塊工具函式）也能推 toast */
export const pushToast = (level: ToastLevel, title: string, detail?: string) =>
  useRobotStore.getState().pushToast(level, title, detail);
