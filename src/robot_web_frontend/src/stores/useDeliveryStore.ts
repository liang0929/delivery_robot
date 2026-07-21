import { create } from 'zustand';
import type {
  DeliveryStatus,
  DeliveryStop,
  Position,
  DeliveryTask,
} from '../types/delivery.types';
import type { Table } from '../types/table.types';
import { apiService } from '../services/api.service';

interface DeliveryState {
  // 任務隊列
  stops: DeliveryStop[];
  currentStopIndex: number;
  mapName: string | null;

  // 狀態
  status: DeliveryStatus;
  distanceRemaining: number | null;
  startPosition: Position | null;
  currentTask: DeliveryTask | null;

  // 錯誤處理
  error: string | null;
  isLoading: boolean;

  // Actions
  setMapName: (mapName: string) => void;
  addStop: (table: Table) => void;
  removeStop: (index: number) => void;
  reorderStops: (fromIndex: number, toIndex: number) => void;
  clearStops: () => void;

  startDelivery: () => Promise<void>;
  confirmArrival: () => Promise<void>;
  skipTable: () => Promise<void>;
  cancelDelivery: () => Promise<void>;
  retryDelivery: () => Promise<void>;

  refreshStatus: () => Promise<void>;
  resetState: () => void;
}

const initialState = {
  stops: [] as DeliveryStop[],
  currentStopIndex: 0,
  mapName: null as string | null,
  status: 'idle' as DeliveryStatus,
  distanceRemaining: null,
  startPosition: null,
  currentTask: null,
  error: null,
  isLoading: false,
};

// 輪詢世代序號：mutation（confirm/skip/cancel/start/retry）開始與完成時遞增，
// 使較早發出、較晚返回的 /delivery/status 輪詢回應被丟棄，
// 避免 out-of-order 回應把新狀態蓋回舊狀態（如已確認到達卻閃回確認彈窗）
let statusEpoch = 0;
const invalidateStatusPolls = () => {
  statusEpoch++;
};

export const useDeliveryStore = create<DeliveryState>((set, get) => ({
  ...initialState,

  setMapName: (mapName: string) => {
    set({ mapName });
  },

  addStop: (table: Table) => {
    const { stops, status } = get();

    // 不允許在送餐中添加
    if (status !== 'idle') {
      return;
    }

    // 檢查是否已存在
    if (stops.some((s) => s.tableId === table.id)) {
      return;
    }

    const newStop: DeliveryStop = {
      tableId: table.id,
      tableNumber: table.number,
      tableName: table.name,
      status: 'pending',
    };

    set({ stops: [...stops, newStop] });
  },

  removeStop: (index: number) => {
    const { stops, status } = get();

    if (status !== 'idle') {
      return;
    }

    set({
      stops: stops.filter((_, i) => i !== index),
    });
  },

  reorderStops: (fromIndex: number, toIndex: number) => {
    const { stops, status } = get();

    if (status !== 'idle') {
      return;
    }

    const newStops = [...stops];
    const [removed] = newStops.splice(fromIndex, 1);
    newStops.splice(toIndex, 0, removed);

    set({ stops: newStops });
  },

  clearStops: () => {
    const { status } = get();

    if (status !== 'idle') {
      return;
    }

    set({ stops: [] });
  },

  startDelivery: async () => {
    const { stops, status, mapName } = get();

    if (status !== 'idle' || stops.length === 0) {
      return;
    }

    invalidateStatusPolls();
    set({ isLoading: true, error: null });

    try {
      // 先取得當前位置
      const position = await apiService.getCurrentPosition();
      const startPosition: Position = {
        x: position.x,
        y: position.y,
        yaw: position.yaw,
      };

      // 開始送餐任務
      const task = await apiService.startDelivery({
        tableIds: stops.map((s) => s.tableId),
        startPosition,
        mapName: mapName || undefined,
      });

      invalidateStatusPolls();
      set({
        status: 'delivering',
        currentTask: task,
        startPosition,
        currentStopIndex: task.currentStopIndex,
        stops: task.stops,
        isLoading: false,
      });
    } catch (error) {
      const message = error instanceof Error ? error.message : '開始送餐失敗';
      set({ error: message, isLoading: false });
      throw error;
    }
  },

  confirmArrival: async () => {
    const { status } = get();

    if (status !== 'at_table') {
      return;
    }

    invalidateStatusPolls();
    set({ isLoading: true, error: null });

    try {
      const task = await apiService.confirmArrival();

      invalidateStatusPolls();
      set({
        currentTask: task,
        status: task.status,
        currentStopIndex: task.currentStopIndex,
        stops: task.stops,
        isLoading: false,
      });

      // 如果任務完成，重置狀態
      if (task.status === 'idle') {
        get().resetState();
      }
    } catch (error) {
      const message = error instanceof Error ? error.message : '確認到達失敗';
      set({ error: message, isLoading: false });
      throw error;
    }
  },

  skipTable: async () => {
    const { status } = get();

    if (status !== 'at_table') {
      return;
    }

    invalidateStatusPolls();
    set({ isLoading: true, error: null });

    try {
      const task = await apiService.skipTable();

      invalidateStatusPolls();
      set({
        currentTask: task,
        status: task.status,
        currentStopIndex: task.currentStopIndex,
        stops: task.stops,
        isLoading: false,
      });

      // 如果任務完成，重置狀態
      if (task.status === 'idle') {
        get().resetState();
      }
    } catch (error) {
      const message = error instanceof Error ? error.message : '跳過桌位失敗';
      set({ error: message, isLoading: false });
      throw error;
    }
  },

  cancelDelivery: async () => {
    const { status } = get();

    if (status === 'idle') {
      return;
    }

    invalidateStatusPolls();
    set({ isLoading: true, error: null });

    try {
      await apiService.cancelDelivery();
      invalidateStatusPolls();
      get().resetState();
    } catch (error) {
      const message = error instanceof Error ? error.message : '取消送餐失敗';
      set({ error: message, isLoading: false });
      throw error;
    }
  },

  retryDelivery: async () => {
    const { status } = get();

    // 僅在卡住時允許重試
    if (status !== 'stuck') {
      return;
    }

    invalidateStatusPolls();
    set({ isLoading: true, error: null });

    try {
      const task = await apiService.retryDelivery();

      invalidateStatusPolls();
      set({
        currentTask: task,
        status: task.status,
        currentStopIndex: task.currentStopIndex,
        stops: task.stops,
        isLoading: false,
      });
    } catch (error) {
      const message = error instanceof Error ? error.message : '重試導航失敗';
      set({ error: message, isLoading: false });
      throw error;
    }
  },

  refreshStatus: async () => {
    // 記錄發出請求時的世代；回應返回時若世代已變（期間有 mutation），此回應視為過期
    const epoch = statusEpoch;
    try {
      const response = await apiService.getDeliveryStatus();

      if (epoch !== statusEpoch) {
        return; // 過期回應，丟棄
      }
      if (get().isLoading) {
        return; // mutation 進行中，不套用輪詢結果
      }

      if (response.task) {
        set({
          currentTask: response.task,
          status: response.task.status,
          currentStopIndex: response.task.currentStopIndex,
          stops: response.task.stops,
          distanceRemaining: response.distanceRemaining,
        });
      } else {
        // 沒有進行中的任務
        set({
          status: 'idle',
          distanceRemaining: null,
        });
      }
    } catch (error) {
      // 靜默處理錯誤，不影響 UI
      console.error('刷新送餐狀態失敗:', error);
    }
  },

  resetState: () => {
    // 保留 mapName：它來自頁面選擇，任務完成後仍需沿用，
    // 否則下一趟任務會以 undefined mapName 落到後端預設地圖
    set({
      ...initialState,
      mapName: get().mapName,
    });
  },
}));
