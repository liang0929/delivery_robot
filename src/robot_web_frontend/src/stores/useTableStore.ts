import { create } from 'zustand';
import type { Table, TableCreate, TableUpdate } from '../types/table.types';
import { apiService } from '../services/api.service';

interface TableState {
  tables: Table[];
  selectedMap: string;
  isLoading: boolean;
  error: string | null;

  // Actions
  setSelectedMap: (mapName: string) => void;
  fetchTables: (mapName: string) => Promise<void>;
  createTable: (data: TableCreate) => Promise<Table>;
  updateTable: (id: string, data: TableUpdate) => Promise<Table>;
  deleteTable: (id: string) => Promise<void>;
  getTableById: (id: string) => Table | undefined;
  getActiveTables: () => Table[];
}

export const useTableStore = create<TableState>((set, get) => ({
  tables: [],
  selectedMap: '',
  isLoading: false,
  error: null,

  setSelectedMap: (mapName: string) => {
    set({ selectedMap: mapName });
  },

  fetchTables: async (mapName: string) => {
    set({ isLoading: true, error: null });
    try {
      const tables = await apiService.getTables(mapName);
      set({ tables, selectedMap: mapName, isLoading: false });
    } catch (error) {
      const message = error instanceof Error ? error.message : '載入桌位失敗';
      set({ error: message, isLoading: false });
      throw error;
    }
  },

  createTable: async (data: TableCreate) => {
    const { selectedMap } = get();
    if (!selectedMap) {
      throw new Error('尚未選擇地圖');
    }

    set({ isLoading: true, error: null });
    try {
      const newTable = await apiService.createTable(selectedMap, data);
      set((state) => ({
        tables: [...state.tables, newTable],
        isLoading: false,
      }));
      return newTable;
    } catch (error) {
      const message = error instanceof Error ? error.message : '新增桌位失敗';
      set({ error: message, isLoading: false });
      throw error;
    }
  },

  updateTable: async (id: string, data: TableUpdate) => {
    const { selectedMap } = get();
    if (!selectedMap) {
      throw new Error('尚未選擇地圖');
    }

    set({ isLoading: true, error: null });
    try {
      const updatedTable = await apiService.updateTable(selectedMap, id, data);
      set((state) => ({
        tables: state.tables.map((t) => (t.id === id ? updatedTable : t)),
        isLoading: false,
      }));
      return updatedTable;
    } catch (error) {
      const message = error instanceof Error ? error.message : '更新桌位失敗';
      set({ error: message, isLoading: false });
      throw error;
    }
  },

  deleteTable: async (id: string) => {
    const { selectedMap } = get();
    if (!selectedMap) {
      throw new Error('尚未選擇地圖');
    }

    set({ isLoading: true, error: null });
    try {
      await apiService.deleteTable(selectedMap, id);
      set((state) => ({
        tables: state.tables.filter((t) => t.id !== id),
        isLoading: false,
      }));
    } catch (error) {
      const message = error instanceof Error ? error.message : '刪除桌位失敗';
      set({ error: message, isLoading: false });
      throw error;
    }
  },

  getTableById: (id: string) => {
    return get().tables.find((t) => t.id === id);
  },

  getActiveTables: () => {
    return get().tables.filter((t) => t.isActive);
  },
}));
