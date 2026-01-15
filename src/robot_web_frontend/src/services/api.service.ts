import axios from 'axios';
import { ROBOT_CONFIG } from '../config/robot.config';

const api = axios.create({
  baseURL: ROBOT_CONFIG.API_BASE_URL,
  timeout: 10000,
});

export interface NavigationGoal {
  x: number;
  y: number;
  yaw_deg: number;
}

export interface MapInfo {
  name: string;
  yaml_path: string;
  pgm_path: string;
}

export interface MapsListResponse {
  maps: MapInfo[];
  default: string;
}

export const apiService = {
  // Maps
  async getMaps(): Promise<MapsListResponse> {
    const response = await api.get('/maps/list');
    return response.data;
  },

  // Navigation
  async startNavigation(mapName?: string): Promise<void> {
    await api.post('/navigation/start', mapName ? { map_name: mapName } : {});
  },

  async stopNavigation(): Promise<void> {
    await api.post('/navigation/stop');
  },

  async navigateToGoal(goal: NavigationGoal): Promise<void> {
    await api.post('/navigate_to_goal', goal);
  },

  async cancelNavigation(): Promise<void> {
    await api.post('/navigation/cancel');
  },

  // SLAM
  async startMapping(): Promise<void> {
    await api.post('/slam/start');
  },

  async stopMapping(): Promise<void> {
    await api.post('/slam/stop');
  },

  async saveMap(mapName: string): Promise<{ map_path: string; files: string[] }> {
    const response = await api.post('/slam/save_map', { map_name: mapName });
    return response.data;
  },
};
