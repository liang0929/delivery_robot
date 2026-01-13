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

export interface SlamStatus {
  status: 'idle' | 'mapping' | 'saving';
  is_mapping: boolean;
}

export interface NavigationStatus {
  is_complete: boolean;
  distance_remaining: number | null;
  nav_running?: boolean;
}

export interface RobotStatus {
  is_running: boolean;
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

  async getNavigationStatus(): Promise<NavigationStatus> {
    const response = await api.get('/navigation/status');
    return response.data;
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

  async getSlamStatus(): Promise<SlamStatus> {
    const response = await api.get('/slam/status');
    return response.data;
  },

  // Robot Core
  async startRobotCore(): Promise<void> {
    await api.post('/robot/start');
  },

  async stopRobotCore(): Promise<void> {
    await api.post('/robot/stop');
  },

  async startLidar(): Promise<void> {
    await api.post('/robot/start_lidar');
  },

  async getRobotStatus(): Promise<RobotStatus> {
    const response = await api.get('/robot/status');
    return response.data;
  },
};
