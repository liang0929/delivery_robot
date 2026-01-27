// Robot configuration
// ROBOT_IP 優先使用環境變數，否則使用當前頁面的 hostname
const getDefaultRobotIP = (): string => {
  // Vite 環境變數 (build 時設定: VITE_ROBOT_IP=x.x.x.x npm run build)
  if (import.meta.env.VITE_ROBOT_IP) {
    return import.meta.env.VITE_ROBOT_IP;
  }
  // 開發環境或未設定時，使用當前頁面的 hostname
  if (typeof window !== 'undefined' && window.location.hostname) {
    return window.location.hostname;
  }
  // 最後回退到 localhost
  return 'localhost';
};

export const ROBOT_CONFIG = {
  // Network
  ROBOT_IP: getDefaultRobotIP(),
  API_PORT: 8000,
  ROSBRIDGE_PORT: 9090,

  // API URLs
  get API_BASE_URL() {
    return `http://${this.ROBOT_IP}:${this.API_PORT}`;
  },
  get ROSBRIDGE_URL() {
    return `ws://${this.ROBOT_IP}:${this.ROSBRIDGE_PORT}`;
  },

  // Topics
  TOPICS: {
    CMD_VEL: '/cmd_vel',
    MAP: '/map_relay',  // 使用 relay 節點解決 QoS 不相容
    SCAN: '/scan',
    ODOM: '/odometry/filtered',
    ROBOT_POSE: '/amcl_pose',
  },

  // Velocity limits (from nav2_params.yaml)
  MAX_LINEAR_VEL: 0.05,  // m/s
  MAX_ANGULAR_VEL: 0.4,  // rad/s

  // Update rates
  CMD_VEL_RATE: 50, // Hz (20ms interval) - 提升響應速度
};
