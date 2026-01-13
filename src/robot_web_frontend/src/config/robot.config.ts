// Robot configuration
export const ROBOT_CONFIG = {
  // Network
  ROBOT_IP: '192.168.99.54',
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
  CMD_VEL_RATE: 20, // Hz (50ms interval)
};
